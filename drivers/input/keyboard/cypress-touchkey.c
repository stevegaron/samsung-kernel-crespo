/*
 * Copyright 2006-2010, Cypress Semiconductor Corporation.
 * Copyright (C) 2010, Samsung Electronics Co. Ltd. All Rights Reserved.
 * Copyright 2011, Michael Richter (alias neldar)
 * Copyright (C) 2011 <kang@insecure.ws>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor
 * Boston, MA  02110-1301, USA.
 *
 */
#include <linux/module.h>
#ifdef CONFIG_CM7_LED_NOTIFICATION
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#endif
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/input/cypress-touchkey.h>
#include <linux/firmware.h>

#ifdef CONFIG_BLD
#include <linux/bld.h>
#endif

#ifdef CONFIG_SCREEN_DIMMER
#include <linux/screen_dimmer.h>
#endif

#ifdef CONFIG_TOUCH_WAKE
#include <linux/touch_wake.h>
#endif

#define SCANCODE_MASK		0x07
#define UPDOWN_EVENT_MASK	0x08
#define ESD_STATE_MASK		0x10

#define BACKLIGHT_ON		0x10
#define BACKLIGHT_OFF		0x20

#define OLD_BACKLIGHT_ON	0x1
#define OLD_BACKLIGHT_OFF	0x2

#define DEVICE_NAME "cypress-touchkey"

#define FW_SIZE 8192

#ifdef CONFIG_CM7_LED_NOTIFICATION
#include <linux/miscdevice.h>
#define LED_VERSION 2 

static int bl_on = 0;
static bool bBlink = true;
static bool bBlinkTimer = false;
static bool bIsOn = false;
static int iCountsBlink = 0;
static unsigned int iTimeBlink = 200;
static unsigned int iTimesOn = 1;
static unsigned int iTimesTotal = 10;
static unsigned int iBlinkOnOffCounts = 0;
static iBlinkMilisecondsTimeout = 1500;
static DECLARE_MUTEX(enable_sem);
static DECLARE_MUTEX(i2c_sem);
static uint32_t blink_count;

struct cypress_touchkey_devdata *bl_devdata;
static struct timer_list bl_timer;
static void bl_off(struct work_struct *bl_off_work);
static DECLARE_WORK(bl_off_work, bl_off);

static void blink_timer_callback(unsigned long data);
static struct timer_list blink_timer = TIMER_INITIALIZER(blink_timer_callback, 0, 0);
static void blink_callback(struct work_struct *blink_work);
static DECLARE_WORK(blink_work, blink_callback);

static struct wake_lock sBlinkWakeLock;
#else
#ifdef CONFIG_KEYPAD_CYPRESS_TOUCH_USE_BLN
#include <linux/miscdevice.h>
#define BACKLIGHTNOTIFICATION_VERSION 8

bool bln_enabled = false; // indicates if BLN function is enabled/allowed (default: false, app enables it on boot)
bool BacklightNotification_ongoing= false; // indicates ongoing LED Notification
bool bln_blink_enabled = false;	// indicates blink is set
struct cypress_touchkey_devdata *blndevdata; // keep a reference to the devdata
#endif
#endif

struct cypress_touchkey_devdata {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct touchkey_platform_data *pdata;
	struct early_suspend early_suspend;
	u8 backlight_on;
	u8 backlight_off;
	bool is_dead;
	bool is_powering_on;
	bool has_legacy_keycode;
#ifdef CONFIG_CM7_LED_NOTIFICATION
	bool is_sleeping;
#endif
};

#ifdef CONFIG_BLD
static struct cypress_touchkey_devdata *blddevdata;
#endif

#ifdef CONFIG_TOUCH_WAKE
static struct cypress_touchkey_devdata *touchwakedevdata;
#endif

static int i2c_touchkey_read_byte(struct cypress_touchkey_devdata *devdata,
					u8 *val)
{
	int ret;
	int retry = 5;

#ifdef CONFIG_CM7_LED_NOTIFICATION
	down(&i2c_sem);
#endif

	while (true) {
		ret = i2c_smbus_read_byte(devdata->client);
		if (ret >= 0) {
			*val = ret;
#ifdef CONFIG_CM7_LED_NOTIFICATION
			ret = 0;
			break;
#else
			return 0;
#endif
		}

		dev_err(&devdata->client->dev, "i2c read error\n");
		if (!retry--)
			break;
		msleep(10);
	}

#ifdef CONFIG_CM7_LED_NOTIFICATION
	up(&i2c_sem);
#endif

	return ret;
}

static int i2c_touchkey_write_byte(struct cypress_touchkey_devdata *devdata,
					u8 val)
{
	int ret;
	int retry = 2;

#ifdef CONFIG_CM7_LED_NOTIFICATION
	down(&i2c_sem);
#endif

	while (true) {
		ret = i2c_smbus_write_byte(devdata->client, val);
#ifdef CONFIG_CM7_LED_NOTIFICATION
		if (!ret) {
		 	 ret = 0;
		 	 break;
		 	 }
#else
		if (!ret)
			return 0;
#endif

		dev_err(&devdata->client->dev, "i2c write error\n");
		if (!retry--)
			break;
		msleep(10);
	}

#ifdef CONFIG_CM7_LED_NOTIFICATION
	up(&i2c_sem);
#endif

	return ret;
}

static void all_keys_up(struct cypress_touchkey_devdata *devdata)
{
	int i;

	for (i = 0; i < devdata->pdata->keycode_cnt; i++)
		input_report_key(devdata->input_dev,
						devdata->pdata->keycode[i], 0);

	input_sync(devdata->input_dev);
}

#ifdef CONFIG_CM7_LED_NOTIFICATION
static void bl_off(struct work_struct *bl_off_work)
{
 	if (bl_devdata == NULL || unlikely(bl_devdata->is_dead) ||
 		bl_devdata->is_powering_on || bl_on >= 1 || bl_devdata->is_sleeping)
 		return;
 	 
	i2c_touchkey_write_byte(bl_devdata, bl_devdata->backlight_off);
}
 	 
void bl_timer_callback(unsigned long data)
{
	 schedule_work(&bl_off_work);
}
#endif

static int recovery_routine(struct cypress_touchkey_devdata *devdata)
{
	int ret = -1;
	int retry = 10;
	u8 data;
	int irq_eint;

	if (unlikely(devdata->is_dead)) {
		dev_err(&devdata->client->dev, "%s: Device is already dead, "
				"skipping recovery\n", __func__);
		return -ENODEV;
	}

	irq_eint = devdata->client->irq;

#ifdef CONFIG_CM7_LED_NOTIFICATION
	down(&enable_sem);
#endif

	all_keys_up(devdata);

	disable_irq_nosync(irq_eint);
	while (retry--) {
		devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
		devdata->pdata->touchkey_onoff(TOUCHKEY_ON);
		ret = i2c_touchkey_read_byte(devdata, &data);
		if (!ret) {
#ifdef CONFIG_CM7_LED_NOTIFICATION
			if (!devdata->is_sleeping)
				enable_irq(irq_eint);
#else
			enable_irq(irq_eint);
#endif
			goto out;
		}
		dev_err(&devdata->client->dev, "%s: i2c transfer error retry = "
				"%d\n", __func__, retry);
	}
	devdata->is_dead = true;
	devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
	dev_err(&devdata->client->dev, "%s: touchkey died\n", __func__);
out:
#ifdef CONFIG_CM7_LED_NOTIFICATION
	up(&enable_sem);
#endif
	return ret;
}

static irqreturn_t touchkey_interrupt_thread(int irq, void *touchkey_devdata)
{
	u8 data;
	int i;
	int ret;
	int scancode;
	struct cypress_touchkey_devdata *devdata = touchkey_devdata;

	ret = i2c_touchkey_read_byte(devdata, &data);
	if (ret || (data & ESD_STATE_MASK)) {
		ret = recovery_routine(devdata);
		if (ret) {
			dev_err(&devdata->client->dev, "%s: touchkey recovery "
					"failed!\n", __func__);
			goto err;
		}
	}

	if (devdata->has_legacy_keycode) {
		scancode = (data & SCANCODE_MASK) - 1;
		if (scancode < 0 || scancode >= devdata->pdata->keycode_cnt) {
			dev_err(&devdata->client->dev, "%s: scancode is out of "
				"range\n", __func__);
			goto err;
		}

#if defined(CONFIG_TOUCH_WAKE) || defined(CONFIG_SCREEN_DIMMER) || defined(CONFIG_BLD)
#ifdef CONFIG_SCREEN_DIMMER
#ifdef CONFIG_TOUCH_WAKE
		if (!device_is_suspended() && !screen_is_dimmed())
#else
		if (!screen_is_dimmed())
#endif
#else
#ifdef CONFIG_TOUCH_WAKE
		if (!device_is_suspended())
#endif
#endif
		    {
			input_report_key(devdata->input_dev,
					 devdata->pdata->keycode[scancode],
					 !(data & UPDOWN_EVENT_MASK));
		    }

		if (!(data & UPDOWN_EVENT_MASK))
		    {
#ifdef CONFIG_BLD			
			touchkey_pressed();
#endif
#ifdef CONFIG_SCREEN_DIMMER
			touchscreen_pressed();
#endif
#ifdef CONFIG_TOUCH_WAKE
			touch_press();
#endif
		    }
#else
		input_report_key(devdata->input_dev,
		 	devdata->pdata->keycode[scancode],
		 	!(data & UPDOWN_EVENT_MASK));
#endif
	} else {
#if defined(CONFIG_TOUCH_WAKE) || defined(CONFIG_SCREEN_DIMMER) || defined(CONFIG_BLD)
#ifdef CONFIG_SCREEN_DIMMER
#ifdef CONFIG_TOUCH_WAKE
		if (!device_is_suspended() && !screen_is_dimmed())
#else
		if (!screen_is_dimmed())
#endif
#else
#ifdef CONFIG_TOUCH_WAKE
		if (!device_is_suspended())
#endif
#endif
		    {
			for (i = 0; i < devdata->pdata->keycode_cnt; i++)
			    input_report_key(devdata->input_dev,
					     devdata->pdata->keycode[i],
					     !!(data & (1U << i)));
		    }

		for (i = 0; i < devdata->pdata->keycode_cnt; i++)
		    {
			if(!!(data & (1U << i)))
			    {
#ifdef CONFIG_BLD			
				touchkey_pressed();
#endif
#ifdef CONFIG_SCREEN_DIMMER
				touchscreen_pressed();
#endif
#ifdef CONFIG_TOUCH_WAKE
				touch_press();
#endif
				break;
			    }
		    }
#else
		for (i = 0; i < devdata->pdata->keycode_cnt; i++)
		{
			input_report_key(devdata->input_dev,
			 	 devdata->pdata->keycode[i],
			 	 !!(data & (1U << i)));
		}
#endif
	}

	input_sync(devdata->input_dev);
#ifdef CONFIG_CM7_LED_NOTIFICATION
	if ( iBlinkOnOffCounts > 0 )
		mod_timer(&bl_timer, jiffies + msecs_to_jiffies(iBlinkOnOffCounts));
#endif
err:
	return IRQ_HANDLED;
}

static irqreturn_t touchkey_interrupt_handler(int irq, void *touchkey_devdata)
{
	struct cypress_touchkey_devdata *devdata = touchkey_devdata;

	if (devdata->is_powering_on) {
		dev_dbg(&devdata->client->dev, "%s: ignoring spurious boot "
					"interrupt\n", __func__);
		return IRQ_HANDLED;
	}

	return IRQ_WAKE_THREAD;
}

#ifdef CONFIG_CM7_LED_NOTIFICATION
static void notify_led_on(void) {
	if (unlikely(bl_devdata->is_dead))
		return;

	if (bl_devdata->is_sleeping) {
		bl_devdata->pdata->touchkey_sleep_onoff(TOUCHKEY_ON);
		bl_devdata->pdata->touchkey_onoff(TOUCHKEY_ON);
	}
	i2c_touchkey_write_byte(bl_devdata, bl_devdata->backlight_on);
	bIsOn = true;
}

static void notify_led_off(void) {
	if (unlikely(bl_devdata->is_dead))
		return;

	bl_devdata->pdata->touchkey_sleep_onoff(TOUCHKEY_OFF);
	if (bl_devdata->is_sleeping)
		bl_devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
	
	if (bl_on != 1) // Don't disable if there's a timer scheduled
		i2c_touchkey_write_byte(bl_devdata, bl_devdata->backlight_off);

	bIsOn = false;
}

static void blink_callback(struct work_struct *blink_work)
{
	if ( bl_on == 0) {
		printk(KERN_DEBUG "%s: ERROR notification BLINK ENTER without CALL 0 \n", __FUNCTION__);
		// notify_led_off();
		del_timer(&blink_timer);
		wake_unlock(&sBlinkWakeLock);
		return;
	}

	if ( bl_on == 2) {
		printk(KERN_DEBUG "%s: ERROR notification BLINK ENTER without CALL 2\n", __FUNCTION__);
		notify_led_on();
		del_timer(&blink_timer);
		wake_unlock(&sBlinkWakeLock);
		bBlinkTimer = false;
		return;
	}

	if (--blink_count == 0 && iBlinkMilisecondsTimeout != 0) {
		notify_led_on();
		if ( bBlinkTimer ) {
			bBlinkTimer = false;
			bl_on = 2;
			del_timer(&blink_timer);
			wake_unlock(&sBlinkWakeLock);
		}
		return;
	}

	if (iCountsBlink++ < iTimesOn) {
		if (!bIsOn)
			notify_led_on();
	} else {
		if (bIsOn)
			notify_led_off();
	}
	
	if ( iCountsBlink >= iTimesTotal)
		iCountsBlink = 0;
}	

static void blink_timer_callback(unsigned long data)
{
	schedule_work(&blink_work);
	mod_timer(&blink_timer, jiffies + msecs_to_jiffies(iTimeBlink));
}

#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cypress_touchkey_early_suspend(struct early_suspend *h)
{
	struct cypress_touchkey_devdata *devdata =
		container_of(h, struct cypress_touchkey_devdata, early_suspend);

#ifdef CONFIG_CM7_LED_NOTIFICATION
	down(&enable_sem);
#endif

#ifdef CONFIG_TOUCH_WAKE
	i2c_touchkey_write_byte(devdata, devdata->backlight_off);
#else
	devdata->is_powering_on = true;

#ifdef CONFIG_CM7_LED_NOTIFICATION
	if (unlikely(devdata->is_dead)){
		up(&enable_sem);	
		return;
	}
#else
	if (unlikely(devdata->is_dead))
		return;
#endif

	disable_irq(devdata->client->irq);

#ifdef CONFIG_KEYPAD_CYPRESS_TOUCH_USE_BLN
	/*
	 * Disallow powering off the touchkey controller
	 * while a led notification is ongoing
	 */
	if(!BacklightNotification_ongoing)
#endif
#ifdef CONFIG_CM7_LED_NOTIFICATION
	if(bl_on == 0)
#endif
	devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);

	all_keys_up(devdata);
#ifdef CONFIG_CM7_LED_NOTIFICATION
	devdata->is_sleeping = true;
	
	up(&enable_sem);
#endif
#endif
}

static void cypress_touchkey_early_resume(struct early_suspend *h)
{
	struct cypress_touchkey_devdata *devdata =
		container_of(h, struct cypress_touchkey_devdata, early_suspend);

#ifdef CONFIG_CM7_LED_NOTIFICATION
	// Avoid race condition with LED notification disable
	 down(&enable_sem);
#endif

#ifdef CONFIG_TOUCH_WAKE
	i2c_touchkey_write_byte(devdata, devdata->backlight_on);
#else
	devdata->pdata->touchkey_onoff(TOUCHKEY_ON);
	if (i2c_touchkey_write_byte(devdata, devdata->backlight_on)) {
		devdata->is_dead = true;
		devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
		dev_err(&devdata->client->dev, "%s: touch keypad not responding"
				" to commands, disabling\n", __func__);
#ifdef CONFIG_CM7_LED_NOTIFICATION
		up(&enable_sem);
#endif
		return;
	}
	devdata->is_dead = false;
	enable_irq(devdata->client->irq);
	devdata->is_powering_on = false;
#ifdef CONFIG_CM7_LED_NOTIFICATION
	devdata->is_sleeping = false;

	up(&enable_sem);

	if ( iBlinkOnOffCounts > 0 )
 		mod_timer(&bl_timer, jiffies + msecs_to_jiffies(iBlinkOnOffCounts));
#endif
#endif
}
#endif

#ifdef CONFIG_CM7_LED_NOTIFICATION
static ssize_t led_status_read(struct device *dev, struct device_attribute *attr, char *buf) {
	return sprintf(buf,"%u\n", bl_on);
}

static ssize_t led_status_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int data;

	printk(KERN_DEBUG "%s: notification LED ENTER\n", __FUNCTION__);
	if (sscanf(buf, "%u\n", &data)) {
		if (data == 0 && bl_on == 0)
			return;
 	 	if (data >= 1) {
			all_keys_up(bl_devdata);
			// printk(KERN_DEBUG "%s: notification led enabled\n", __FUNCTION__);
			if (bBlink && data == 1 && bl_on != 1) {
				// printk(KERN_DEBUG "%s: notification led TIMER enabled\n", __FUNCTION__);
				wake_lock(&sBlinkWakeLock);
				blink_timer.expires = jiffies + msecs_to_jiffies(iTimeBlink);
				blink_count = iBlinkMilisecondsTimeout;
				add_timer(&blink_timer);
				bBlinkTimer = true;
			}
	 		bl_on = 1;
			notify_led_on();
		}else {
			// printk(KERN_DEBUG "%s: notification led dissabled\n", __FUNCTION__);
			bl_on = 0;
			notify_led_off();
			if ( bBlinkTimer ) {
				bBlinkTimer = false;
				del_timer(&blink_timer);
				wake_unlock(&sBlinkWakeLock);
			}
		}
	}
	return size;
}


static ssize_t led_timeout_read(struct device *dev, struct device_attribute *attr, char *buf) {
	unsigned int iSeconds;
	
	iSeconds = iBlinkOnOffCounts / 1000;
	return sprintf(buf,"%u\n", iSeconds);
}
	
static ssize_t led_timeout_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int data;

	if (sscanf(buf, "%u\n", &data)) {
		iBlinkOnOffCounts = data * 1000;
	}
	return size;
}

static ssize_t led_blinktimeout_read(struct device *dev, struct device_attribute *attr, char *buf) {
	unsigned int iMinutes;

	iMinutes = iBlinkMilisecondsTimeout / 300;
	return sprintf(buf,"%u\n", iMinutes);
}

static ssize_t led_blinktimeout_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int data;

	if (sscanf(buf, "%u\n", &data)) {
		iBlinkMilisecondsTimeout = data * 300;
	}
	mod_timer(&blink_timer, jiffies + msecs_to_jiffies(iTimeBlink));
	return size;
}

static ssize_t led_blink_read(struct device *dev, struct device_attribute *attr, char *buf) {
	return sprintf(buf,"%u\n", bBlink);
}

static ssize_t led_blink_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int data;

	if (sscanf(buf, "%u\n", &data)) {
		if (data == 0)
			bBlink = false;
		else
			bBlink = true;
	}
	return size;
}

static ssize_t led_version_read(struct device * dev, struct device_attribute * attr, char * buf)
{
	return sprintf(buf, "%u\n", LED_VERSION);
}

static DEVICE_ATTR(led, S_IRUGO | S_IWUGO , led_status_read, led_status_write);
static DEVICE_ATTR(timeout, S_IRUGO | S_IWUGO , led_timeout_read, led_timeout_write);
static DEVICE_ATTR(blinktimeout, S_IRUGO | S_IWUGO , led_blinktimeout_read, led_blinktimeout_write);
static DEVICE_ATTR(blink, S_IRUGO | S_IWUGO , led_blink_read, led_blink_write);
static DEVICE_ATTR(version, S_IRUGO , led_version_read, NULL);

static struct attribute *bl_led_attributes[] = {
	&dev_attr_led.attr,
	&dev_attr_timeout.attr,
	&dev_attr_blinktimeout.attr,
	&dev_attr_blink.attr,
 	&dev_attr_version.attr,
	NULL
};

static struct attribute_group bl_led_group = {
	.attrs  = bl_led_attributes,
};

static struct miscdevice bl_led_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "notification",
};
#endif

static void set_device_params(struct cypress_touchkey_devdata *devdata,
								u8 *data)
{
	if (data[1] < 0xc4 && (data[1] > 0x8 ||
				(data[1] == 0x8 && data[2] >= 0x9))) {
		devdata->backlight_on = BACKLIGHT_ON;
		devdata->backlight_off = BACKLIGHT_OFF;
	} else {
		devdata->backlight_on = OLD_BACKLIGHT_ON;
		devdata->backlight_off = OLD_BACKLIGHT_OFF;
	}

	devdata->has_legacy_keycode = data[1] >= 0xc4 || data[1] < 0x9 ||
					(data[1] == 0x9 && data[2] < 0x9);
}

static int update_firmware(struct cypress_touchkey_devdata *devdata)
{
	int ret;
	const struct firmware *fw = NULL;
	struct device *dev = &devdata->input_dev->dev;
	int retries = 10;

	dev_info(dev, "%s: Updating " DEVICE_NAME " firmware\n", __func__);

	if (!devdata->pdata->fw_name) {
		dev_err(dev, "%s: Device firmware name is not set\n", __func__);
		return -EINVAL;
	}

	ret = request_firmware(&fw, devdata->pdata->fw_name, dev);
	if (ret) {
		dev_err(dev, "%s: Can't open firmware file from %s\n", __func__,
						devdata->pdata->fw_name);
		return ret;
	}

	if (fw->size != FW_SIZE) {
		dev_err(dev, "%s: Firmware file size invalid\n", __func__);
		return -EINVAL;
	}

	disable_irq(devdata->client->irq);
	/* Lock the i2c bus since the firmware updater accesses it */
	i2c_lock_adapter(devdata->client->adapter);
	while (retries--) {
		ret = touchkey_flash_firmware(devdata->pdata, fw->data);
		if (!ret)
			break;
	}
	if (ret)
		dev_err(dev, "%s: Firmware update failed\n", __func__);
	i2c_unlock_adapter(devdata->client->adapter);
	enable_irq(devdata->client->irq);

	release_firmware(fw);
	return ret;
}

static int cypress_touchkey_open(struct input_dev *input_dev)
{
	struct device *dev = &input_dev->dev;
	struct cypress_touchkey_devdata *devdata = dev_get_drvdata(dev);
	u8 data[3];
	int ret;

	ret = update_firmware(devdata);
	if (ret)
		goto done;

	ret = i2c_master_recv(devdata->client, data, sizeof(data));
	if (ret < sizeof(data)) {
		if (ret >= 0)
			ret = -EIO;
		dev_err(dev, "%s: error reading hardware version\n", __func__);
		goto done;
	}

	dev_info(dev, "%s: hardware rev1 = %#02x, rev2 = %#02x\n", __func__,
				data[1], data[2]);
	set_device_params(devdata, data);

	devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
	devdata->pdata->touchkey_onoff(TOUCHKEY_ON);

	ret = i2c_touchkey_write_byte(devdata, devdata->backlight_on);
	if (ret) {
		dev_err(dev, "%s: touch keypad backlight on failed\n",
				__func__);
		goto done;
	}

done:
	input_dev->open = NULL;
	return 0;
}

#ifdef CONFIG_TOUCH_WAKE
static void cypress_touchwake_disable(void)
{
    touchwakedevdata->is_powering_on = true;

    if (unlikely(touchwakedevdata->is_dead))
	return;

    disable_irq(touchwakedevdata->client->irq);

    touchwakedevdata->pdata->touchkey_onoff(TOUCHKEY_OFF);

    all_keys_up(touchwakedevdata);

    return;
}

static void cypress_touchwake_enable(void)
{
    touchwakedevdata->pdata->touchkey_onoff(TOUCHKEY_ON);
    if (i2c_touchkey_write_byte(touchwakedevdata, touchwakedevdata->backlight_on)) {
	touchwakedevdata->is_dead = true;
	touchwakedevdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
	dev_err(&touchwakedevdata->client->dev, "%s: touch keypad not responding"
		" to commands, disabling\n", __func__);
	return;
    }
    touchwakedevdata->is_dead = false;
    enable_irq(touchwakedevdata->client->irq);
    touchwakedevdata->is_powering_on = false;

    return;
}

static struct touchwake_implementation cypress_touchwake = 
    {
	.enable = cypress_touchwake_enable,
	.disable = cypress_touchwake_disable,
    };
#endif

#ifdef CONFIG_BLD
static void cypress_touchkey_bld_disable(void)
{
    i2c_touchkey_write_byte(blddevdata, blddevdata->backlight_off);
}

static void cypress_touchkey_bld_enable(void)
{
    i2c_touchkey_write_byte(blddevdata, blddevdata->backlight_on);
}

static struct bld_implementation cypress_touchkey_bld = 
    {
	.enable = cypress_touchkey_bld_enable,
	.disable = cypress_touchkey_bld_disable,
    };
#endif

#ifdef CONFIG_KEYPAD_CYPRESS_TOUCH_USE_BLN
/* bln start */

static void enable_touchkey_backlights(void){
	i2c_touchkey_write_byte(blndevdata, blndevdata->backlight_on);
}

static void disable_touchkey_backlights(void){
	i2c_touchkey_write_byte(blndevdata, blndevdata->backlight_off);
}

static void enable_led_notification(void){
	if (bln_enabled){
		/* is_powering_on signals whether touchkey lights are used for touchmode */
		if (blndevdata->is_powering_on){
			/* signal ongoing led notification */
			BacklightNotification_ongoing = true;

			/* reconfigure gpio for sleep mode */
			blndevdata->pdata->touchkey_sleep_onoff(TOUCHKEY_ON);

			/*
			 * power on the touchkey controller
			 * This is actually not needed, but it is intentionally
			 * left for the case that the early_resume() function
			 * did not power on the touchkey controller for some reasons
			 */
			blndevdata->pdata->touchkey_onoff(TOUCHKEY_ON);

			/* write to i2cbus, enable backlights */
			enable_touchkey_backlights();

			pr_info("%s: notification led enabled\n", __FUNCTION__);
		}
		else
			pr_info("%s: cannot set notification led, touchkeys are enabled\n",__FUNCTION__);
	}
}

static void disable_led_notification(void){
	pr_info("%s: notification led disabled\n", __FUNCTION__);

	/* disable the blink state */
	bln_blink_enabled = false;

	/*
	 * reconfigure gpio for sleep mode, this has to be done
	 * independently from the power status
	 */
	blndevdata->pdata->touchkey_sleep_onoff(TOUCHKEY_OFF);

	/* if touchkeys lights are not used for touchmode */
	if (blndevdata->is_powering_on){
		disable_touchkey_backlights();

		#if 0
		/*
		 * power off the touchkey controller
		 * This is actually not needed, the early_suspend function
		 * should take care of powering off the touchkey controller
		 */
		blndevdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
		#endif
	}

	/* signal led notification is disabled */
	BacklightNotification_ongoing = false;
}

static ssize_t backlightnotification_status_read(struct device *dev, struct device_attribute *attr, char *buf) {
    return sprintf(buf,"%u\n",(bln_enabled ? 1 : 0));
}
static ssize_t backlightnotification_status_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int data;
	if(sscanf(buf, "%u\n", &data) == 1) {
		pr_devel("%s: %u \n", __FUNCTION__, data);
		if(data == 0 || data == 1){

			if(data == 1){
				pr_info("%s: backlightnotification function enabled\n", __FUNCTION__);
				bln_enabled = true;
			}

			if(data == 0){
				pr_info("%s: backlightnotification function disabled\n", __FUNCTION__);
				bln_enabled = false;
				if (BacklightNotification_ongoing)
					disable_led_notification();
			}
		}
		else
			pr_info("%s: invalid input range %u\n", __FUNCTION__, data);
	}
	else
		pr_info("%s: invalid input\n", __FUNCTION__);

	return size;
}

static ssize_t notification_led_status_read(struct device *dev, struct device_attribute *attr, char *buf) {
	return sprintf(buf,"%u\n", (BacklightNotification_ongoing ? 1 : 0));
}

static ssize_t notification_led_status_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int data;

	if(sscanf(buf, "%u\n", &data) == 1) {
		if(data == 0 || data == 1){
			pr_devel("%s: %u \n", __FUNCTION__, data);
			if (data == 1)
				enable_led_notification();

			if(data == 0)
				disable_led_notification();

		} else
			pr_info("%s: wrong input %u\n", __FUNCTION__, data);
	} else
		pr_info("%s: input error\n", __FUNCTION__);

	return size;
}

static ssize_t blink_control_read(struct device *dev, struct device_attribute *attr, char *buf) {
	return sprintf(buf,"%u\n", (bln_blink_enabled ? 1 : 0));
}

static ssize_t blink_control_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int data;

	if(sscanf(buf, "%u\n", &data) == 1) {
		if(data == 0 || data == 1){
			if (BacklightNotification_ongoing){
				pr_devel("%s: %u \n", __FUNCTION__, data);
				if (data == 1){
					bln_blink_enabled = true;
					disable_touchkey_backlights();
				}

				if(data == 0){
					bln_blink_enabled = false;
					enable_touchkey_backlights();
				}
			}

		} else
			pr_info("%s: wrong input %u\n", __FUNCTION__, data);
	} else
		pr_info("%s: input error\n", __FUNCTION__);

	return size;
}

static ssize_t backlightnotification_version(struct device *dev, struct device_attribute *attr, char *buf) {
	return sprintf(buf, "%u\n", BACKLIGHTNOTIFICATION_VERSION);
}

static DEVICE_ATTR(blink_control, S_IRUGO | S_IWUGO , blink_control_read, blink_control_write);
static DEVICE_ATTR(enabled, S_IRUGO | S_IWUGO , backlightnotification_status_read, backlightnotification_status_write);
static DEVICE_ATTR(notification_led, S_IRUGO | S_IWUGO , notification_led_status_read, notification_led_status_write);
static DEVICE_ATTR(version, S_IRUGO , backlightnotification_version, NULL);

static struct attribute *bln_notification_attributes[] = {
		&dev_attr_blink_control.attr,
		&dev_attr_enabled.attr,
		&dev_attr_notification_led.attr,
		&dev_attr_version.attr,
		NULL
};

static struct attribute_group bln_notification_group = {
		.attrs  = bln_notification_attributes,
};

static struct miscdevice backlightnotification_device = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "backlightnotification",
};
/* bln end */
#endif

static int cypress_touchkey_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct input_dev *input_dev;
	struct cypress_touchkey_devdata *devdata;
	u8 data[3];
	int err;
	int cnt;

	if (!dev->platform_data) {
		dev_err(dev, "%s: Platform data is NULL\n", __func__);
		return -EINVAL;
	}

	devdata = kzalloc(sizeof(*devdata), GFP_KERNEL);
	if (devdata == NULL) {
		dev_err(dev, "%s: failed to create our state\n", __func__);
		return -ENODEV;
	}

	devdata->client = client;
	i2c_set_clientdata(client, devdata);

	devdata->pdata = client->dev.platform_data;
	if (!devdata->pdata->keycode) {
		dev_err(dev, "%s: Invalid platform data\n", __func__);
		err = -EINVAL;
		goto err_null_keycodes;
	}

	strlcpy(devdata->client->name, DEVICE_NAME, I2C_NAME_SIZE);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		goto err_input_alloc_dev;
	}

	devdata->input_dev = input_dev;
	dev_set_drvdata(&input_dev->dev, devdata);
	input_dev->name = DEVICE_NAME;
	input_dev->id.bustype = BUS_HOST;

	for (cnt = 0; cnt < devdata->pdata->keycode_cnt; cnt++)
		input_set_capability(input_dev, EV_KEY,
					devdata->pdata->keycode[cnt]);

	devdata->is_powering_on = true;
#ifdef CONFIG_CM7_LED_NOTIFICATION
	devdata->is_sleeping = false;
#endif

	devdata->pdata->touchkey_onoff(TOUCHKEY_ON);

	err = i2c_master_recv(client, data, sizeof(data));
	if (err < sizeof(data)) {
		if (err >= 0)
			err = -EIO;
		dev_err(dev, "%s: error reading hardware version\n", __func__);
		goto err_read;
	}

	dev_info(dev, "%s: hardware rev1 = %#02x, rev2 = %#02x\n", __func__,
				data[1], data[2]);

	if (data[1] != 0xa || data[2] < 0x9)
		input_dev->open = cypress_touchkey_open;

	err = input_register_device(input_dev);
	if (err)
		goto err_input_reg_dev;

	set_device_params(devdata, data);

#ifdef CONFIG_CM7_LED_NOTIFICATION
	err = i2c_touchkey_write_byte(devdata, devdata->backlight_off);
#else
	err = i2c_touchkey_write_byte(devdata, devdata->backlight_on);
#endif
	if (err) {
		dev_err(dev, "%s: touch keypad backlight on failed\n",
				__func__);
		/* The device may not be responding because of bad firmware
		 * Allow the firmware to be reflashed if it needs to be
		 */
		if (!input_dev->open)
#ifdef CONFIG_CM7_LED_NOTIFICATION
			goto err_backlight_off;
#else
			goto err_backlight_on;
#endif
	}

	err = request_threaded_irq(client->irq, touchkey_interrupt_handler,
				touchkey_interrupt_thread, IRQF_TRIGGER_FALLING,
				DEVICE_NAME, devdata);
	if (err) {
		dev_err(dev, "%s: Can't allocate irq.\n", __func__);
		goto err_req_irq;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	devdata->early_suspend.suspend = cypress_touchkey_early_suspend;
	devdata->early_suspend.resume = cypress_touchkey_early_resume;
#endif
	register_early_suspend(&devdata->early_suspend);

	devdata->is_powering_on = false;

#ifdef CONFIG_CM7_LED_NOTIFICATION
	if (misc_register(&bl_led_device))
		printk("%s misc_register(%s) failed\n", __FUNCTION__, bl_led_device.name);
	else {
		bl_devdata = devdata;
		if (sysfs_create_group(&bl_led_device.this_device->kobj, &bl_led_group) < 0)
			pr_err("failed to create sysfs group for device %s\n", bl_led_device.name);
	}
#endif

#ifdef CONFIG_BLD
	blddevdata = devdata;
	register_bld_implementation(&cypress_touchkey_bld);
#endif

#ifdef CONFIG_TOUCH_WAKE
	touchwakedevdata = devdata;
	register_touchwake_implementation(&cypress_touchwake);
#endif

#ifdef CONFIG_KEYPAD_CYPRESS_TOUCH_USE_BLN
	pr_info("%s misc_register(%s)\n", __FUNCTION__, backlightnotification_device.name);
	err = misc_register(&backlightnotification_device);
	if (err) {
		pr_err("%s misc_register(%s) fail\n", __FUNCTION__, backlightnotification_device.name);
	}else {
		/*
		 *  keep a reference to the devdata,
		 *  misc driver does not give access to it (or i did miss that somewhere)
		 */
		blndevdata = devdata;

		/* add the backlightnotification attributes */
		if (sysfs_create_group(&backlightnotification_device.this_device->kobj, &bln_notification_group) < 0)
		{
			pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
			pr_err("Failed to create sysfs group for device (%s)!\n", backlightnotification_device.name);
		}
	}
#endif

	return 0;


err_req_irq:
#ifdef CONFIG_CM7_LED_NOTIFICATION
err_backlight_off:
#else
err_backlight_on:
#endif
	input_unregister_device(input_dev);
	goto touchkey_off;
err_input_reg_dev:
err_read:
	input_free_device(input_dev);
touchkey_off:
#ifdef CONFIG_CM7_LED_NOTIFICATION
	devdata->is_powering_on = false;
#endif
	devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
err_input_alloc_dev:
err_null_keycodes:
	kfree(devdata);
	return err;
}

static int __devexit i2c_touchkey_remove(struct i2c_client *client)
{
	struct cypress_touchkey_devdata *devdata = i2c_get_clientdata(client);

#ifdef CONFIG_KEYPAD_CYPRESS_TOUCH_USE_BLN
	misc_deregister(&backlightnotification_device);
#endif
#ifdef CONFIG_CM7_LED_NOTIFICATION
	misc_deregister(&bl_led_device);
#endif

	unregister_early_suspend(&devdata->early_suspend);
	/* If the device is dead IRQs are disabled, we need to rebalance them */
	if (unlikely(devdata->is_dead))
		enable_irq(client->irq);
#ifdef CONFIG_CM7_LED_NOTIFICATION
	else{
		devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
		devdata->is_powering_on = false;
}
#else
	else
		devdata->pdata->touchkey_onoff(TOUCHKEY_OFF);
#endif
	free_irq(client->irq, devdata);
	all_keys_up(devdata);
	input_unregister_device(devdata->input_dev);
#ifdef CONFIG_CM7_LED_NOTIFICATION
	del_timer(&bl_timer);
	if ( bBlinkTimer ) {
		bBlinkTimer = false;
		del_timer(&blink_timer);
	}
#endif
	kfree(devdata);
	return 0;
}

static const struct i2c_device_id cypress_touchkey_id[] = {
	{ CYPRESS_TOUCHKEY_DEV_NAME, 0 },
};

MODULE_DEVICE_TABLE(i2c, cypress_touchkey_id);

struct i2c_driver touchkey_i2c_driver = {
	.driver = {
		.name = "cypress_touchkey_driver",
	},
	.id_table = cypress_touchkey_id,
	.probe = cypress_touchkey_probe,
	.remove = __devexit_p(i2c_touchkey_remove),
};

static int __init touchkey_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&touchkey_i2c_driver);
	if (ret)
		pr_err("%s: cypress touch keypad registration failed. (%d)\n",
				__func__, ret);

	/* Initialize wake locks */
	wake_lock_init(&sBlinkWakeLock, WAKE_LOCK_SUSPEND, "blink_wake");

	setup_timer(&bl_timer, bl_timer_callback, 0);
	setup_timer(&blink_timer, blink_timer_callback, 0);
	mod_timer(&blink_timer, jiffies + msecs_to_jiffies(iTimeBlink));

	return ret;
}

static void __exit touchkey_exit(void)
{
	i2c_del_driver(&touchkey_i2c_driver);
}

late_initcall(touchkey_init);
module_exit(touchkey_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("@@@");
MODULE_DESCRIPTION("cypress touch keypad");
