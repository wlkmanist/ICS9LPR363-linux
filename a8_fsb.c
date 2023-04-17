/*
 *   FSB control driver for Asus A8E/A8S/A8J.
 *   Compatible with ICS9LPR363(DGLF) PLL.
 *   Author: wlkmanist <vlad.king5555@mail.ru>, (C) 2023
 *           t.me/wlkmanist
 *
 *   black.electronics
 * 
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/types.h>

MODULE_AUTHOR("wlkmanist");
MODULE_DESCRIPTION("ICS9LPR363 PLL device driver");
MODULE_LICENSE("GPL");

#define BYTECOUNT           21
#define CMD                 0x00
#define FSB_MAX             369000
#define FSB_MIN             94500

static uint8_t i2c_addr =   0x69;
static int i2c_adapter =    0;
static int pll_m =          8; 	// =8 by default (94.5-369MHz range)

struct i2c_adapter *adapter;
struct i2c_client  *client;
static int fsb_freq = -1; // KHz
static int pll_step = -1;
static uint16_t raw_ctrl_word = 0;
static int pll_m_actual = -1;
static bool pll_m_unlock = false;

enum SetLogic
{
    byPllStep = 1,
    byRaw,
};

static void set_by_pll_step_logic(unsigned char * buf)
{

	if (pll_step > 0x1FF)
		buf[0x0B] = 0xC0;
	else if (pll_step > 0xFF)
		buf[0x0B] = 0x40;
	else
		buf[0x0B] = 0x80;
	buf[0x0B] |= (pll_m & 0x3F);
	buf[0x0C] = pll_step & 0xFF;
}

static void set_by_raw_logic(unsigned char * buf)
{
	buf[0x0B] = (raw_ctrl_word >> 8) & 0xFF;
	buf[0x0C] = raw_ctrl_word & 0xFF;
}

static int ics9lpr363_SetFSB(struct device *dev, enum SetLogic setLogic)
{
	int res;
	unsigned char buf[32];

    // Get a pointer to the i2c adapter
    adapter = i2c_get_adapter(i2c_adapter);
    if (!adapter)
    {
        dev_err(dev, "Failed to get i2c adapter\n");
        return -ENODEV;
    }

    // Dynamically create an i2c_client for the slave device
    client = i2c_new_device(adapter, &(struct i2c_board_info)
    {
        I2C_BOARD_INFO("ics9lpr363", i2c_addr),
    });
    if (!client) {
        dev_err(dev, "Failed to create i2c_client\n");
        i2c_put_adapter(adapter);
        return -ENODEV;
    }

	res = i2c_smbus_read_block_data(client, CMD, buf);
	if(res < 0)
    {
        dev_err(dev, "Failed to read data from 0x%02x on i2c-%d\n",
                        client->addr, adapter->nr);
        i2c_unregister_device(client);
        i2c_put_adapter(adapter);
        return -1;
    }

    switch (setLogic)
    {
        case((enum SetLogic)byPllStep):
            // pll_step is already constrained to 0-767
            set_by_pll_step_logic(buf);
        break;

        case((enum SetLogic)byRaw):
            set_by_raw_logic(buf);
        break;
        
        default:
        break;
    }

	res = i2c_smbus_write_block_data(client, CMD, BYTECOUNT, buf);

    i2c_unregister_device(client);
    i2c_put_adapter(adapter);

    if(res < 0)
    {
        dev_err(dev, "Failed to write data to 0x%02x on i2c-%d\n",
                        i2c_addr, adapter->nr);
	    return -1;
    }
    
	return 0;
}

// reads values from PLL registers to pll_step and fsb_freq
static int ics9lpr363_GetFSB(struct device *dev)
{
	int res;
	unsigned char buf[32];
  	int step_mult;

    pll_step = -1;
    fsb_freq = -1;
    raw_ctrl_word = 0;
    pll_m_actual = -1;

    // Get a pointer to the i2c adapter
    adapter = i2c_get_adapter(i2c_adapter);
    if (!adapter)
    {
        dev_err(dev, "Failed to get i2c adapter\n");
        return -ENODEV;
    }

    // Dynamically create an i2c_client for the slave device
    client = i2c_new_device(adapter, &(struct i2c_board_info)
    {
        I2C_BOARD_INFO("ics9lpr363", i2c_addr),
    });
    if (!client) {
        dev_err(dev, "Failed to create i2c_client\n");
        i2c_put_adapter(adapter);
        return -ENODEV;
    }
    
	res = i2c_smbus_read_block_data(client, CMD, buf);
	if(res < 0)
    {
        dev_err(dev, "Failed to read data from 0x%02x on i2c-%d\n",
                        client->addr, adapter->nr);
        i2c_unregister_device(client);
        i2c_put_adapter(adapter);
        return -1;
    }

	if ((buf[0x0B] & 0xC0)== 0xC0)
		step_mult = 2;
	else if ((buf[0x0B] & 0xC0) == 0x40)
		step_mult = 1;
	else if ((buf[0x0B] & 0xC0) == 0x80)
		step_mult = 0;
    else    // wtf
    {
        dev_err(dev, "Unexpected 0x0B byte from read buffer. Wrong device at 0x%02x?\n",
                        client->addr);
        i2c_unregister_device(client);
        i2c_put_adapter(adapter);
        return -1;
    }

	pll_step = buf[0x0C] + (0x100 * step_mult);     // Frequency step
    raw_ctrl_word = (buf[0x0B] << 8) | buf[0x0C];   // Raw
    pll_m_actual = buf[0x0B] & 0x3F;                // PLL M

    if ((buf[0x0B] & 0x3F) != 0x08)
        dev_info(dev, "Unknown PLL M value (!= 8). Can't calculate FSB frequency.\n");
    else
	    fsb_freq = ((unsigned int)pll_step * 3578U) / 10 + FSB_MIN;  // Real freq

    i2c_unregister_device(client);
    i2c_put_adapter(adapter);
    return 0;
}

/////////////////////////////////// SYSFS /////////////////////////////////////

static ssize_t i2c_addr_show(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "0x%02X\n", i2c_addr);
}

static ssize_t i2c_addr_store(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    unsigned int var;
    sscanf(buf, "%x", &var);
    if (var < 0x03 || var > 0x77)
    {
        dev_err(dev, "Wrong I2C Address (0x%02X). Limits: 0x03-0x77.\n", var);
        return -EINVAL;
    }

    i2c_addr = var & 0xFF;

    return count;
}

static ssize_t i2c_adapter_show(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", i2c_adapter);
}

static ssize_t i2c_adapter_store(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    sscanf(buf, "%d", &i2c_adapter);
    return count;
}

static ssize_t raw_show(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    int ret = 0;
   
    ret = ics9lpr363_GetFSB(dev);
    if (ret)
        return ret;

    return snprintf(buf, PAGE_SIZE, "0x%04X\n", raw_ctrl_word);
}

static ssize_t raw_store(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    int var;
    int ret = 0;

    sscanf(buf, "%x", &var);
    if (var < 0x0000 || var > 0xFFFF)
    {
        dev_err(dev, "Wrong raw WORD.\n");
        return -EINVAL;
    }

    raw_ctrl_word = var & 0xFFFF;
    ret = ics9lpr363_SetFSB(dev, (enum SetLogic)byRaw);
    if (ret)
        return ret;

    return count;
}

static ssize_t fsb_freq_show(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    int ret = 0;

    ret = ics9lpr363_GetFSB(dev);
    if (ret)
        return ret;
    
    return snprintf(buf, PAGE_SIZE, "%d KHz\n", fsb_freq);
}

static ssize_t fsb_freq_store(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    int val;
    int ret = 0;
    sscanf(buf, "%d", &val);

    if (val < FSB_MIN || val > FSB_MAX)
    {
        dev_err(dev, "Wrong FSB value (%d). Limits: %d-%d.\n",
                        val, FSB_MIN, FSB_MAX);
        return -EINVAL;
    }

    if (pll_m != 8)
    {
        dev_warn(dev, "PLL M == %d. Force setting to %d.\n",
                        pll_m, 8);
        pll_m = 8;
    }

    fsb_freq = val;
    fsb_freq += 2; // Mathematics compensation

    pll_step = (unsigned int)(fsb_freq - FSB_MIN) * 10U / 3578U; // 0.35789474
    pll_step = (pll_step < 0x00 ? 0x00 : (pll_step > 0x2ff ? 0x2ff : pll_step));
    ret = ics9lpr363_SetFSB(dev, (enum SetLogic)byPllStep);

    ret |= ics9lpr363_GetFSB(dev);
    if (ret)
        return ret;

    dev_info(dev, "Set FSB Frequency to %d KHz.\n", fsb_freq);
    return count;
}

static ssize_t pll_step_show(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    int ret = 0;

    ret = ics9lpr363_GetFSB(dev);
    if (ret)
        return ret;

    return snprintf(buf, PAGE_SIZE, "%d\n", pll_step);
}

static ssize_t pll_step_store(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    int val;
    int ret = 0;
    sscanf(buf, "%d", &val);

    if (val < 0x00 || val > 0x2ff)
    {
        dev_err(dev, "Wrong PLL step value (%d). Limits: 0-767.\n", val);
        return -EINVAL;
    }

    ret = ics9lpr363_GetFSB(dev);
    if (ret)
        return ret;

    if (!pll_m_unlock && pll_m != pll_m_actual)
    {
        dev_warn(dev, "pll_m != pll_m_actual. (%d != %d). Operation aborted.\n",
                        pll_m, pll_m_actual);
        dev_info(dev, "Set pll_m_unlock to 1 if this was not a mistake.\n");
        return -EINVAL;
    }

    pll_step = val;

    ret = ics9lpr363_SetFSB(dev, (enum SetLogic)byPllStep);
    if (ret)
        return ret;
    return count;
}

static ssize_t pll_m_unlock_show(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", (int)pll_m_unlock);
}

static ssize_t pll_m_unlock_store(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    int val;
    sscanf(buf, "%d", &val);
    pll_m_unlock = !!val;
    return count;
}

static ssize_t pll_m_show(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", pll_m);
}

static ssize_t pll_m_store(struct device *dev,
                  struct device_attribute *attr,
                  const char *buf, size_t count)
{
    int var;
    sscanf(buf, "%d", &var);
    if (var < 0x00 || var > 0x3F)
    {
        dev_err(dev, "Wrong PLL M value (%d). Limits: 0-63.\n", var);
        return -EINVAL;
    }

    pll_m = var & 0x3F;

    return count;
}

static ssize_t pll_m_actual_show(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    int ret = 0;

    ret = ics9lpr363_GetFSB(dev);
    if (ret)
        return ret;

    return snprintf(buf, PAGE_SIZE, "%d\n", pll_m_actual);
}

DEVICE_ATTR(i2c_addr, 0644, i2c_addr_show, i2c_addr_store);
DEVICE_ATTR(i2c_adapter_id, 0644, i2c_adapter_show, i2c_adapter_store);
DEVICE_ATTR(raw, 0644, raw_show, raw_store);
DEVICE_ATTR(fsb_freq, 0644, fsb_freq_show, fsb_freq_store);
DEVICE_ATTR(pll_step, 0644, pll_step_show, pll_step_store);
DEVICE_ATTR(pll_m_unlock, 0644, pll_m_unlock_show, pll_m_unlock_store);
DEVICE_ATTR(pll_m_set, 0644, pll_m_show, pll_m_store);
DEVICE_ATTR(pll_m_actual, 0444, pll_m_actual_show, NULL);

static struct attribute *attrs[] = {
    &dev_attr_i2c_addr.attr,
    &dev_attr_i2c_adapter_id.attr,
    &dev_attr_raw.attr,
    &dev_attr_fsb_freq.attr,
    &dev_attr_pll_step.attr,
    &dev_attr_pll_m_unlock.attr,
    &dev_attr_pll_m_set.attr,
    &dev_attr_pll_m_actual.attr,
   NULL
};

static const struct attribute_group attr_group = {
    .attrs = attrs,
};

static struct miscdevice ics9lpr363_device = 
    {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ics9lpr363_pll",
    };

static int __init ics9lpr363_init(void)
{
    int ret;

    pr_info("%s: misc_register(%s)\n", __FUNCTION__,
                    ics9lpr363_device.name);

    ret = misc_register(&ics9lpr363_device);
    if (ret) {
	    pr_err("%s: misc_register(%s) fail\n", __FUNCTION__,
                        ics9lpr363_device.name);
	    return 1;
	}

    if (sysfs_create_group(&ics9lpr363_device.this_device->kobj,
                    &attr_group) < 0) {
	    pr_err("%s: sysfs_create_group fail\n", __FUNCTION__);
	    pr_err("Failed to create sysfs group for device (%s)!\n",
                        ics9lpr363_device.name);
	}

    return 0;
}

static void __exit ics9lpr363_exit(void)
{
    // Remove sysfs interface
    pr_info("%s: unregistering misc device (%s)\n", __FUNCTION__, ics9lpr363_device.name);
    misc_deregister(&ics9lpr363_device);
    sysfs_remove_group(&ics9lpr363_device.this_device->kobj, &attr_group);
}

module_init(ics9lpr363_init);
module_exit(ics9lpr363_exit);
