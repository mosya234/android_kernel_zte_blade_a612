#ifdef CONFIG_COMPAT

#include <linux/fs.h>
#include <linux/compat.h>

#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/gpio.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>
int led_vendor_id=0;
#define STROBE_DEVICE_ID 0xC7
/* liukun@wind-mobi.com 20141231 begin */
#define I2C_STROBE_MAIN_2_7_BIT_ADDR 0x37
#define I2C_STROBE_MAIN_2_CHANNEL 2
/* liukun@wind-mobi.com 20141231 end */

#define TAG_NAME "leds_sky81294.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)



#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
#endif


static struct i2c_client *SKY81294_i2c_client = NULL;

struct SKY81294_platform_data {
	u8 torch_pin_enable;    // 1:  TX1/TORCH pin isa hardware TORCH enable
	u8 pam_sync_pin_enable; // 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input
	u8 thermal_comp_mode_enable;// 1: LEDI/NTC pin in Thermal Comparator Mode
	u8 strobe_pin_disable;  // 1 : STROBE Input disabled
	u8 vout_mode_enable;  // 1 : Voltage Out Mode enable
};

struct SKY81294_chip_data {
	struct i2c_client *client;
	struct SKY81294_platform_data *pdata;
	struct mutex lock;
	u8 last_flag;
	u8 no_pdata;
};

static int SKY81294_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret=0;
	struct SKY81294_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret =  i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_DBG("failed writting at 0x%02x\n", reg);
	return ret;
}

static int SKY81294_read_reg(struct i2c_client *client, u8 reg)
{
	int val=0;
	struct SKY81294_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val =  i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);


	return val;
}
static int SKY81294_chip_init(struct SKY81294_chip_data *chip)
{
    s32 regVal6 = 0;
	regVal6 = SKY81294_read_reg(SKY81294_i2c_client, 0x06);
	if (regVal6 < 0) 
	{
	 PK_DBG("SKY81294 i2c bus read error\n");
	 return -1;
	}
    else
    {
     led_vendor_id = 0x8129;
    }
	return 0;
}
int is_ic_sky81294(void)
{
    return led_vendor_id;
}
/* liukun@wind-mobi.com 20150509 begin */
void SKY81294_torch_set_level(int level)
{
	  char buf[2];
	  PK_DBG("SKY81294 %s,%d,%d,level",__func__,__LINE__,level);
	  buf[0]=0x02;
	  buf[1]=level;  
	  SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);
	  buf[0]=0x03;
	  buf[1]=0x01;
	  SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);	  
}
/* liukun@wind-mobi.com 20150509 end */
void SKY81294_torch_mode(int g_duty)
{
      char buf[2];
      PK_DBG("SKY81294 %s,%d,%d,torch",__func__,__LINE__,g_duty);
      buf[0]=0x02;
      buf[1]=0x07;//205 MA    
  	  SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);
      buf[0]=0x03;
      buf[1]=0x01;
  	  SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);    
}
void SKY81294_flash_mode(int g_duty)
{
      char buf[2];
      PK_DBG("SKY81294 %s,%d,%d,flash",__func__,__LINE__,g_duty);
      buf[0]=0x00;
      buf[1]=g_duty;
      SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);
      buf[0]=0x03;
      buf[1]=0x02;
      SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);   
}
void SKY81294_shutdown_mode(void)
{
      char buf[2];
      printk("SKY81294 %s,%d,flash",__func__,__LINE__);
      buf[0]=0x03;
      buf[1]=0x00;
      SKY81294_write_reg(SKY81294_i2c_client, buf[0], buf[1]);    
}
static int SKY81294_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct SKY81294_chip_data *chip;
	struct SKY81294_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("SKY81294_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		printk(KERN_ERR  "SKY81294 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct SKY81294_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if(pdata == NULL){ //values are set to Zero.
		PK_DBG("SKY81294 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct SKY81294_platform_data),GFP_KERNEL);
		chip->pdata  = pdata;
		chip->no_pdata = 1;
	}
	SKY81294_i2c_client = client;
	chip->pdata  = pdata;
	if(SKY81294_chip_init(chip)<0)
		goto err_chip_init;

	PK_DBG("SKY81294 Initializing is done \n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_DBG("SKY81294 probe is failed \n");
	return -ENODEV;
}

static int SKY81294_remove(struct i2c_client *client)
{
	struct SKY81294_chip_data *chip = i2c_get_clientdata(client);

    if(chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}

#define SKY81294_NAME "leds-SKY81294"
static const struct i2c_device_id SKY81294_id[] = {
	{SKY81294_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id SKY81294_of_match[] = {
	{.compatible = "mediatek,strobe_main_2"},
	{},
};
#endif
static struct i2c_driver SKY81294_i2c_driver = {
	.driver = {
		.name  = SKY81294_NAME,
#ifdef CONFIG_OF
		   .of_match_table = SKY81294_of_match,
#endif
	},
	.probe	= SKY81294_probe,
	.remove   = SKY81294_remove,
	.id_table = SKY81294_id,
};

static int __init SKY81294_init(void)
{
	printk("SKY81294_init\n");
	return i2c_add_driver(&SKY81294_i2c_driver);
}

static void __exit SKY81294_exit(void)
{
	i2c_del_driver(&SKY81294_i2c_driver);
}

module_init(SKY81294_init);
module_exit(SKY81294_exit);

MODULE_DESCRIPTION("Flash driver for SKY81294");
MODULE_AUTHOR("<liukun@wind-mobi.com>");
MODULE_LICENSE("GPL v2");
