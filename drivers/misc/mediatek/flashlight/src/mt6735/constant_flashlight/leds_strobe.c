/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

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



/*
// flash current vs index
    0       1      2       3    4       5      6       7    8       9     10
93.74  140.63  187.5  281.25  375  468.75  562.5  656.25  750  843.75  937.5
     11    12       13      14       15    16
1031.25  1125  1218.75  1312.5  1406.25  1500mA
*/
/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)


#define LED_SKY81294
#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#define PK_VER PK_TRC_VERBOSE
#define PK_ERR PK_ERROR
#else
#define PK_DBG(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_duty=-1;
static int g_timeOutTimeMs=0;

static DEFINE_MUTEX(g_strobeSem);



#define STROBE_DEVICE_ID 0xC6
#define I2C_STROBE_MAIN_SLAVE_7_BIT_ADDR 0x63
#define I2C_STROBE_MAIN_CHANNEL 2

#define  GPIO_FLASH_LED_EN       43
#define  GPIO_TORCH_EN           42
#define     GPIO_MODE_00         0
#define     GPIO_DIR_OUT         1
#define     GPIO_OUT_ZERO       0         
static struct work_struct workTimeOut;

/* #define FLASH_GPIO_ENF GPIO12 */
/* #define FLASH_GPIO_ENT GPIO13 */



static int g_bLtVersion=0;

/* current(mA) 50,94,141,188,281,375,469,563,656,750,844,938,1031,1125,1220,1313,1406,1500 */



/*****************************************************************************
Functions
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data);
#ifdef LED_SKY81294
extern  int is_ic_sky81294(void);
extern void SKY81294_torch_mode(int g_duty);
extern void SKY81294_flash_mode(int g_duty);
extern void SKY81294_shutdown_mode(void);
extern void SKY81294_torch_set_level(int level);
#endif
#define e_DutyNum 16
static struct i2c_client *LM3642_i2c_client;
static int flash_sky81294[e_DutyNum+1] = {0,0,1,2,4,6,8,10,12,14,15,16,17,18,19,20};




struct LM3642_platform_data {
	u8 torch_pin_enable;    // 1:  TX1/TORCH pin isa hardware TORCH enable
	u8 pam_sync_pin_enable; // 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input
	u8 thermal_comp_mode_enable;// 1: LEDI/NTC pin in Thermal Comparator Mode
	u8 strobe_pin_disable;  // 1 : STROBE Input disabled
	u8 vout_mode_enable;  // 1 : Voltage Out Mode enable
};

struct LM3642_chip_data {
	struct i2c_client *client;

	//struct led_classdev cdev_flash;
	//struct led_classdev cdev_torch;
	//struct led_classdev cdev_indicator;

	struct LM3642_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

/* i2c access*/
/*
static int LM3642_read_reg(struct i2c_client *client, u8 reg,u8 *val)
{
	int ret;
	struct LM3642_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	if (ret < 0) {
		PK_DBG("failed reading at 0x%02x error %d\n",reg, ret);
		return ret;
	}
	*val = ret&0xff;

	return 0;
}*/

static int LM3642_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret = 0;
	struct LM3642_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		PK_DBG("failed writting at 0x%02x\n", reg);
	return ret;
}

static int LM3642_read_reg(struct i2c_client *client, u8 reg)
{
	int val=0;
	struct LM3642_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);


	return val;
}



/* ========================= */

static int LM3642_chip_init(struct LM3642_chip_data *chip)
{


	return 0;
}

static int LM3642_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct LM3642_chip_data *chip;
	struct LM3642_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("LM3642_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		PK_DBG("LM3642 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct LM3642_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if (pdata == NULL) {	/* values are set to Zero. */
		PK_DBG("LM3642 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct LM3642_platform_data),GFP_KERNEL);
		chip->pdata = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata = pdata;
	if (LM3642_chip_init(chip) < 0)
		goto err_chip_init;

	LM3642_i2c_client = client;
	PK_DBG("LM3642 Initializing is done\n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_DBG("LM3642 probe is failed\n");
	return -ENODEV;
}

static int LM3642_remove(struct i2c_client *client)
{
	struct LM3642_chip_data *chip = i2c_get_clientdata(client);

	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


#define LM3642_NAME "leds-LM3642"
static const struct i2c_device_id LM3642_id[] = {
	{LM3642_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id LM3642_of_match[] = {
	{.compatible = "mediatek,strobe_main"},
	{},
};
#endif

static struct i2c_driver LM3642_i2c_driver = {
	.driver = {
		   .name = LM3642_NAME,
#ifdef CONFIG_OF
		   .of_match_table = LM3642_of_match,
#endif
		   },
	.probe = LM3642_probe,
	.remove = LM3642_remove,
	.id_table = LM3642_id,
};

static int __init LM3642_init(void)
{
	PK_DBG("LM3642_init\n");
	return i2c_add_driver(&LM3642_i2c_driver);
}

static void __exit LM3642_exit(void)
{
	i2c_del_driver(&LM3642_i2c_driver);
}


module_init(LM3642_init);
module_exit(LM3642_exit);

MODULE_DESCRIPTION("Flash driver for LM3642");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");

int readReg(int reg)
{

	int val;

	val = LM3642_read_reg(LM3642_i2c_client, reg);
	return (int)val;
}
void FL_Torch(int level)
{
  char buf[2];
  if(level>7)
    level =7;
  else if(level<0)
	level=0;
  PK_DBG(" liukun FL_Torch level=%d\n",level);
  if(is_ic_sky81294()== 0x8129)
  {
    SKY81294_torch_set_level(level);
  }
  else
  {
    buf[0]=9;
    buf[1]=level<<4;
    LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);
	
    buf[0]=10;
    buf[1]=0x02;
    LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);

	buf[0]=0xa;
	buf[1]=0x72;
	LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);
	if(gpio_direction_output(GPIO_FLASH_LED_EN,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
     gpio_set_value(GPIO_FLASH_LED_EN,1);
	}
}
int FL_Enable(void)
{
	char buf[2];
	
	if(g_duty<0)
	g_duty=0;
	else if(g_duty>16)
	g_duty=16;
		
	if(is_ic_sky81294()== 0x8129)
	{
		if(g_duty <= 2)
		{
			//torch mode       
			PK_DBG(" FL_Enable sky81294 torch mode\n");
			SKY81294_torch_mode(g_duty);
		}	
		else
		{
			//flash mode
			PK_DBG(" FL_Enable sky81294 flash mode\n");
			SKY81294_flash_mode(flash_sky81294[g_duty]);
		}
	}
	else
	{
		if(g_duty<=2)
		{
			int val;
			if(g_bLtVersion==1)
			{
				if(g_duty==0)
				val=3;
				else if(g_duty==1)
				val=5;
				else //if(g_duty==2)
				val=7;
			}
			else
			{
				if(g_duty==0)
				val=1;
				else if(g_duty==1)
				val=2;
				else //if(g_duty==2)
				val=3;
			}
			buf[0]=9;
			buf[1]=val<<4;
			LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);

			buf[0]=10;
			buf[1]=0x02;
			LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);
			buf[0]=0xa;
			buf[1]=0x72;
			LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);
			if(gpio_direction_output(GPIO_FLASH_LED_EN,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
			gpio_set_value(GPIO_FLASH_LED_EN,1);
		}
		else
		{
		int val;
		val = (g_duty-1);
		buf[0]=9;
		buf[1]=val;
		LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);

		buf[0]=10;
		buf[1]=0x03;
		LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);
		}
	PK_DBG(" FL_Enable line=%d\n",__LINE__);

	readReg(0);
	readReg(1);
	readReg(6);
	readReg(8);
	readReg(9);
	readReg(0xa);
	readReg(0xb);

	if(gpio_direction_output(GPIO_TORCH_EN,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
	gpio_set_value(GPIO_TORCH_EN,1);
	}
	return 0;
}



int FL_Disable(void)
{
	char buf[2];
	if(is_ic_sky81294()== 0x8129)
	{
	 SKY81294_shutdown_mode();
	}
	else
    {
		if(gpio_direction_output(GPIO_TORCH_EN,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
		gpio_set_value(GPIO_TORCH_EN,GPIO_OUT_ZERO);
		if(gpio_direction_output(GPIO_FLASH_LED_EN,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
		gpio_set_value(GPIO_FLASH_LED_EN,GPIO_OUT_ZERO);
  	   buf[0]=0xa;
  	   buf[1]=0x00;
  	   LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);
		
       buf[0]=10;
       buf[1]=0x00;
   	   LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);
	}
	PK_DBG(" FL_Disable line=%d\n", __LINE__);
	return 0;
}



int main_strobe_single_on(void){
	char buf[2];
	int my_duty=1;
	if(is_ic_sky81294()== 0x8129)
    {
      if(my_duty <= 2)
      {
       //torch mode       
	   PK_DBG(" FL_Enable sky81294 torch mode\n");
       SKY81294_torch_mode(my_duty);
      }	
      else
      {
       //flash mode
       PK_DBG(" FL_Enable sky81294 flash mode\n");
       SKY81294_flash_mode(flash_sky81294[my_duty]);
      }
	}
	else
    {
     if(my_duty<=2)
		{
           int val;
           if(g_bLtVersion==1)
           {
               if(my_duty==0)
                   val=3;
               else if(my_duty==1)
                   val=5;
               else //if(g_duty==2)
                   val=7;
           }
           else
           {
               if(my_duty==0)
                   val=1;
               else if(my_duty==1)
                   val=2;
               else //if(g_duty==2)
                   val=3;
           }
			buf[0]=9;
			buf[1]=val<<4;
			LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);

			buf[0]=10;
			buf[1]=0x02;
			LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);
			
			buf[0]=0xa;
			buf[1]=0x72;
			LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);
			if(gpio_direction_output(GPIO_FLASH_LED_EN,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
			gpio_set_value(GPIO_FLASH_LED_EN,1);
		}
		else
		{
		int val;
		val = (my_duty-1);
		buf[0]=9;
		buf[1]=val;
		LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);

		buf[0]=10;
		buf[1]=0x03;
		LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);
		}
	PK_DBG(" FL_Enable line=%d\n",__LINE__);

	readReg(0);
	readReg(1);
	readReg(6);
	readReg(8);
	readReg(9);
	readReg(0xa);
	readReg(0xb);

	if(gpio_direction_output(GPIO_TORCH_EN,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
	gpio_set_value(GPIO_TORCH_EN,1);
	}
	return 0;
}

int main_strobe_single_off(void){
	char buf[2];
	if(is_ic_sky81294()== 0x8129)
	{
	 SKY81294_shutdown_mode();
	}
	else
    {

	if(gpio_direction_output(GPIO_TORCH_EN,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
		gpio_set_value(GPIO_TORCH_EN,GPIO_OUT_ZERO);
	if(gpio_direction_output(GPIO_FLASH_LED_EN,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
		gpio_set_value(GPIO_FLASH_LED_EN,GPIO_OUT_ZERO);
	buf[0]=0xa;
	buf[1]=0x00;
	LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);

	buf[0]=10;
	buf[1]=0x00;
	LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);
	}
	PK_DBG(" FL_Disable line=%d\n", __LINE__);
	return 0;
}

int FL_dim_duty(kal_uint32 duty)
{

	PK_DBG(" FL_dim_duty line=%d\n", __LINE__);
	g_duty = duty;
	return 0;
}




int FL_Init(void)
{
   int regVal0;
   char buf[2];
   if(is_ic_sky81294()!= 0x8129)
   {
    buf[0]=0xa;
	buf[1]=0x0;
	LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);

	buf[0]=0x8;
	buf[1]=0x47;
	LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);

	buf[0]=9;
	buf[1]=0x35;
	LM3642_write_reg(LM3642_i2c_client, buf[0], buf[1]);
	regVal0 = LM3642_read_reg(LM3642_i2c_client, 0);
	if(regVal0==1)
	    g_bLtVersion=1;
	else
	    g_bLtVersion=0;
    PK_DBG(" FL_Init regVal0=%d isLtVer=%d\n",regVal0, g_bLtVersion);
	}
	

	PK_DBG(" FL_Init line=%d\n", __LINE__);
	return 0;
}


int FL_Uninit(void)
{
	FL_Disable();
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	static int init_flag;

	if (init_flag == 0) {
		init_flag = 1;
		INIT_WORK(&workTimeOut, work_timeOutFunc);
		g_timeOutTimeMs = 1000;
		hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		g_timeOutTimer.function = ledTimeOutCallback;
	}
}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;

	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
/*	PK_DBG
	    ("LM3642 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",
	     __LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
*/
    switch(cmd)
    {

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;

	case FLASH_IOC_SET_ONOFF:
		PK_DBG("FLASHLIGHT_ONOFF: %d\n", (int)arg);
    		if(arg==1)
    		{
    		    int s;
    		    int ms;
    		    if(g_timeOutTimeMs>1000)
            	{
            		s = g_timeOutTimeMs/1000;
            		ms = g_timeOutTimeMs - s*1000;
            	}
            	else
            	{
            		s = 0;
            		ms = g_timeOutTimeMs;
            	}
				if(g_timeOutTimeMs!=0)
	            {
				ktime_t ktime;

				ktime = ktime_set(0, g_timeOutTimeMs * 1000000);
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			FL_Enable();
    		}
    		else
    		{
    			FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;

	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
		FL_Init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_DBG(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
