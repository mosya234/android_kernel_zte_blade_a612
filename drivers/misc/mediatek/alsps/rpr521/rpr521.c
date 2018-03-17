/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/* drivers/hwmon/mt16/amit/rpr521.c - rpr521 ALS/PS driver
 * 
 * Author: MingHsien Hsieh <minghsien.hsieh@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
  /******************************************************************************
*Revision History
*Ver 0.9		Grace	Feb 2014		Make a new file based on original files.
*******************************************************************************/

#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>

//#define POWER_NONE_MACRO MT65XX_POWER_NONE
#include <hwmsensor.h>
#include <hwmsen_dev.h>
//#include <asm/io.h>
#include <cust_alsps.h>
#include "rpr521.h"
 
#include <linux/proc_fs.h>
#include <alsps.h>

/******************************************************************************
 * configuration
*******************************************************************************/
/*----------------------------------------------------------------------------*/

#define RPR521_DEV_NAME     "rpr521"
/*----------------------------------------------------------------------------*/
#define APS_TAG                  "[ALS/PS] "
#define DEBUG

#if defined(DEBUG)
#define APS_FUN(f)               	 printk(APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(APS_TAG fmt, ##args)  
#define APS_FUN_BEGIN(f)               	 printk(APS_TAG"%s%s\n", __FUNCTION__," begin")
#define APS_FUN_END(f)               	 printk(APS_TAG"%s%s\n", __FUNCTION__," end")
#else
#define APS_FUN(f)           
#define APS_ERR(fmt, args...)
#define APS_LOG(fmt, args...)
#define APS_DBG(fmt, args...)
#endif

/*
//GPIO
#define GPIO_WM_ALS_EINT_PIN         (GPIO3 | 0x80000000)
#define GPIO_WM_ALS_EINT_PIN_M_EINT           GPIO_MODE_00
#define CUST_WM_EINT_ALS_NUM  3
#define CUST_WM_EINT_ALS_DEBOUNCE_CN  0
#define CUST_WM_EINT_ALS_TYPE  CUST_EINTF_TRIGGER_LOW
*/
//
#define _AUTO_THRESHOLD_CHANGE_
//#define _TEST_


/******************************************************************************
 * extern functions
*******************************************************************************/

/*----------------------------------------------------------------------------*/
static struct i2c_client *rpr521_i2c_client = NULL;
struct alsps_hw		alsps_rpr521;
static struct alsps_hw	*hw = &alsps_rpr521;

static int alsps_rpr521_init(void);
static int alsps_rpr521_uninit(void);

static struct alsps_init_info rpr521_init_info=
{
    .name ="rpr521",
    .init = alsps_rpr521_init,
    .uninit = alsps_rpr521_uninit,

};

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id rpr521_i2c_id[] = {{RPR521_DEV_NAME,0},{}};
//static struct i2c_board_info __initdata i2c_RPR521={ I2C_BOARD_INFO(RPR521_DEV_NAME, (0X70>>1))};

/*----------------------------------------------------------------------------*/
static int rpr521_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int rpr521_i2c_remove(struct i2c_client *client);
static int rpr521_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
/*----------------------------------------------------------------------------*/
static int rpr521_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int rpr521_i2c_resume(struct i2c_client *client);
long rpr521_read_ps(struct i2c_client *client, u16 *data);
int rpr521_read_als(struct i2c_client *client, u16 *data);
static void rpr521_check_prox_mean(unsigned int prox_mean);

//static struct rpr521_priv *g_rpr521_ptr = NULL;
u16 prev_als_value = 0;
static unsigned int alsps_irq;

typedef struct {
    unsigned long long data;
    unsigned long long data0;
    unsigned long long data1;
    unsigned char      gain_data0;
    unsigned char      gain_data1;
    unsigned long      dev_unit;
    unsigned char      als_time;
    unsigned short     als_data0;
    unsigned short     als_data1;
} CALC_DATA;

typedef struct {
    unsigned long positive;
    unsigned long decimal;
} CALC_ANS;

/*----------------------------------------------------------------------------*/
typedef enum {
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
    CMC_BIT_IRQ    = 3,
} CMC_BIT;
/*----------------------------------------------------------------------------*/

struct rpr521_i2c_addr {    /*define a series of i2c slave address*/
    u8  write_addr;  
    u8  ps_thd;     /*PS INT threshold*/
};


struct rpr521_priv {
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;

    /*i2c address group*/
    struct rpr521_i2c_addr  addr;
    
    /*misc*/
    u16		    als_modulus;
    atomic_t    i2c_retry;
    atomic_t    als_suspend;
    atomic_t    als_debounce;   /*debounce time after enabling als*/
    atomic_t    als_deb_on;     /*indicates if the debounce is on*/
    atomic_t    als_deb_end;    /*the jiffies representing the end of debounce*/
    atomic_t    ps_mask;        /*mask ps: always return far away*/
    atomic_t    ps_debounce;    /*debounce time after enabling ps*/
    atomic_t    ps_deb_on;      /*indicates if the debounce is on*/
    atomic_t    ps_deb_end;     /*the jiffies representing the end of debounce*/
    atomic_t    ps_suspend;

    struct device_node	*irq_node;
    int		irq;
    
    /*data*/
    u16         als;
    u16          ps;
	u16         als_data0;
	u16         als_data1;
    u8          _align;
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL];
    u32         als_value[C_CUST_ALS_LEVEL];

    atomic_t    als_cmd_val;    /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_cmd_val;     /*the cmd value can't be read, stored in ram*/
    atomic_t    ps_thd_val;     /*the cmd value can't be read, stored in ram*/
    ulong       enable;         /*enable mask*/
    ulong       pending_intr;   /*pending interrupt*/
     u16              ps_th_h;
     u16              ps_th_l;
};

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,alsps"},
	{},
};
#endif

/*----------------------------------------------------------------------------*/
static struct i2c_driver rpr521_i2c_driver = {	
	.probe      = rpr521_i2c_probe,
	.remove     = rpr521_i2c_remove,
	.detect     = rpr521_i2c_detect,
	.suspend    = rpr521_i2c_suspend,
	.resume     = rpr521_i2c_resume,
	.id_table   = rpr521_i2c_id,
	.driver = {
		.owner          = THIS_MODULE,
		.name           = RPR521_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif
	},
};

static int rpr521_ps_startup_cross_talk = 0;
static int rpr521_ps_startup_calibration = 0;
static u16 rpr521_ps_cross_talk = 0;

static int THRES_TOLERANCE = 25;
static int THRES_DEFAULT_DIFF = 35;

static int als_value_backup = -1;
static int lux_backup = -1;

static struct proc_dir_entry *rpr521_proc_file = NULL;
static struct proc_dir_entry *prox_calibration_value = NULL;

static DEFINE_MUTEX(rpr521_mutex);


static int rpr521_get_threshold(struct i2c_client *client);
static int rpr521_set_threshold(int average);
static int rpr521_set_ps_threshold(struct i2c_client *client, u16 ps_tl, u16 ps_th);
static long rpr521_enable_ps(struct i2c_client *client, int enable);

#ifdef _AUTO_THRESHOLD_CHANGE_
static int rpr521_calibrate(struct i2c_client *client);
#endif

static ssize_t prox_calibration_value_read(struct file *file,	char __user *buffer, size_t count, loff_t *offset)
{
    int p_data = 0;
    int cnt;	
    struct rpr521_priv *obj = i2c_get_clientdata(rpr521_i2c_client);	
    APS_FUN();
    
    if(*offset != 0)
    {
        APS_DBG("%s,return 0\n", __FUNCTION__);
        return 0;
    }


    //less on/off-----
/*
	rpr521_ps_original_power = test_bit(CMC_BIT_PS, &obj->enable) ? (TRUE) : (FALSE);
    if(FALSE == rpr521_ps_original_power)
    {
        rpr521_enable_ps(rpr521_i2c_client, 1);
        set_bit(CMC_BIT_PS, &obj->enable);
    }
    //less on/off-----
*/
    p_data = rpr521_get_threshold(rpr521_i2c_client);
    cnt = sprintf(buffer, "%d %d %d %d %d\n", rpr521_ps_startup_calibration, rpr521_ps_startup_cross_talk, obj->ps_th_h, obj->ps_th_l, p_data);
	
    *offset += cnt;    
    return cnt;
}


static ssize_t rpr521_read_proc(struct file *file,	char __user *buffer, size_t count, loff_t *offset)
{
    int cnt;    
    APS_FUN();
    
    if(*offset != 0)
    {
        APS_DBG("%s,return 0\n", __FUNCTION__);
        return 0;
    }	

	rpr521_ps_cross_talk = rpr521_get_threshold(rpr521_i2c_client);	
/*	
	rpr521_check_prox_mean(rpr521_ps_cross_talk); 
       rpr521_set_ps_threshold(rpr521_i2c_client, rpr521_ps_cross_talk + THRES_TOLERANCE, rpr521_ps_cross_talk + THRES_TOLERANCE + THRES_DEFAULT_DIFF);
*/
    rpr521_set_threshold(rpr521_ps_cross_talk);
	
	cnt = sprintf(buffer, "%d\n", rpr521_ps_cross_talk);	
	
    *offset += cnt;
    APS_DBG("%s,rpr521_ps_cross_talk=%d\n",__func__,rpr521_ps_cross_talk);
    return cnt;
}

static ssize_t rpr521_write_proc(struct file *file, const char __user *user_buf, size_t len, loff_t *offset)
{
	int value = 0;
    char buf[16]={0};

	size_t copyLen = 0;
	
	APS_DBG("%s: write len = %d\n",__func__, (int)len);
	copyLen = len < 16 ? len : 16;
	if (copy_from_user(buf, user_buf, copyLen))
	{
		APS_DBG("%s, copy_from_user error\n", __func__);
		return -EFAULT;
	}
		
	sscanf(buf, "%d", &value);
	APS_DBG("buf=%s, value = %d, copyLen=%d\n", buf, value, (int)copyLen);
	// ps is 12bit value (0x0fff), if it's 0, no nv calibration.  
	if (value > 0 && value < 4096)
	{
		rpr521_ps_cross_talk = value;
	}	

    //开机校准
    rpr521_ps_startup_cross_talk = rpr521_get_threshold(rpr521_i2c_client);
    rpr521_ps_startup_calibration = rpr521_ps_startup_cross_talk;
    APS_DBG("%s, rpr521_ps_cross_talk=%d, rpr521_ps_startup_cross_talk=%d\n", __func__,rpr521_ps_cross_talk, rpr521_ps_startup_cross_talk);

	if(rpr521_ps_cross_talk > 0)
	{
		if(rpr521_ps_startup_cross_talk > rpr521_ps_cross_talk)
		{
			if((rpr521_ps_startup_cross_talk - rpr521_ps_cross_talk) > 100)
			{
				rpr521_ps_startup_cross_talk = rpr521_ps_cross_talk;
			}
		}
	/*
		else if (rpr521_ps_startup_cross_talk <= rpr521_ps_cross_talk )
		{
			if((rpr521_ps_cross_talk - rpr521_ps_startup_cross_talk) > 20)
			{
				rpr521_ps_startup_cross_talk = rpr521_ps_cross_talk;
			}
		}
	*/
	}
 /*	
	rpr521_check_prox_mean(rpr521_ps_startup_cross_talk); 	
	rpr521_set_ps_threshold(rpr521_i2c_client, rpr521_ps_startup_cross_talk + THRES_TOLERANCE, rpr521_ps_startup_cross_talk + THRES_TOLERANCE + THRES_DEFAULT_DIFF);
*/
	rpr521_set_threshold(rpr521_ps_startup_cross_talk);
	return len;
}

static const struct file_operations rpr521_proc_fops = {
	.owner		= THIS_MODULE,
	//.open       = rpr521_calibrate_open,
	.read       = rpr521_read_proc,
	.write      = rpr521_write_proc,
};

static const struct file_operations prox_calibration_value_fops = {
	.owner		= THIS_MODULE,	
	.read       = prox_calibration_value_read,	
};


/*   20150315 zhangxin add calibration in call   */	
static struct proc_dir_entry *calibration_inCall = NULL;
static ssize_t calibration_inCall_read(struct file *file,	char __user *buffer, size_t count, loff_t *offset)
{
    int len;

    APS_FUN();
	
    if(*offset != 0)
    {
        APS_DBG("%s,return 0\n", __FUNCTION__);
        return 0;
    }

    rpr521_ps_startup_cross_talk = rpr521_get_threshold(rpr521_i2c_client);
    APS_DBG("%s, rpr521_ps_cross_talk=%d, rpr521_ps_calibration_incall_cross_talk=%d\n", __func__,rpr521_ps_cross_talk, rpr521_ps_startup_cross_talk);

	if(rpr521_ps_cross_talk > 0)
	{
		if(rpr521_ps_startup_cross_talk > rpr521_ps_cross_talk)
		{
			if((rpr521_ps_startup_cross_talk - rpr521_ps_cross_talk) > 100)
			{
				rpr521_ps_startup_cross_talk = rpr521_ps_cross_talk;
			}
		}
		else if (rpr521_ps_startup_cross_talk <= rpr521_ps_cross_talk )
		{
			if((rpr521_ps_cross_talk - rpr521_ps_startup_cross_talk) > 20)
			{
				rpr521_ps_startup_cross_talk = rpr521_ps_cross_talk;
			}
		}		
	}
/*
    rpr521_check_prox_mean(rpr521_ps_startup_cross_talk); 	
	rpr521_set_ps_threshold(rpr521_i2c_client, rpr521_ps_startup_cross_talk + THRES_TOLERANCE, rpr521_ps_startup_cross_talk + THRES_TOLERANCE + THRES_DEFAULT_DIFF);
*/
    rpr521_set_threshold(rpr521_ps_startup_cross_talk);
    len = sprintf(buffer, "%d", rpr521_ps_startup_cross_talk);
    *offset += len;
    APS_DBG("%s,rpr521_ps_cross_talk=%d\n",__func__,rpr521_ps_cross_talk);
    return len;
}

static const struct file_operations calibration_inCall_fops = {
	.owner		= THIS_MODULE,
	.read       = calibration_inCall_read,
};
/*   20150315 zhangxin add calibration in call  end */	

static void rpr521_check_prox_mean(unsigned int prox_mean)
{
   
	if(prox_mean <= 100)
	{
	    THRES_TOLERANCE =  30;//25;
        THRES_DEFAULT_DIFF = 30;//35;  
	}
	else if(prox_mean > 100)
	{
           THRES_TOLERANCE = 35; //30;
	    THRES_DEFAULT_DIFF = 50; //20;
	}
	
	APS_ERR("rpr521_check_prox_mean,DIFF=%d,TOLERANCE=%d\n", THRES_DEFAULT_DIFF,THRES_TOLERANCE);
}



static struct rpr521_priv *rpr521_obj = NULL;
static struct platform_driver rpr521_alsps_driver;

/*----------------------------------------------------------------------------*/
int rpr521_get_addr(struct alsps_hw *hw, struct rpr521_i2c_addr *addr)
{
	if(!hw || !addr)
	{
		return -EFAULT;
	}
	addr->write_addr= hw->i2c_addr[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static void rpr521_power(struct alsps_hw *hw, unsigned int on) 
{
}
/*----------------------------------------------------------------------------*/
static long rpr521_enable_als(struct i2c_client *client, int enable)
{
	struct rpr521_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];	  
	int res = 0;
	//u8 buffer[1];
	//u8 reg_value[1];	
	//u8 power_state, power_set;
	//uint32_t testbit_ALS;
	int rpr521_als_enabled;
 	APS_DBG("rpr521_enable_als enable:%d \n",enable);
 	
	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
		
	rpr521_als_enabled = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);	
		
	if(enable)
	{
		if(rpr521_als_enabled == CTL_STANDALONE)
		{
		    APS_DBG("als already enabled, return directly\n");
			return 0;
		}

	    mutex_lock(&rpr521_mutex);
	/*	
		databuf[0]= REG_MODECONTROL;
		res = i2c_master_send(client, databuf, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		res = i2c_master_recv(client, databuf, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
	*/
		res = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
		if (res < 0)
		{
			APS_ERR("read RegModeControl err (%d)\n", res);
			goto EXIT_ERR;
		}
		databuf[0] = res;
		APS_DBG("RegModeControl [als_enable]= 0x%x\n",databuf[0]);

		databuf[0] &= (~ALS_PS_MEASUREMENT_MASK);
		databuf[1] = databuf[0] | ALS_ENABLE | ALS_PS_MEASUREMENT_100MS;
	/*   
	 	databuf[0]= REG_MODECONTROL;		
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
	*/
		res = i2c_smbus_write_byte_data(client, REG_MODECONTROL, databuf[1]);
		if(res < 0)
		{
			APS_ERR("write RegModeControl err (%d)\n", res);
			goto EXIT_ERR;
		}
		APS_DBG("als_enable RegModeControl set to 0x%x\n", databuf[1]);
		
		mutex_unlock(&rpr521_mutex);
		als_value_backup = -1;
		atomic_set(&obj->als_deb_on, 1);
		atomic_set(&obj->als_deb_end, jiffies+atomic_read(&obj->als_debounce)/(1000/HZ));
		
#ifdef _TEST_
		databuf[0]  = REG_SYSTEMCONTROL;
		res = i2c_master_send(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
		res = i2c_master_recv(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
	
		APS_DBG(">>>>>>>rpr521: ALS_ENABLE REG_SYSTEMCONTROL    reg_value = %x \n", databuf[0]);
	
		databuf[0]  = REG_MODECONTROL;
		res = i2c_master_send(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
		res = i2c_master_recv(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
	
		APS_DBG(">>>>>>>rpr521: ALS_ENABLE REG_MODECONTROL    reg_value = %x \n", databuf[0]);
		
		databuf[0]  = REG_INTERRUPT;
		res = i2c_master_send(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
		res = i2c_master_recv(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
	
		APS_DBG(">>>>>>>rpr521: ALS_ENABLE REG_INTERRUPT    reg_value = %x \n", databuf[0]);
#endif
				
	}
	else
	{
		if(rpr521_als_enabled == CTL_STANDBY)
		{
		    APS_DBG("als already disabled, return directly\n");
			return 0;
		}
        mutex_lock(&rpr521_mutex);
	/*	
		databuf[0]= REG_MODECONTROL;
		res = i2c_master_send(client, databuf, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		res = i2c_master_recv(client, databuf, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
	*/
		res = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
		if (res < 0)
		{
			APS_ERR("read RegModeControl err (%d)\n", res);
			goto EXIT_ERR;
		}
		databuf[0] = res;
		APS_DBG("RegModeControl [als_disable]= 0x%x\n",databuf[0]);
		databuf[1] = databuf[0] & (~ALS_ENABLE);			
	/*	
		databuf[0] = REG_MODECONTROL;	
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
	*/
		res = i2c_smbus_write_byte_data(client, REG_MODECONTROL, databuf[1]);
		if(res < 0)
		{
			APS_ERR("write RegModeControl err (%d)\n", res);
			goto EXIT_ERR;
		}
		APS_DBG("als_enable RegModeControl set to 0x%x\n", databuf[1]);
		
		mutex_unlock(&rpr521_mutex);
		atomic_set(&obj->als_deb_on, 0);
		
#ifdef _TEST_
		databuf[0]  = REG_SYSTEMCONTROL;
		res = i2c_master_send(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
		res = i2c_master_recv(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
	
		APS_DBG(">>>>>>>rpr521: ALS_OFF REG_SYSTEMCONTROL    reg_value = %x \n", databuf[0]);
	
		databuf[0]  = REG_MODECONTROL;
		res = i2c_master_send(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
		res = i2c_master_recv(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
	
		APS_DBG(">>>>>>>rpr521: ALS_OFF REG_MODECONTROL    reg_value = %x \n", databuf[0]);
		
		databuf[0]  = REG_INTERRUPT;
		res = i2c_master_send(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
		res = i2c_master_recv(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
	
		APS_DBG(">>>>>>>rpr521: ALS_OFF REG_INTERRUPT    reg_value = %x \n", databuf[0]);
#endif
	}
	return 0;
		
	EXIT_ERR:
		mutex_unlock(&rpr521_mutex);
		APS_ERR("rpr521_enable_als fail\n");
		return res;
}


/*----------------------------------------------------------------------------*/
static long rpr521_enable_ps(struct i2c_client *client, int enable)
{
	struct rpr521_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
	//u8 buffer[1];
	//u8 reg_value[1];
	//u8 power_state, power_set; 	
	//PWR_ST  pwr_st;	
	int rpr521_ps_enabled;
 	APS_DBG("rpr521_enable_ps enable:%d \n",enable);

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	rpr521_ps_enabled = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
 
	if(enable)
	{

		if(rpr521_ps_enabled == CTL_STANDALONE)
		{
		    APS_DBG("ps enabled, return directly\n");
			return 0;
		}
        mutex_lock(&rpr521_mutex);
	/*	
	    databuf[0]= REG_MODECONTROL;
		res = i2c_master_send(client, databuf, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		res = i2c_master_recv(client, databuf, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
	*/
		res = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
		if (res < 0)
		{
			APS_ERR("read REG_MODECONTROL err (%d)\n", res);
			goto EXIT_ERR;
		}
		databuf[0] = res;
		APS_DBG("RegModeControl [ps_enable]= 0x%x\n",databuf[0]);

		databuf[0] &= (~ALS_PS_MEASUREMENT_MASK);
		databuf[1] = databuf[0] | PS_ENABLE | ALS_PS_MEASUREMENT_100MS;             
	/*	
		databuf[0] = REG_MODECONTROL;					   
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
	*/
		res = i2c_smbus_write_byte_data(client, REG_MODECONTROL, databuf[1]);
		if(res < 0)
		{
			APS_ERR("write REG_MODECONTROL err (%d)\n", res);
			goto EXIT_ERR;
		}
		APS_DBG("ps_enable RegModeControl set to 0x%x\n", databuf[1]);
		
		mutex_unlock(&rpr521_mutex);
		atomic_set(&obj->ps_deb_on, 1);
		atomic_set(&obj->ps_deb_end, jiffies+atomic_read(&obj->ps_debounce)/(1000/HZ));


		if(0 == obj->hw->polling_mode_ps)
		{
			/*
			databuf[0] = REG_PSTL_LSB;	
			databuf[1] = (u8)(PS_ALS_SET_PS_TL & 0x00FF);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = REG_PSTL_MBS;	
			databuf[1] = (u8)((PS_ALS_SET_PS_TL & 0xFF00) >> 8);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = REG_PSTH_LSB;	
			databuf[1] = (u8)(PS_ALS_SET_PS_TH & 0x00FF);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
			databuf[0] = REG_PSTH_MBS; 
			databuf[1] = (u8)((PS_ALS_SET_PS_TH & 0xFF00) >> 8);
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
                   */
            // set both threshold (H & L) 
            rpr521_set_ps_threshold(rpr521_i2c_client, obj->ps_th_l, obj->ps_th_h);
            mutex_lock(&rpr521_mutex);
		//	databuf[0] = REG_INTERRUPT;
			databuf[1] = PS_ALS_SET_INTR|MODE_PROXIMITY; //caosq  reg_value

		/*	 
			res = i2c_master_send(client, databuf, 0x2);
			if(res <= 0)
			{
				goto EXIT_ERR;
			}
		*/
			res = i2c_smbus_write_byte_data(client, REG_INTERRUPT, databuf[1]);
			if(res < 0)
			{
				APS_ERR("write REG_MODECONTROL err (%d)\n", res);
				goto EXIT_ERR;
			}
			mutex_unlock(&rpr521_mutex);
			if (0 == test_and_set_bit(CMC_BIT_IRQ, &obj->enable))
			{
				enable_irq(rpr521_obj->irq); //mt_eint_unmask(CUST_EINT_ALS_NUM); 
				APS_ERR("enable irq...\n");
			}
		}
		
#ifdef _TEST_
		databuf[0]  = REG_SYSTEMCONTROL;
		res = i2c_master_send(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
		res = i2c_master_recv(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
	
		APS_DBG(">>>>>>>rpr521: PS_ENABLE REG_SYSTEMCONTROL    reg_value = %x \n", databuf[0]);
	
		databuf[0]  = REG_MODECONTROL;
		res = i2c_master_send(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
		res = i2c_master_recv(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
	
		APS_DBG(">>>>>>>rpr521: PS_ENABLE REG_MODECONTROL    reg_value = %x \n", databuf[0]);
		
		databuf[0]  = REG_INTERRUPT;
		res = i2c_master_send(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
		res = i2c_master_recv(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
	
		APS_DBG(">>>>>>>rpr521: PS_ENABLE REG_INTERRUPT    reg_value = %x \n", databuf[0]);
#endif
			
	}
	else
	{

		if(rpr521_ps_enabled == CTL_STANDBY)
		{
		    APS_DBG("ps disable already, return directly\n");
			return 0;
		}
        mutex_lock(&rpr521_mutex);
		databuf[0]= REG_MODECONTROL;
		res = i2c_master_send(client, databuf, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		res = i2c_master_recv(client, databuf, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		APS_DBG("RegModeControl [ps_disable]= 0x%x\n",databuf[0]);
		databuf[1] = databuf[0] & (~PS_ENABLE);			
		databuf[0] = REG_MODECONTROL;				

		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
        APS_DBG("RegModeControl set to 0x%x\n", databuf[1]);
		mutex_unlock(&rpr521_mutex);
		atomic_set(&obj->ps_deb_on, 0);
		
#ifdef _TEST_
		databuf[0]  = REG_SYSTEMCONTROL;
		res = i2c_master_send(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
		res = i2c_master_recv(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
	
		APS_DBG(">>>>>>>rpr521: PS_OFF REG_SYSTEMCONTROL    reg_value = %x \n", databuf[0]);
	
		databuf[0]  = REG_MODECONTROL;
		res = i2c_master_send(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
		res = i2c_master_recv(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
	
		APS_DBG(">>>>>>>rpr521: PS_OFF REG_MODECONTROL    reg_value = %x \n", databuf[0]);
		
		databuf[0]  = REG_INTERRUPT;
		res = i2c_master_send(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
		res = i2c_master_recv(client, databuf, 0x1);

		if(res <= 0)
		{
			return -1;
		}
	
		APS_DBG(">>>>>>>rpr521: PS_OFF REG_INTERRUPT    reg_value = %x \n", databuf[0]);
#endif

		/*for interrup work mode support */
		if(0 == obj->hw->polling_mode_ps)
		{
			cancel_work_sync(&obj->eint_work);
			if (test_and_clear_bit(CMC_BIT_IRQ, &obj->enable))
			{
				disable_irq_nosync(rpr521_obj->irq); //mt_eint_mask(CUST_EINT_ALS_NUM);
				APS_ERR("disable irq...\n");
			}
		}
	}

	
	return 0;
	
EXIT_ERR:
    mutex_unlock(&rpr521_mutex);
	APS_ERR("rpr521_enable_ps fail\n");
	return res;
	
}


/*----------------------------------------------------------------------------*/
/*for interrup work mode support*/
static int rpr521_check_and_clear_intr(struct i2c_client *client) 
{
	//struct rpr521_priv *obj = i2c_get_clientdata(client);
	int res;
	u8 buffer[2], int_status[1];
	mutex_lock(&rpr521_mutex);
	buffer[0] = REG_INTERRUPT;
	res = i2c_master_send(client, buffer, 0x1);

	if(res <= 0)
	{
	    mutex_unlock(&rpr521_mutex);
		return -1;
	}
	res = i2c_master_recv(client, int_status, 0x1);

	if(res <= 0)
	{
	    mutex_unlock(&rpr521_mutex);
		return -1;
	}
	mutex_unlock(&rpr521_mutex);
	APS_DBG(">>>>>>>rpr521: 0x4A    int_status = %x \n", int_status[0]);
	return int_status[0];
}
/*----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------*/
static irqreturn_t rpr521_eint_func(int irq, void *desc)
{
	struct rpr521_priv *obj = rpr521_obj;
    
	APS_FUN();
    
	if(!obj)
	{
		return IRQ_HANDLED;
	}
	if (test_and_clear_bit(CMC_BIT_IRQ, &obj->enable))
	{
		disable_irq_nosync(rpr521_obj->irq);
		APS_ERR("disable irq...\n");
	}
	schedule_work(&obj->eint_work);
    
	return IRQ_HANDLED;
}

/*----------------------------------------------------------------------------*/
/*for interrup work mode support*/
int rpr521_setup_eint(struct i2c_client *client)
{
	struct rpr521_priv *obj = i2c_get_clientdata(client);       
	APS_LOG("rpr521_setup_eint\n");

	//g_rpr521_ptr = obj;
    
	if (request_irq(obj->irq, rpr521_eint_func, IRQF_TRIGGER_LOW, "ALS-eint", NULL)) {
		APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
		return -EINVAL;
	}
	irq_set_irq_wake(obj->irq, 1);
	//enable_irq(rpr521_obj->irq);
	disable_irq(obj->irq);
	clear_bit(CMC_BIT_IRQ, &obj->enable);
	
	return 0;
}

static int rpr521_init_client(struct i2c_client *client)
{
	struct rpr521_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
#ifdef _TEST_
	u8 data[1];
#endif

	APS_FUN_BEGIN();
	databuf[0] = REG_SYSTEMCONTROL;    
	databuf[1] = REG_SW_RESET | REG_INT_RESET;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return rpr521_ERR_I2C;
	}	
       APS_LOG("rpr521_init_client~ 1\n");
	
	databuf[0] = REG_MODECONTROL;    
	databuf[1] = PS_ALS_SET_MODE_CONTROL|PWRON_PS_ALS;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return rpr521_ERR_I2C;
	}
	
	databuf[0] = REG_ALSPSCONTROL;    
	databuf[1] = PS_ALS_SET_ALSPS_CONTROL;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return rpr521_ERR_I2C;
	}

	databuf[0] = REG_PERSISTENCE;    
	databuf[1] = PS_ALS_SET_INTR_PERSIST;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return rpr521_ERR_I2C;
	}
	APS_LOG("rpr521_init_client~ 2\n");
	/*for interrup work mode support */
	if(0 == obj->hw->polling_mode_ps)
	{

	#ifdef _AUTO_THRESHOLD_CHANGE_		
	    rpr521_calibrate(client);//zzf add
    #else
		databuf[0] = REG_PSTL_LSB;	
		databuf[1] = (u8)(PS_ALS_SET_PS_TL & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return rpr521_ERR_I2C;
		}
			
		databuf[0] = REG_PSTL_MBS;	
		databuf[1] = (u8)((PS_ALS_SET_PS_TL & 0xFF00) >> 8);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return rpr521_ERR_I2C;
		}
			
		databuf[0] = REG_PSTH_LSB;	
		databuf[1] = (u8)( PS_ALS_SET_PS_TH & 0x00FF);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return rpr521_ERR_I2C;
		}
			
		databuf[0] = REG_PSTH_MBS;	
		databuf[1] = (u8)(( PS_ALS_SET_PS_TH & 0xFF00) >> 8);
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return rpr521_ERR_I2C;
		}
    #endif
	
		databuf[0] = REG_INTERRUPT;
		databuf[1] = PS_ALS_SET_INTR | MODE_PROXIMITY;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return rpr521_ERR_I2C;
		}

	}


	databuf[0] = REG_ALSDATA0TH_LSB;    
	databuf[1] = PS_ALS_SET_ALS_TH & 0x00FF ;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return rpr521_ERR_I2C;
	}

      
	databuf[0] = REG_ALSDATA0TH_MBS;    
	databuf[1] = (PS_ALS_SET_ALS_TH& 0xFF00) >> 8;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return rpr521_ERR_I2C;
	}

	/*for interrup work mode support */
	res = rpr521_setup_eint(client);
	if(res)
	{
		APS_ERR("setup eint: %d\n", res);
		return res;
	}

#ifdef _TEST_
	databuf[0]  = REG_SYSTEMCONTROL;
	res = i2c_master_send(client, databuf, 0x1);

	if(res <= 0)
	{
		return -1;
	}
	res = i2c_master_recv(client, data, 0x1);

	if(res <= 0)
	{
		return -1;
	}
	
	APS_DBG(">>>>>>>rpr521: REG_SYSTEMCONTROL    data = %x \n", data[0]);
	
	databuf[0]  = REG_MODECONTROL;
	res = i2c_master_send(client, databuf, 0x1);

	if(res <= 0)
	{
		return -1;
	}
	res = i2c_master_recv(client, data, 0x1);

	if(res <= 0)
	{
		return -1;
	}
	
	APS_DBG(">>>>>>>rpr521: REG_MODECONTROL    data = %x \n", data[0]);
#endif

    APS_FUN_END();

	return 0;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

/******************************************************************************
 * NAME       : ps_als_driver_reset
 * FUNCTION   : reset rpr521 register
 * REMARKS    :
 *****************************************************************************/
#if 0
static int rpr521_ps_als_driver_reset(struct i2c_client *client)
{
	struct rpr521_priv *obj = i2c_get_clientdata(client);
	u8 databuf[2];    
	int res = 0;
   
	databuf[0] = REG_SYSTEMCONTROL;    
	databuf[1] = REG_SW_RESET | REG_INT_RESET;
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return rpr521_ERR_I2C;
	}	

   	return 0;	 	
	
EXIT_ERR:

	APS_ERR("rpr521 reset fail\n");
	return res;
}
#endif

/******************************************************************************
 * NAME       : long_long_divider
 * FUNCTION   : calc divider of unsigned long long int or unsgined long
 * REMARKS    :
 *****************************************************************************/
static void long_long_divider(unsigned long long data, unsigned long base_divier, unsigned long *answer, unsigned long long *overplus)
{
    volatile unsigned long long divier;
    volatile unsigned long      unit_sft;

    if ((long long)data < 0)	// . If data MSB is 1, it may go to endless loop. 
    	{
	*answer = 0;
	*overplus = 0;
	return;		//Theorically, if data is negative, the program will have been returned CALC_ERROR earlier. 
    	}
    divier = base_divier;
    if (data > MASK_LONG) {
        unit_sft = 0;
        while (data > divier) {
            unit_sft++;
            divier = divier << 1;
        }
        while (data > base_divier) {
            if (data > divier) {
                *answer += 1 << unit_sft;
                data    -= divier;
            }
            unit_sft--;
            divier = divier >> 1;
        }
        *overplus = data;
    } else {
        *answer = (unsigned long)(data & MASK_LONG) / base_divier;
        /* calculate over plus and shift 16bit */
        *overplus = (unsigned long long)(data - (*answer * base_divier));
    }
}

/******************************************************************************
 * NAME       : calc_rohm_als_data
 * FUNCTION   : calculate illuminance data for rpr521
 * REMARKS    : final_data is 1000 times, which is defined as CUT_UNIT, of the actual lux value
 *****************************************************************************/
static int calc_rohm_als_data(READ_DATA_BUF data, DEVICE_VAL dev_val)
{
#define DECIMAL_BIT      (15)
#define JUDGE_FIXED_COEF (1000)
#define MAX_OUTRANGE     65535 //(11357) //grace modify in 2016.3.4
#define MAXRANGE_NMODE   (0xFFFF)
#define MAXSET_CASE      (4)
#define CUT_UNIT         1  // 10  modify this for als * 1

	int                final_data, mid_data;
	CALC_DATA          calc_data;
	CALC_ANS           calc_ans;
	unsigned long      calc_judge;
	unsigned char      set_case;
	unsigned long      div_answer;
	unsigned long long div_overplus;
	unsigned long long overplus;
	unsigned long      max_range;

	/* set the value of measured als data */
	calc_data.als_data0  = data.als_data0;
	calc_data.als_data1  = data.als_data1;
	calc_data.gain_data0 = GAIN_TABLE[dev_val.gain].DATA0;

	/* set max range */
	if (calc_data.gain_data0 == 0) 
	{
		/* issue error value when gain is 0 */
		return (CALC_ERROR);
	}
	else
	{
		max_range = MAX_OUTRANGE / calc_data.gain_data0;
	}
	
	/* calculate data */
	if (calc_data.als_data0 == MAXRANGE_NMODE) 
	{
		calc_ans.positive = max_range;
		calc_ans.decimal  = 0;
	} 
	else 
	{
		/* get the value which is measured from power table */
		calc_data.als_time = MCTL_TABLE[dev_val.time].ALS;
		if (calc_data.als_time == 0) 
		{
			/* issue error value when time is 0 */
			return (CALC_ERROR);
		}

		calc_judge = calc_data.als_data1 * JUDGE_FIXED_COEF;
		if (calc_judge < (calc_data.als_data0 * judge_coefficient[0])) 
		{
			set_case = 0;
		} 
		else if (calc_judge < (data.als_data0 * judge_coefficient[1]))
		{
			set_case = 1;
		} 
		else if (calc_judge < (data.als_data0 * judge_coefficient[2])) 
		{
			set_case = 2;
		}
		else if (calc_judge < (data.als_data0 * judge_coefficient[3])) 
		{
			 set_case = 3;
		} 
		else
		{
			set_case = MAXSET_CASE;
		}
		calc_ans.positive = 0;
		if (set_case >= MAXSET_CASE) 
		{
			calc_ans.decimal = 0;	//which means that lux output is 0
		}
		else
		{
			calc_data.gain_data1 = GAIN_TABLE[dev_val.gain].DATA1;
			if (calc_data.gain_data1 == 0) 
			{
				/* issue error value when gain is 0 */
				return (CALC_ERROR);
			}
			calc_data.data0      = (unsigned long long )(data0_coefficient[set_case] * calc_data.als_data0) * calc_data.gain_data1;
			calc_data.data1      = (unsigned long long )(data1_coefficient[set_case] * calc_data.als_data1) * calc_data.gain_data0;
			if (calc_data.data0 < calc_data.data1) 
			{
				/* issue error value when data is negtive */
				return (CALC_ERROR);
			}
			calc_data.data       = (calc_data.data0 - calc_data.data1);
			calc_data.dev_unit   = calc_data.gain_data0 * calc_data.gain_data1 * calc_data.als_time * 10;
			if (calc_data.dev_unit == 0) 
			{
				/* issue error value when dev_unit is 0 */
				return (CALC_ERROR);
			}

			/* calculate a positive number */
			div_answer   = 0;
			div_overplus = 0;
			long_long_divider(calc_data.data, calc_data.dev_unit, &div_answer, &div_overplus);
			calc_ans.positive = div_answer;
			/* calculate a decimal number */
			calc_ans.decimal = 0;
			overplus         = div_overplus;
			if (calc_ans.positive < max_range)
			{
				if (overplus != 0)
				{
					overplus     = overplus << DECIMAL_BIT;
					div_answer   = 0;
					div_overplus = 0;
					long_long_divider(overplus, calc_data.dev_unit, &div_answer, &div_overplus);
					calc_ans.decimal = div_answer;
				}
			}
			else
			{
				calc_ans.positive = max_range;
			}
		}
	}
	
	mid_data = (calc_ans.positive << DECIMAL_BIT) + calc_ans.decimal;
	final_data = calc_ans.positive * CUT_UNIT + ((calc_ans.decimal * CUT_UNIT) >> DECIMAL_BIT);

      /* if final_data  > 65535 , then final_data = 65535*/
      if (MAX_OUTRANGE < final_data)
      {
		final_data = MAX_OUTRANGE;
      }

	return (final_data);

#undef CUT_UNIT
#undef DECIMAL_BIT
#undef JUDGE_FIXED_COEF
#undef MAX_OUTRANGE
#undef MAXRANGE_NMODE
#undef MAXSET_CASE
}


/******************************************************************************
 * NAME       : get_from_device
 * FUNCTION   : periodically reads the data from sensor(thread of work)
 * REMARKS    :
 *****************************************************************************/
static int get_from_device(DEVICE_VAL *dev_val, struct i2c_client *client)
{
#define LEDBIT_MASK   (3)
#define GAIN_VAL_MASK (0xF)

#if 0
	struct PS_ALS_DATA *obj = i2c_get_clientdata(client);	 
	u8 buffer[1];
	int res = 0;
    	unsigned char alsps_ctl[1], read_time[1];

   	 /* initalize the returning value */
    	dev_val->time        = 6;
    	dev_val->gain        = (PS_ALS_SET_ALSPS_CONTROL >> 2) & GAIN_VAL_MASK;
    	dev_val->led_current = PS_ALS_SET_ALSPS_CONTROL & LEDBIT_MASK;

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}

	buffer[0]=REG_MODECONTROL;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, read_time, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	dev_val->time = read_time[0] & 0xF;

	buffer[0]=REG_ALSPSCONTROL;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, alsps_ctl, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

    	dev_val->led_current = alsps_ctl[0] & LEDBIT_MASK;
    	dev_val->gain        = (alsps_ctl[0] >> 2) & GAIN_VAL_MASK;

#else
    	dev_val->time        = 6;
    	dev_val->gain        = (PS_ALS_SET_ALSPS_CONTROL >> 2) & GAIN_VAL_MASK;
    	dev_val->led_current = PS_ALS_SET_ALSPS_CONTROL & LEDBIT_MASK;
#endif

    return (0);
		
//EXIT_ERR:
	APS_ERR("rpr521_read_ps fail\n");

#undef LEDBIT_MASK
#undef GAIN_VAL_MASK

}

/************************************************************
 *                      sysfs interface                    					      *
 ***********************************************************/

/*----------------------------------------------------------------------------*/


static ssize_t show_als_value(struct device_driver *ddri, char *buf)
{
  struct i2c_client *client = rpr521_obj->client;
	u16 als;
	int res;

	if (NULL == client)
	{
		APS_ERR("i2c client is NULL!!!\n");
		return 0;
	}

	rpr521_read_als(client, &als);

	res = snprintf(buf, PAGE_SIZE, "%d\n", als);

	return res;
}

static ssize_t show_ps_value(struct device_driver *ddri, char *buf)
{
  struct i2c_client *client = rpr521_obj->client;
	u16 ps;
	int res;

	if (NULL == client)
	{
		APS_ERR("i2c client is NULL!!!\n");
		return 0;
	}

	rpr521_read_ps(client, &ps);

	res = snprintf(buf, PAGE_SIZE, "%d\n", ps);

	return res;
}

static ssize_t show_registers(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = rpr521_obj->client;
	//int res = 0;
	int len = 0;
	u8 buffer[1];
	u8 reg_val;
	int i;

	if (NULL == client)
	{
		APS_ERR("i2c client is NULL!!!\n");
		return 0;
	}

	mutex_lock(&rpr521_mutex);
	for (i = REG_SYSTEMCONTROL; i < REG_ALSDATA0TL_MBS + 1; i++)
	{
		buffer[0]= i;		
		i2c_master_send(client, buffer, 0x1);
		i2c_master_recv(client, &reg_val, 0x1);		
		len += snprintf(buf + len, PAGE_SIZE - len, "[0x%02x]:0x%02x\n", i, reg_val);		
	}
	mutex_unlock(&rpr521_mutex);
	return len;    
}

static ssize_t store_registers(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = rpr521_obj->client;
	unsigned int addr;
	unsigned int val;
	int ret;
	u8 buffer[2];
           
	ret = sscanf(buf, "%x %x", &addr, &val);

	if (addr < REG_SYSTEMCONTROL || addr > REG_ALSDATA0TL_MBS) {
		APS_ERR("addr 0x%x is out of range!!!\n", addr);
		return -1;
	}

	buffer[0] = addr;
	buffer[1] = val;
	mutex_lock(&rpr521_mutex);
	ret = i2c_master_send(client, buffer, 0x2);
	mutex_unlock(&rpr521_mutex);
	if (ret < 0) {
		APS_ERR("set addr 0x%x, value 0x%x set failed.\n", addr, val);
		return -1;
	}

    return count;
}

static ssize_t rpr521_show_alschannel_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = rpr521_obj->client;
	//u16 als;
	int res;

	if (NULL == client)
	{
		APS_ERR("i2c client is NULL!!!\n");
		return 0;
	}

	//rpr521_read_als(client, &als);

	res = snprintf(buf, PAGE_SIZE, "%d %d\n", rpr521_obj->als_data0, rpr521_obj->als_data1);

	return res;
}

static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = rpr521_i2c_client;
	if(NULL == client)
	{
		APS_ERR("i2c client is null!!\n");
		return 0;
	}	
	return snprintf(buf, PAGE_SIZE, "ROHM Rpr521\n");        
}

static DRIVER_ATTR(als,   S_IRUGO, show_als_value, NULL);
static DRIVER_ATTR(regs,   S_IWUSR | S_IRUGO, show_registers, store_registers);
static DRIVER_ATTR(ps,   S_IRUGO, show_ps_value, NULL);
static DRIVER_ATTR(als_channel_value, S_IRUGO, rpr521_show_alschannel_value, NULL);
static DRIVER_ATTR(chipinfo,             S_IRUGO, show_chipinfo_value,      NULL);


/*----------------------------------------------------------------------------*/
static struct driver_attribute *rpr521_attr_list[] = {
	&driver_attr_als,				/* read als value */
	&driver_attr_regs,				/* registers info */
	&driver_attr_ps,				/* read ps value */
    &driver_attr_als_channel_value,
    &driver_attr_chipinfo,
};
/*----------------------------------------------------------------------------*/
static int rpr521_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(rpr521_attr_list)/sizeof(rpr521_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		err = driver_create_file(driver, rpr521_attr_list[idx]);
		if(err)
		{            
			APS_ERR("driver_create_file (%s) = %d\n", rpr521_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int rpr521_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(rpr521_attr_list)/sizeof(rpr521_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, rpr521_attr_list[idx]);
	}
	
	return err;
}


static void create_rpr521_proc_file(void)
{
	rpr521_proc_file = proc_create("driver/alsps_threshold", 0666, NULL, &rpr521_proc_fops);
    APS_FUN(f);

    if(NULL == rpr521_proc_file)
	{
	    APS_ERR("create_rpr521_proc_file fail!\n");
	}

       //wangmin  add prox calibration value procfile start.
       prox_calibration_value = proc_create("driver/prox_calibration_value", 0666, NULL, &prox_calibration_value_fops);
       if(NULL == prox_calibration_value)
	{
	    APS_ERR("create_rpr521_proc_file prox_calibration_value fail!\n");
	}  
       //wangmin  add prox calibration value procfile end.
	   
/*   20150315 zhangxin add calibration in call   */	
	calibration_inCall = proc_create("driver/calibration_inCall", 0444, NULL, &calibration_inCall_fops);
    if(NULL == calibration_inCall)
	{
	    APS_ERR("create_rpr521_proc_file calibration_inCall  fail!\n");
	}
/*   20150315 zhangxin add calibration in call end  */		
}

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
int rpr521_read_als(struct i2c_client *client, u16 *data)
{
	struct rpr521_priv *obj = i2c_get_clientdata(client);	
	//u8 als_ch0_value_low[1], als_ch0_value_high[1];
	//u8 als_ch1_value_low[1], als_ch1_value_high[1];
	//u8 buffer[1];
	//u16 atio;

	int res = 0;
	READ_DATA_BUF   als_data;
	DEVICE_VAL  dev_val;
	
	if(client == NULL)
	{
		APS_ERR("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
	
#ifdef _TEST_
	APS_DBG("rpr521 ENTER rpr521 read als\n"); 
#endif
	
//get adc channel 0 value
    mutex_lock(&rpr521_mutex);
/*
	buffer[0]=REG_ALSDATA0_LSB;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_ch0_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	buffer[0]=REG_ALSDATA0_MBS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_ch0_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
*/
	res = i2c_smbus_read_word_data(client, REG_ALSDATA0_LSB);
	if (res < 0)
	{
		APS_ERR("read als_ch0 err (%d)\n", res);
		goto EXIT_ERR;
	}
	als_data.als_data0 = res;
//get adc channel 1 value
/*
	buffer[0]=REG_ALSDATA1_LSB;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_ch1_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
		
	buffer[0]=REG_ALSDATA1_MBS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_ch1_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
*/	
	res = i2c_smbus_read_word_data(client, REG_ALSDATA1_LSB);
	if (res < 0)
	{
		APS_ERR("read als_ch0 err (%d)\n", res);
		goto EXIT_ERR;
	}
	als_data.als_data1 = res;
	mutex_unlock(&rpr521_mutex);
	
	//als_data.als_data0 = als_ch0_value_low[0] | (als_ch0_value_high[0]<<8);
	//als_data.als_data1 = als_ch1_value_low[0] | (als_ch1_value_high[0]<<8);

    obj->als_data0 = als_data.als_data0;
	obj->als_data1 = als_data.als_data1;
	
	get_from_device(&dev_val, client);

	*data = calc_rohm_als_data(als_data, dev_val);
	if(*data == 0)
		*data = *data + 1;
	if(*data == CALC_ERROR)
		*data = prev_als_value;	//Report same value as previous.
	else
		prev_als_value = *data;  //grace modified 

    APS_DBG("rpr521-als: [%d, %d] => %d\n", als_data.als_data0, als_data.als_data1,*data);

#ifdef _TEST_    
    rpr521_read_ps(obj->client, &obj->ps); 
#endif
    	
	return 0;	 
	
EXIT_ERR:
	mutex_unlock(&rpr521_mutex);
	APS_ERR("rpr521_read_als fail\n");
	return res;
}


int rpr521_read_als_ch0(struct i2c_client *client, u16 *data)
{
	//struct rpr521_priv *obj = i2c_get_clientdata(client);	 
	u8 als_value_low[1], als_value_high[1];
	u8 buffer[1];
	int res = 0;
	
	if(client == NULL)
	{
		APS_ERR("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
//get adc channel 0 value
    mutex_lock(&rpr521_mutex);
	buffer[0]=REG_ALSDATA0_LSB;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	buffer[0]=REG_ALSDATA0_MBS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, als_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	mutex_unlock(&rpr521_mutex);
	*data = als_value_low[0] | (als_value_high[0]<<8);

	return 0;	 

	
	
EXIT_ERR:
	mutex_unlock(&rpr521_mutex);
	APS_ERR("rpr521_read_als fail\n");
	return res;
}
/*----------------------------------------------------------------------------*/

static int rpr521_get_als_value(struct rpr521_priv *obj, u16 als)
{
	//int idx;
	bool changed = false;
	int invalid = 0;
	u16 als_lux = 0;
/*	
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als < obj->hw->als_level[idx])
		{
			break;
		}
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
*/
	if(-1 != als_value_backup)
	{   	
	    if (als > 20) // >20lux
		{
			if ((als < als_value_backup * 7/10) || (als > als_value_backup * 14/10))
			{
				APS_DBG("rpr521-ALS: caculate lux\n"); 
				als_lux = als * 11 / 10;
				changed = true;
			}
			else
			{
				APS_DBG("rpr521-ALS: in debounce, use backup lux\n");
				als_lux = lux_backup;
				changed = false;
			}			
		}
	    else if (als >= 0)  //0-20lux
	   	{
			if ((als < als_value_backup - 8) || (als > als_value_backup + 8) || (als < 1))
			{
				APS_DBG("rpr521-ALS: caculate lux\n"); 
				als_lux = als * 11 / 10;
				changed = true;
			}
			else
			{
				APS_DBG("rpr521-ALS: in debounce, use backup lux\n");
				als_lux = lux_backup;
				changed = false;
			}
	   	}
		else
		{
			APS_DBG("rpr521-ALS: als < 0, als = %05d\n", als);
			als_lux = lux_backup;
			changed = false;
		}
	}
	else
	{
		APS_DBG("rpr521-ALS: ----als_value_backup = -1\n"); 
		als_lux = als * 11 / 10;
		changed = true;
	}

	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->als_deb_on, 0);
		}
		
		if(1 == atomic_read(&obj->als_deb_on))
		{
			invalid = 1;
		}
	}

	if(!invalid)
	{
		APS_DBG("ALS: raw data %05d => value = %05d\n", als, als_lux); //obj->hw->als_value[idx]);	
		//return obj->hw->als_value[idx];
		if (changed)
		{
		    als_value_backup = als;
            lux_backup = als_lux;
		}
		return als_lux;
	}
	else
	{
		APS_ERR("ALS: %05d => %05d (-1)\n", als, als_lux); //obj->hw->als_value[idx]);
		als_value_backup = -1;
		lux_backup = -1;
		return -1;
	}

}
/*----------------------------------------------------------------------------*/
long rpr521_read_ps(struct i2c_client *client, u16 *data)
{
   	//struct rpr521_priv *obj = i2c_get_clientdata(client);	 
	//u16 ps_value;
	//u8 ps_value_low[1], ps_value_high[1];
	//u8 buffer[1];
	int res = 0;
#ifdef _TEST_
	u8 mode[1], int_sts[1]; 
#endif

	if(client == NULL)
	{
		APS_DBG("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
    mutex_lock(&rpr521_mutex);
/*	
	buffer[0]=REG_PSDATA_LSB;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ps_value_low, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}

	buffer[0]=REG_PSDATA_MBS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, ps_value_high, 0x01);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
*/
	res = i2c_smbus_read_word_data(client, REG_PSDATA_LSB);
	if (res < 0)
	{
		APS_ERR("read ps err (%d)\n", res);
		goto EXIT_ERR;
	}
    mutex_unlock(&rpr521_mutex);
	//*data = (ps_value_low[0] | (ps_value_high[0]<<8))& 0xfff;
	//APS_DBG("ps_data=%d, low:%d  high:%d\n", *data, ps_value_low[0], ps_value_high[0]);
	*data = res & 0x0fff;
	APS_DBG("ps_data = %d\n", *data);
#ifdef _TEST_

	buffer[0]=REG_MODECONTROL;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, mode, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	buffer[0]=REG_INTERRUPT;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	res = i2c_master_recv(client, int_sts, 0x1);
	if(res <= 0)
	{
		goto EXIT_ERR;
	}
	
	APS_DBG("ROHM rpr521--ps_data=%d, low:%d  high:%d", *data, ps_value_low[0], ps_value_high[0]); 
	APS_DBG("ROHM rpr521--mode=%x, int_sts:%x", mode[0], int_sts[0]);

#endif	
	return 0;    

EXIT_ERR:
	mutex_unlock(&rpr521_mutex);
	APS_ERR("rpr521_read_ps fail\n");
	return res;
}


#define REG_PSTH_MAX	0xFFF
#define REG_PSTL_MAX	0xFFF

static int rpr521_set_ps_threshold(struct i2c_client *client, u16 ps_tl, u16 ps_th)
{
	u8 databuf[2];
  	//u8 buffer[2];  
 	int res = 0;

	APS_FUN();//20130109
	mutex_lock(&rpr521_mutex);
	databuf[0] = REG_PSTL_LSB;	
	databuf[1] = (u8)(ps_tl & 0x00FF);
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto I2CERR;
	}
			
	databuf[0] = REG_PSTL_MBS;	
	databuf[1] = (u8)((ps_tl & 0xFF00) >> 8);
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto I2CERR;
	}
			
	databuf[0] = REG_PSTH_LSB;	
	databuf[1] = (u8)( ps_th & 0x00FF);
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto I2CERR;
	}
			
	databuf[0] = REG_PSTH_MBS;	
	databuf[1] = (u8)(( ps_th & 0xFF00) >> 8);
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto I2CERR;
	}
    mutex_unlock(&rpr521_mutex);
	APS_DBG(">>>>>>rpr521: ps_tl=%d , ps_th=%d\n", ps_tl, ps_th);//20130109
	return 0;
	
I2CERR:
	mutex_unlock(&rpr521_mutex);
	APS_ERR("rpr521 i2c err!");
	return rpr521_ERR_I2C;
}

/*----------------------------------------------------------------------------*/
static int rpr521_get_ps_value(struct rpr521_priv *obj, u16 ps)
{
	int val; // mask = atomic_read(&obj->ps_mask);
	int invalid = 0;
	static int val_temp=1;
	//u16 temp_ps[1];
	
	APS_FUN();
	//APS_DBG("PS raw data:  %05d => \n", ps);

       //mdelay(160);

#ifdef _AUTO_THRESHOLD_CHANGE_
	if(ps > obj->ps_th_h)
	{
		val = 1;  /*close*/
		val_temp = 1;
		if(0 == obj->hw->polling_mode_ps)
			rpr521_set_ps_threshold(obj->client,obj->ps_th_l,REG_PSTH_MAX);
	}
	else if(ps < obj->ps_th_l)
	{
		val = 5;  /*far away*/
		val_temp = 5;
		if(0 == obj->hw->polling_mode_ps)
			rpr521_set_ps_threshold(obj->client,0,obj->ps_th_h);

	}
	else
		 val = val_temp;	
#else
	if(ps >= PS_ALS_SET_PS_TH)
	{
		val = 0;  /*close*/
		val_temp = 0;
	#if 1//def _AUTO_THRESHOLD_CHANGE_
		if(0 == obj->hw->polling_mode_ps)
			rpr521_set_ps_threshold(obj->client,PS_ALS_SET_PS_TL,REG_PSTH_MAX);
	#endif

	}
	else if(ps <= PS_ALS_SET_PS_TL)
	{
		val = 5;  /*far away*/
		val_temp = 5;
	#if 1//def _AUTO_THRESHOLD_CHANGE_
		if(0 == obj->hw->polling_mode_ps)
			rpr521_set_ps_threshold(obj->client,0,PS_ALS_SET_PS_TH);
      #endif

	}
	else
		 val = val_temp;	
#endif			
	
	if(atomic_read(&obj->ps_suspend))
	{
		invalid = 1;
	}
	else if(1 == atomic_read(&obj->ps_deb_on))
	{
		unsigned long endt = atomic_read(&obj->ps_deb_end);
		if(time_after(jiffies, endt))
		{
			atomic_set(&obj->ps_deb_on, 0);
		}
		
		if (1 == atomic_read(&obj->ps_deb_on))
		{
			invalid = 1;
		}
	}
	
	APS_DBG("RPR521-PS:  %05d => %05d\n", 	ps, val);

	if(!invalid)
	{
		return val;
	}	
	else
	{
		return -1;
	}	
}


/*----------------------------------------------------------------------------*/
/*for interrup work mode support*/
static void rpr521_eint_work(struct work_struct *work)
{
	struct rpr521_priv *obj = (struct rpr521_priv *)container_of(work, struct rpr521_priv, eint_work);
	int err;
	struct hwm_sensor_data sensor_data;

	//int res, int_status;
	//u8 buffer[2];

	
	int result = 0;
	int status;    
	//READ_DATA_BUF read_data_buf;

	//DEVICE_VAL    dev_val;
	//long          get_timer;
	//long          wait_sec;
	//unsigned long wait_nsec;
	APS_FUN();

	if(0 == obj->hw->polling_mode_ps)
	{
		result =  rpr521_check_and_clear_intr(obj->client);
		
		if(result < 0)
		{
			APS_DBG("ERROR! read interrupt status. \n");
		}
		else  
		{
			status = result;
			if(status & PS_INT_MASK)
			{
				rpr521_read_ps(obj->client, &obj->ps);
			}
			if(status & ALS_INT_MASK) // 2 kinds of interrupt may occur at same time
			{
			}
			if(!((status & ALS_INT_MASK) || (status & PS_INT_MASK)))
			{
				APS_DBG( "Unknown interrupt source.\n");
				goto EXIT;
			}
			
			sensor_data.values[0] = rpr521_get_ps_value(obj, obj->ps);
			sensor_data.value_divide = 1;
			sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;			

			//let up layer to know
			if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
			{
		 		 APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
			}
		}		
		//enable_irq(ps_als->client->irq);
	}
	else
	{
		rpr521_read_ps(obj->client, &obj->ps);
		//mdelay(160);
		APS_DBG("rpr521_eint_work rawdata ps=%d als_ch0=%d!\n",obj->ps,obj->als);
		sensor_data.values[0] = rpr521_get_ps_value(obj, obj->ps);
		sensor_data.value_divide = 1;
		sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;			

		//let up layer to know
		if((err = hwmsen_get_interrupt_data(ID_PROXIMITY, &sensor_data)))
		{
		  APS_ERR("call hwmsen_get_interrupt_data fail = %d\n", err);
		}		
	}

EXIT:	
	if (0 == test_and_set_bit(CMC_BIT_IRQ, &obj->enable))
	{
		enable_irq(rpr521_obj->irq); //mt_eint_unmask(CUST_EINT_ALS_NUM);     
		APS_ERR("enable irq...\n");
	}
}


/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int rpr521_open(struct inode *inode, struct file *file)
{
	file->private_data = rpr521_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int rpr521_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}


/*----------------------------------------------------------------------------*/
static long rpr521_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct rpr521_priv *obj = i2c_get_clientdata(client);  
	long err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
#ifdef _TEST_	
	APS_DBG("%s:cmd=%x\n",__func__,cmd);
#endif
	switch (cmd)
	{
		case ALSPS_SET_PS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
#ifdef _TEST
			APS_DBG("%s:enable=%x\n",__func__,enable);
#endif
			if(enable)
			{
				if((err = rpr521_enable_ps(obj->client, 1)))
				{
					APS_ERR("enable ps fail: %ld\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_PS, &obj->enable);
				
#ifdef _TEST_	
				APS_DBG("rpr521 enable ps success\n");
#endif
			}
			else
			{
				if((err = rpr521_enable_ps(obj->client, 0)))
				{
					APS_ERR("disable ps fail: %ld\n", err); 
					goto err_out;
				}
				clear_bit(CMC_BIT_PS, &obj->enable);
#ifdef _TEST_	
				APS_DBG("rpr521 disable ps success\n");
#endif
			}
			break;

		case ALSPS_GET_PS_MODE:
			enable = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:    
			if((err = rpr521_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			
			dat = rpr521_get_ps_value(obj, obj->ps);
#ifdef _TEST_	
			APS_DBG("rpr521 ALSPS_GET_PS_DATA %d\n", dat); 
#endif			
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;

		case ALSPS_GET_PS_RAW_DATA:    
			if((err = rpr521_read_ps(obj->client, &obj->ps)))
			{
				goto err_out;
			}
			
			dat = obj->ps;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}  
			break;              

		case ALSPS_SET_ALS_MODE:
			if(copy_from_user(&enable, ptr, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			if(enable)
			{
				if((err = rpr521_enable_als(obj->client, 1)))
				{
					APS_ERR("enable als fail: %ld\n", err); 
					goto err_out;
				}
				set_bit(CMC_BIT_ALS, &obj->enable);
#ifdef _TEST_	
				APS_DBG("rpr521 enable als success\n");
#endif
			}
			else
			{
				if((err = rpr521_enable_als(obj->client, 0)))
				{
					APS_ERR("disable als fail: %ld\n", err); 
					goto err_out;
				}
				clear_bit(CMC_BIT_ALS, &obj->enable);
#ifdef _TEST_	
				APS_DBG("rpr521 disable als success\n"); 
#endif 
			}
			break;

		case ALSPS_GET_ALS_MODE:
			enable = test_bit(CMC_BIT_ALS, &obj->enable) ? (1) : (0);
			if(copy_to_user(ptr, &enable, sizeof(enable)))
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_ALS_DATA: 
			if((err = rpr521_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = rpr521_get_als_value(obj, obj->als);
#ifdef _TEST_	
			APS_DBG("rpr521 ALSPS_GET_ALS_DATA data=%d, level=%d\n", obj->als,dat); 
#endif
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;

		case ALSPS_GET_ALS_RAW_DATA:    
			if((err = rpr521_read_als(obj->client, &obj->als)))
			{
				goto err_out;
			}

			dat = obj->als;
			if(copy_to_user(ptr, &dat, sizeof(dat)))
			{
				err = -EFAULT;
				goto err_out;
			}              
			break;
			
		default:
			APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
			err = -ENOIOCTLCMD;
			break;
	}

	err_out:
	return err;    
}
/*----------------------------------------------------------------------------*/
static struct file_operations rpr521_fops = {
	.owner = THIS_MODULE,
	.open = rpr521_open,
	.release = rpr521_release,
	.unlocked_ioctl = rpr521_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice rpr521_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &rpr521_fops,
};
/*----------------------------------------------------------------------------*/
static int rpr521_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct rpr521_priv *obj = i2c_get_clientdata(client);    
	int err;
	APS_FUN();    

	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(!obj)
		{
			APS_ERR("null pointer!!\n");
			return -EINVAL;
		}
		
		atomic_set(&obj->als_suspend, 1);
		err = rpr521_enable_als(client, 0);
		if(err)
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}
/*
		atomic_set(&obj->ps_suspend, 1);		
		err = rpr521_enable_ps(client, 0);
		if(err)
		{
			APS_ERR("disable ps:  %d\n", err);
			return err;
		}
*/		
		rpr521_power(obj->hw, 0);
	}

	return 0;
}
/*----------------------------------------------------------------------------*/
static int rpr521_i2c_resume(struct i2c_client *client)
{
	struct rpr521_priv *obj = i2c_get_clientdata(client);        
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	rpr521_power(obj->hw, 1);
/*	
	err = rpr521_init_client(client);
	if(err)
	{
		APS_ERR("initialize client fail!!\n");
		return err;        
	}
*/	
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		err = rpr521_enable_als(client, 1);
		if(err)
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}
/*	
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		err = rpr521_enable_ps(client, 1);
		if(err)
		{
			APS_ERR("enable ps fail: %d\n", err);                
		}
	}
*/
	return 0;
}
/*----------------------------------------------------------------------------*/
#if 0
static void rpr521_early_suspend(struct early_suspend *h) 
{   /*early_suspend is only applied for ALS*/
	struct rpr521_priv *obj = container_of(h, struct rpr521_priv, early_drv);   
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	#if 1
	atomic_set(&obj->als_suspend, 1);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if((err = rpr521_enable_als(obj->client, 0)))
		{
			APS_ERR("disable als fail: %d\n", err); 
		}
	}
	#endif
}
/*----------------------------------------------------------------------------*/

static void rpr521_late_resume(struct early_suspend *h)
{   /*early_suspend is only applied for ALS*/
	struct rpr521_priv *obj = container_of(h, struct rpr521_priv, early_drv);       
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

        #if 1
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
		if((err = rpr521_enable_als(obj->client, 1)))
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
	}
	#endif
}
#endif

int rpr521_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data* sensor_data;
	struct rpr521_priv *obj = (struct rpr521_priv *)self;
	
	APS_DBG("rpr521_ps_operate command1:%d SENSOR_ENABLE:%d\n",command,(uint32_t)SENSOR_ENABLE);
	
	APS_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{
					if((err = rpr521_enable_ps(obj->client, 1)))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_PS, &obj->enable);
					#if 0	
					if(err = rpr521_enable_als(obj->client, 1))
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
					#endif
				}
				else
				{
					if((err = rpr521_enable_ps(obj->client, 0)))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_PS, &obj->enable);
					#if 0
					if(err = rpr521_enable_als(obj->client, 0))
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
					#endif
				}
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (struct hwm_sensor_data *)buff_out;	
				rpr521_read_ps(obj->client, &obj->ps);
				
                                //mdelay(160);
				APS_ERR("rpr521_ps_operate als data=%d!\n",obj->als);
				sensor_data->values[0] = rpr521_get_ps_value(obj, obj->ps);
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;		
				APS_LOG("rpr521_ps_operate ps raw data=%d!, value=%d\n", obj->ps, sensor_data->values[0]);
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

/*----------------------------------------------------------------------------*/

int rpr521_als_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	struct hwm_sensor_data* sensor_data;
	struct rpr521_priv *obj = (struct rpr521_priv *)self;
    APS_DBG("rpr521_als_operate command1:%d SENSOR_ENABLE:%d\n",command,(uint32_t)SENSOR_ENABLE);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;				
				if(value)
				{
					if((err = rpr521_enable_als(obj->client, 1)))
					{
						APS_ERR("enable als fail: %d\n", err); 
						return -1;
					}
					set_bit(CMC_BIT_ALS, &obj->enable);
				}
				else
				{
					if((err = rpr521_enable_als(obj->client, 0)))
					{
						APS_ERR("disable als fail: %d\n", err); 
						return -1;
					}
					clear_bit(CMC_BIT_ALS, &obj->enable);
				}
				
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(struct hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (struct hwm_sensor_data *)buff_out;
				/*yucong MTK add for fixing know issue*/
				rpr521_read_als(obj->client, &obj->als);
		// for AAL  GAOYUAN
		//		#if defined(MTK_AAL_SUPPORT)
		//		sensor_data->values[0] = obj->als;
		//		#else
				sensor_data->values[0] = rpr521_get_als_value(obj, obj->als);
		//		#endif
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;
		default:
			APS_ERR("light sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}

static int get_median(unsigned int tmpdata[20])
{ 
     unsigned int tmp = 0;
     int i, j;     
	 
     for(i = 0; i< 10 ; i++)
     {
         printk("P%d=%d", i , tmpdata[i]);
     }
     printk("\n");
     for(i = 10; i< 20 ; i++)
     {
         printk("P%d=%d", i , tmpdata[i]);
     }
     printk("\n");	 
	 
     for(i = 0; i< 19 ; i++)
     {
         for(j = i+1; j< 20 ; j++)
         {
	      if(tmpdata[i] > tmpdata[j])
	      {
	           tmp = tmpdata[i];
		    tmpdata[i] = tmpdata[j];	   
		    tmpdata[j] = tmp;	
	      }
         }
     }
    
     return tmpdata[10];
}

static int rpr521_enalbe_ps_without_irq(struct i2c_client *client, int enable)
{
	u8 databuf[2];	  
	int res = 0;
	
	APS_LOG("%s enable:%d \n", __func__, enable);

	if(client == NULL)
	{
		APS_ERR("CLIENT CANN'T EQUL NULL\n");
		return -1;
	}
 
	if(enable)
	{
		mutex_lock(&rpr521_mutex);
	/*	
	    databuf[0]= REG_MODECONTROL;
		res = i2c_master_send(client, databuf, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		res = i2c_master_recv(client, databuf, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
	*/	
		res = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
		if (res < 0)
		{
			APS_ERR("read RegModeControl err (%d)\n", res);
			goto EXIT_ERR;
		}
		databuf[0] = res;
		APS_LOG("%s: RegModeControl [ps_enable]= 0x%x\n", __func__, databuf[0]);

		databuf[0] &= (~ALS_PS_MEASUREMENT_MASK);
		databuf[1] = databuf[0] | PS_ENABLE | ALS_PS_MEASUREMENT_100MS;             
	/*	
		databuf[0] = REG_MODECONTROL;				   
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
	*/
		res = i2c_smbus_write_byte_data(client, REG_MODECONTROL, databuf[1]);
		if(res < 0)
		{
			APS_ERR("write RegModeControl err (%d)\n", res);
			goto EXIT_ERR;
		}
		APS_LOG("%s: ps_enable RegModeControl set to 0x%x\n", __func__, databuf[1]);
		
		mutex_unlock(&rpr521_mutex);		
				
	}
	else
	{
		mutex_lock(&rpr521_mutex);

	/*	
		databuf[0]= REG_MODECONTROL;
		res = i2c_master_send(client, databuf, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		res = i2c_master_recv(client, databuf, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
	*/
		res = i2c_smbus_read_byte_data(client, REG_MODECONTROL);
		if (res < 0)
		{
			APS_ERR("read REG_MODECONTROL err\n");
			goto EXIT_ERR;
		}
		databuf[0] = res;
		APS_LOG("%s: RegModeControl [ps_disable]= 0x%x\n", __func__, databuf[0]);
		
		databuf[1] = databuf[0] & (~PS_ENABLE);             
	/*	databuf[0] = REG_MODECONTROL;		
					   
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
	*/
		res = i2c_smbus_write_byte_data(client, REG_MODECONTROL, databuf[1]);
		if(res < 0)
		{
			goto EXIT_ERR;
		}
		APS_LOG("%s: ps_disable RegModeControl set to 0x%x\n", __func__, databuf[1]);
		
		mutex_unlock(&rpr521_mutex);		
	}
	return 0;
EXIT_ERR:
	mutex_unlock(&rpr521_mutex);
	APS_ERR("R/W REG_MODCONTROL err\n");
	return -1;
}

static int rpr521_get_threshold(struct i2c_client *client)
{
       struct rpr521_priv *obj = i2c_get_clientdata(client);
	int average = 0;
 	unsigned int i, tmp;
 	//unsigned int ps_th_h, ps_th_l;
	unsigned int tmpdata[20];
 	//u8 buffer[2];  
  	//int res = 0;
  	//u8 ps_value_low[1], ps_value_high[1]; // syssta[1], modectl[1];	
	int rpr521_ps_original_power;

  	APS_LOG("rpr521_get_threshold\n");	
	rpr521_ps_original_power = test_bit(CMC_BIT_PS, &obj->enable) ? (1) : (0);
    if(0 == rpr521_ps_original_power)
    {
	   if(rpr521_enalbe_ps_without_irq(rpr521_i2c_client, 1))
	   {
	       APS_ERR("%s-rpr521_enalbe_ps_without_irq err, return -1\n", __func__);
           return -1;
	   }
    }
	//mutex_lock(&rpr521_mutex);
	for(i = 0; i < 20; i ++)
	{

	/*
		buffer[0]=REG_PSDATA_LSB;
		res = i2c_master_send(client, buffer, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		res = i2c_master_recv(client, ps_value_low, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}

		buffer[0]=REG_PSDATA_MBS;
		res = i2c_master_send(client, buffer, 0x1);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		res = i2c_master_recv(client, ps_value_high, 0x01);
		if(res <= 0)
		{
			goto EXIT_ERR;
		}
		
		tmp = ps_value_low[0] | (ps_value_high[0]<<8);
     */   
     	tmp = i2c_smbus_read_word_data(client, REG_PSDATA_LSB);
		if(tmp < 0)
		{
			APS_ERR("%s: i2c read ps data fail. \n", __func__);
			goto EXIT_ERR;
		}
		//average = tmp & 0xFFF;	// 12 bit data
		tmpdata[i] = tmp & 0xFFF;	// 12 bit data
		APS_DBG("%s: calibrate data %d = %d. \n", __func__, i, tmpdata[i]);
	}
	//mutex_unlock(&rpr521_mutex);
	//average /= 20;
	average = get_median(tmpdata);
   	APS_LOG("rpr521_get_threshold the average is :%d!\n", average);
	if(0 == rpr521_ps_original_power)
    {
	     rpr521_enalbe_ps_without_irq(rpr521_i2c_client, 0);
	}
	return average;

EXIT_ERR:
	//mutex_unlock(&rpr521_mutex);
	return -1;
}

static int rpr521_set_threshold(int average)
{
	struct rpr521_priv *obj = i2c_get_clientdata(rpr521_i2c_client);	
 	unsigned int ps_th_h, ps_th_l;	
  	u8 databuf[2];
 	// u8 buffer[2];  
  	int res = 0;
  	//u8 ps_value_low[1], ps_value_high[1], syssta[1], modectl[1];

	APS_LOG("rpr521_set_threshold,average=%d\n", average);
	if(average < 0)
	{		
		goto EXIT_ERR;
	}

	rpr521_check_prox_mean(average); 
	ps_th_h = average + THRES_TOLERANCE + THRES_DEFAULT_DIFF;
	ps_th_l = average + THRES_TOLERANCE;

	APS_LOG("[andy]%s: calculated threshold: %d -> %d.\n", __FUNCTION__, ps_th_l, ps_th_h);

	if(ps_th_h < 0)
	{
		APS_ERR("%s: high threshold is less than 0.\n", __func__);
		goto EXIT_ERR;
	}
	if(ps_th_h > 0xfff)
	{
		APS_ERR("%s: high threshold is greater than maximum allowed value.\n", __func__);
		goto EXIT_ERR;
	}

	if(ps_th_l < 0)
	{
		APS_ERR("%s: low threshold is less than 0.\n", __func__);
		goto EXIT_ERR;
   }
	if(ps_th_l > 0xfff)
	{
		APS_ERR("%s: low threshold is greater than maximum allowed value.\n", __func__);
		goto EXIT_ERR;
	}
     mutex_lock(&rpr521_mutex);
     databuf[0] = REG_PSTL_LSB;
     databuf[1] = (u8)(ps_th_l & 0x00FF);
     res = i2c_master_send(rpr521_i2c_client, databuf, 0x2);
     if(res <= 0)
     {
        goto EXIT_ERR;
     }
     
     databuf[0] = REG_PSTL_MBS;
     databuf[1] = (u8)( ps_th_l  & 0xFF00 >> 8);
     res = i2c_master_send(rpr521_i2c_client, databuf, 0x2);
     if(res <= 0)
     {
        goto EXIT_ERR;
     }

     databuf[0] = REG_PSTH_LSB;
     databuf[1] = (u8)(ps_th_h & 0x00FF);
     res = i2c_master_send(rpr521_i2c_client, databuf, 0x2);
     if(res <= 0) 
     {
        goto EXIT_ERR;
     }
     
     databuf[0] = REG_PSTH_MBS;
     databuf[1] = (u8)(ps_th_h & 0xFF00 >> 8);
     res = i2c_master_send(rpr521_i2c_client, databuf, 0x2);
     if(res <= 0) 
     {
         goto EXIT_ERR;
     }
    mutex_unlock(&rpr521_mutex);
	obj->ps_th_h = ps_th_h;
	obj->ps_th_l = ps_th_l;

	APS_LOG("rpr521_set_threshold OK:ps_th_h=%d,sps_th_l=%d\n", ps_th_h, ps_th_l);	
    return 0;
	
EXIT_ERR:
	mutex_unlock(&rpr521_mutex);
	rpr521_enable_ps(rpr521_i2c_client, 0);  //disable ps
	APS_LOG("rpr521_set_threshold fail, disable ps!\n");
	return -1;
}


#ifdef _AUTO_THRESHOLD_CHANGE_
static int rpr521_calibrate(struct i2c_client *client)
{	
	int average;	       
  	u8 databuf[2]; 	 
//	int res = 0;

  	APS_DBG("%s:rpr521_get_threshold\n", __func__);
/*	
	databuf[0] = REG_MODECONTROL;    
	databuf[1] = 0x41;    //PS 10ms
	res = i2c_master_send(rpr521_i2c_client, databuf, 0x2);
	if(res <= 0)
	{
		return -1;
	}	
*/	
	average = rpr521_get_threshold(client);
	if(average>= 0)
	{
	 	rpr521_set_threshold(average); 
	}
	else
	{ 
		APS_ERR("rpr521_get_threshold error,average=%d\n", average);
	}
	databuf[0] = REG_MODECONTROL;    
	databuf[1] = PWRON_PS_ALS;    //disable ps
	i2c_master_send(client,databuf, 0x2);
	
	return average;
	
}
#endif


/*----------------------------------------------------------------------------*/
static int rpr521_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    	
	strcpy(info->type, RPR521_DEV_NAME);	
	return 0;
}

static int of_get_rpr521_platform_data(struct device *dev)
{
	struct device_node *node = NULL;

	node = of_find_compatible_node(NULL, NULL, "mediatek, ALS-eint");
	if (node) {
		//alsps_int_gpio_number = of_get_named_gpio(node, "int-gpio", 0);
		alsps_irq = irq_of_parse_and_map(node, 0);
		if (alsps_irq < 0) {
			APS_ERR("alsps request_irq IRQ LINE NOT AVAILABLE!.");
			return -1;
		}
		APS_ERR("alsps_irq : %d\n", alsps_irq);
	}
	return 0;
}

/*----------------------------------------------------------------------------*/
static int rpr521_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct rpr521_priv *obj;
	struct hwmsen_object obj_ps, obj_als;
	int err = 0;
	APS_FUN_BEGIN();

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	rpr521_obj = obj;

	obj->hw = hw; //get_rpr521_alsps_hw();
	rpr521_get_addr(obj->hw, &obj->addr);

	/*for interrup work mode support*/
	INIT_WORK(&obj->eint_work, rpr521_eint_work);
	APS_LOG("%s: OK\n", "INIT_WORK ");
	obj->client = client;
	i2c_set_clientdata(client, obj);
	APS_LOG("rpr521_i2c_probe~1 \n");
	atomic_set(&obj->als_debounce, 50);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 10);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->ps_suspend, 0);
	atomic_set(&obj->als_cmd_val, 0xDF);
	atomic_set(&obj->ps_cmd_val,  0xC1);
	APS_LOG("rpr521_i2c_probe~2 \n");
	//atomic_set(&obj->ps_thd_val_high,  obj->hw->ps_threshold_high);//grace modified in 2013.8.28
	//atomic_set(&obj->ps_thd_val_low,  obj->hw->ps_threshold_low);
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]); 
	APS_LOG("rpr521_i2c_probe~3 \n");
	
	obj->als_modulus = (400*100*40)/(1*1500);

	of_get_rpr521_platform_data(&client->dev);
	obj->irq = alsps_irq;
/*    
	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	APS_LOG("rpr521_i2c_probe~4 \n");
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
*/	
	atomic_set(&obj->i2c_retry, 3);
	clear_bit(CMC_BIT_ALS, &obj->enable);
	clear_bit(CMC_BIT_PS, &obj->enable);
	clear_bit(CMC_BIT_IRQ, &obj->enable);

	
	rpr521_i2c_client = client;

	APS_LOG("rpr521_i2c_probe~ 5\n");
	if((err = rpr521_init_client(client)))
	{
		goto exit_init_failed;
	}
	APS_LOG("rpr521_init_client() OK!\n");

	if((err = misc_register(&rpr521_device)))
	{
		APS_ERR("rpr521_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	obj_ps.self = rpr521_obj;
	/*for interrup work mode support*/
	if(1 == obj->hw->polling_mode_ps)
	{
		obj_ps.polling = 1;
	}
	else
	{
		obj_ps.polling = 0;
	}

	obj_ps.sensor_operate = rpr521_ps_operate;
	if((err = hwmsen_attach(ID_PROXIMITY, &obj_ps)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	
	obj_als.self = rpr521_obj;
	obj_als.polling = 1;
	obj_als.sensor_operate = rpr521_als_operate;
	if((err = hwmsen_attach(ID_LIGHT, &obj_als)))
	{
		APS_ERR("attach fail = %d\n", err);
		goto exit_create_attr_failed;
	}
	APS_LOG("%s: OK\n", "hwmsen_attach  ");

	//if (rpr521_create_attr(&(rpr521_alsps_driver.driver)))
	{
		//APS_ERR("create attr failed.\n");
	}
    /* Register sysfs attribute */
	err = rpr521_create_attr(&rpr521_init_info.platform_diver_addr->driver);
	if(err) //by xianjuxiang 130729
	{
		APS_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
       create_rpr521_proc_file();

	APS_LOG("%s: OK\n", __func__);
	APS_FUN_END();
	return 0;

	exit_create_attr_failed:
	misc_deregister(&rpr521_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(client);
	//exit_kfree:
	kfree(obj);
	exit:
	rpr521_i2c_client = NULL;           
//	mt16_EINTIRQMask(CUST_WM_EINT_ALS_NUM);  /*mask interrupt if fail*/
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}
/*----------------------------------------------------------------------------*/
static int rpr521_i2c_remove(struct i2c_client *client)
{
	int err;	

	if (rpr521_delete_attr(&(rpr521_alsps_driver.driver)))
	{
		APS_ERR("delete attr failed.\n");
	}
  
	if((err = misc_deregister(&rpr521_device)))
	{
		APS_ERR("misc_deregister fail: %d\n", err);    
	}
	
	rpr521_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}
/*----------------------------------------------------------------------------*/

static int alsps_rpr521_init(void) 
{
	APS_FUN_BEGIN();

	rpr521_power(hw, 1);    
	
	if(i2c_add_driver(&rpr521_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	APS_FUN_END();
	return 0;
}
static int alsps_rpr521_uninit(void) 
{
	APS_FUN();    
	rpr521_power(hw, 0);     
	i2c_del_driver(&rpr521_i2c_driver);
	return 0;
}
#if 0
static int rpr521_probe(struct platform_device *pdev) 
{
	struct alsps_hw *hw = get_rpr521_alsps_hw();
	APS_FUN_BEGIN();

	rpr521_power(hw, 1);    
	
	if(i2c_add_driver(&rpr521_i2c_driver))
	{
		APS_ERR("add driver error\n");
		return -1;
	} 
	APS_FUN_END();
	return 0;
}
/*----------------------------------------------------------------------------*/
static int rpr521_remove(struct platform_device *pdev)
{
	struct alsps_hw *hw = get_rpr521_alsps_hw();
	APS_FUN();    
	rpr521_power(hw, 0);    
	i2c_del_driver(&rpr521_i2c_driver);
	return 0;
}
/*----------------------------------------------------------------------------*/
static struct platform_driver rpr521_alsps_driver = {
	.probe      = rpr521_probe,
	.remove     = rpr521_remove,    
	.driver     = {
		.name  = "als_ps",
		.owner = THIS_MODULE,
	}
};
#endif

/*----------------------------------------------------------------------------*/
static int __init rpr521_init(void)
{
	APS_FUN_BEGIN();
    
#if 0
	struct alsps_hw *hw = get_rpr521_alsps_hw();
	APS_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num); 
	i2c_register_board_info(hw->i2c_num, &i2c_RPR521, 1);
#endif

	hw = get_alsps_dts_func("mediatek,rpr521", hw);
	if (!hw) {
		APS_ERR("get dts info fail\n");
		return 0;
	}
    
	alsps_driver_add(&rpr521_init_info);
	#if 0
	if(platform_driver_register(&rpr521_alsps_driver))
	{
		APS_ERR("failed to register driver");
		return -ENODEV;
	}
	#endif
	
	APS_FUN_END();
	return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit rpr521_exit(void)
{
	APS_FUN();
	#if 0
	platform_driver_unregister(&rpr521_alsps_driver);
	#endif
}
/*----------------------------------------------------------------------------*/
module_init(rpr521_init);
module_exit(rpr521_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("Grace Huang");
MODULE_DESCRIPTION("rpr521 driver");
MODULE_LICENSE("GPL");
