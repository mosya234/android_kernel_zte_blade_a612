/* drivers/i2c/chips/lis2dh12.c - LIS2DH12 motion sensor driver
 *
 *
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
/*
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
*/
#include <linux/platform_device.h>
#include <asm/atomic.h>

#include <cust_acc.h>
#include "lis2dh12.h"

//#include <mach/mt_typedefs.h>
//#include <mach/mt_gpio.h>
//#include <mach/mt_pm_ldo.h>
#include <accel.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/pinctrl/pinctrl-state.h>
#include <linux/proc_fs.h>

//#define POWER_NONE_MACRO MT65XX_POWER_NONE

/*----------------------------------------------------------------------------*/
//#define I2C_DRIVERID_LIS2DH12 345
/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
//#define CONFIG_LIS2DH12_LOWPASS   /*apply low pass filter on output*/       
/*----------------------------------------------------------------------------*/
#define LIS2DH12_AXIS_X          0
#define LIS2DH12_AXIS_Y          1
#define LIS2DH12_AXIS_Z          2
#define LIS2DH12_AXES_NUM        3
#define LIS2DH12_DATA_LEN        6
#define LIS2DH12_DEV_NAME        "LIS2DH12"
/*----------------------------------------------------------------------------*/
static const struct i2c_device_id lis2dh12_i2c_id[] = {{LIS2DH12_DEV_NAME,0},{}};
/*the adapter id will be available in customization*/
//static struct i2c_board_info __initdata i2c_LIS2DH12={ I2C_BOARD_INFO("LIS2DH12", (0x30>>1))};

#define LIS2DH12_CALI_LEN            (10)
#define LIS2DH12_CALI_TOLERANCE		(1000) //mg

struct acc_hw accel_cust;
static struct acc_hw *hw = &accel_cust;

/* For  driver get cust info */
struct acc_hw *get_cust_acc(void)
{
	return &accel_cust;
}

#ifdef CONFIG_OF
static const struct of_device_id accel_of_match[] = {
	{.compatible = "mediatek,gsensor"},
	{.compatible = "mediatek,lis2dh12"},
	{},
};
#endif

//static unsigned short lis2dh12_force[] = {0x00, LIS2DH12_I2C_SLAVE_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const lis2dh12_forces[] = { lis2dh12_force, NULL };
//static struct i2c_client_address_data lis2dh12_addr_data = { .forces = lis2dh12_forces,};

/*----------------------------------------------------------------------------*/
static int lis2dh12_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int lis2dh12_i2c_remove(struct i2c_client *client);
static int lis2dh12_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
//#ifndef USE_EARLY_SUSPEND
static int lis2dh12_suspend(struct i2c_client *client, pm_message_t msg);
static int lis2dh12_resume(struct i2c_client *client);
//#endif
static struct proc_dir_entry *acc_calibrate_proc_file = NULL;

extern struct acc_hw* lis2dh12_get_cust_acc_hw(void);

static int  lis2dh12_local_init(void);
static int  lis2dh12_remove(void);
static int lis2dh12_init_flag =-1; // 0<==>OK -1 <==> fail
/*----------------------------------------------------------------------------*/
typedef enum {
    ADX_TRC_FILTER  = 0x01,
    ADX_TRC_RAWDATA = 0x02,
    ADX_TRC_IOCTL   = 0x04,
    ADX_TRC_CALI	= 0X08,
    ADX_TRC_INFO	= 0X10,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][LIS2DH12_AXES_NUM];
    int sum[LIS2DH12_AXES_NUM];
    int num;
    int idx;
};

/*----------------------------------------------------------------------------*/
static struct acc_init_info lis2dh12_init_info = {
		.name = "lis2dh12",
		.init = lis2dh12_local_init,
		.uninit = lis2dh12_remove,
};
/*----------------------------------------------------------------------------*/
struct lis2dh12_i2c_data {
    struct i2c_client *client;
    struct acc_hw *hw;
    struct hwmsen_convert   cvt;
    
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
	atomic_t				filter;
    s16                     cali_sw[LIS2DH12_AXES_NUM+1];

    /*data*/
    s8                      offset[LIS2DH12_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s16                     data[LIS2DH12_AXES_NUM+1];

#if 1//defined(CONFIG_LIS2DH12_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
    /*early suspend*/
#ifdef USE_EARLY_SUSPEND
    struct early_suspend    early_drv;
#endif     
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver lis2dh12_i2c_driver = {
    .driver = {
        .owner          = THIS_MODULE,
        .name           = LIS2DH12_DEV_NAME,
#ifdef CONFIG_OF
	  .of_match_table = accel_of_match,
#endif
    },
	.probe      		= lis2dh12_i2c_probe,
	.remove    			= lis2dh12_i2c_remove,
	.detect				= lis2dh12_i2c_detect,
//#if !defined(USE_EARLY_SUSPEND)    
    .suspend            = lis2dh12_suspend,
    .resume             = lis2dh12_resume,
//#endif
	.id_table = lis2dh12_i2c_id,
//	.address_data = &lis2dh12_addr_data,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *lis2dh12_i2c_client = NULL;
static struct lis2dh12_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = true;
static struct GSENSOR_VECTOR3D gsensor_gain, gsensor_offset;
//static char selftestRes[10] = {0};
static DEFINE_MUTEX(lis2dh12_i2c_mutex);
static DEFINE_MUTEX(lis2dh12_op_mutex);
static bool enable_status = false;
static int sensor_suspend = 0;

/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               printk(KERN_INFO GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk(KERN_ERR GSE_TAG fmt, ##args)
/*----------------------------------------------------------------------------*/
static struct data_resolution lis2dh12_data_resolution[] = {
     /* combination by {FULL_RES,RANGE}*/
    {{ 1, 0}, 1000},   // dataformat +/-2g  in 12-bit resolution;  { 1, 0} = 1.0 = (2*2*1000)/(2^12);  1024 = (2^12)/(2*2) 
    {{ 1, 9}, 500},   // dataformat +/-4g  in 12-bit resolution;  { 1, 9} = 1.9 = (2*4*1000)/(2^12);  512 = (2^12)/(2*4) 
    {{ 3, 9}, 250},   // dataformat +/-8g  in 12-bit resolution;  { 1, 0} = 1.0 = (2*8*1000)/(2^12);  1024 = (2^12)/(2*8) 
};
/*----------------------------------------------------------------------------*/
static struct data_resolution lis2dh12_offset_resolution = {{15, 6}, 64};
#if 0
/*--------------------read function----------------------------------*/
static int lis_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
    return hwmsen_read_block(client, addr, data, len);

    u8 beg = addr;
	int err;
	struct i2c_msg msgs[2]={{0},{0}};
	
	mutex_lock(&lis2dh12_i2c_mutex);
	
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len =1;
	msgs[0].buf = &beg;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len =len;
	msgs[1].buf = data;
	
	if (!client)
	{
	    mutex_unlock(&lis2dh12_i2c_mutex);
		return -EINVAL;
	}
	else if (len > C_I2C_FIFO_SIZE) 
	{
		GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&lis2dh12_i2c_mutex);
		return -EINVAL;
	}
	err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
	//GSE_LOG(" lis_i2c_read_block return value  %d\n", err);
	if (err < 0) 
	{
		GSE_ERR("i2c_transfer error: (%d %p %d) %d\n",addr, data, len, err);
		err = -EIO;
	} 
	else 
	{
		err = 0;
	}
	mutex_unlock(&lis2dh12_i2c_mutex);
	return err; //if success will return 0

}

static int lis_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{   /*because address also occupies one byte, the maximum length for write is 7 bytes*/
    int err, idx, num;
    char buf[C_I2C_FIFO_SIZE];
    err =0;
	mutex_lock(&lis2dh12_i2c_mutex);
    if (!client)
    {
        mutex_unlock(&lis2dh12_i2c_mutex);
        return -EINVAL;
    }
    else if (len >= C_I2C_FIFO_SIZE) 
	{        
        GSE_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
		mutex_unlock(&lis2dh12_i2c_mutex);
        return -EINVAL;
    }    

    num = 0;
    buf[num++] = addr;
    for (idx = 0; idx < len; idx++)
    {
        buf[num++] = data[idx];
    }

    err = i2c_master_send(client, buf, num);
    if (err < 0)
	{
        GSE_ERR("send command error!!\n");
		mutex_unlock(&lis2dh12_i2c_mutex);
        return -EFAULT;
    } 
	mutex_unlock(&lis2dh12_i2c_mutex);
    return err; //if success will return transfer lenth
}
/*----------------------------------------------------------------------------*/
#endif
static void dumpReg(struct i2c_client *client)
{
  int i=0;
  u8 addr = 0x20;
  u8 regdata=0;
  for(i=0; i<3 ; i++)
  {
    //dump all
    hwmsen_read_block(client,addr,&regdata,1);
	GSE_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
	addr++;
  }
}
/*--------------------ADXL power control function----------------------------------*/
static void LIS2DH12_power(struct acc_hw *hw, unsigned int on) 
{
#if 0
	static unsigned int power_on = 0;

	if(hw->power_id != POWER_NONE_MACRO)		// have externel LDO
	{        
		GSE_LOG("power %s\n", on ? "on" : "off");
		if(power_on == on)	// power status not change
		{
			GSE_LOG("ignore power control: %d\n", on);
		}
		else if(on)	// power on
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "LIS2DH12"))
			{
				GSE_ERR("power on fails!!\n");
			}
		}
		else	// power off
		{
			if (!hwPowerDown(hw->power_id, "LIS2DH12"))
			{
				GSE_ERR("power off fail!!\n");
			}			  
		}
	}
	power_on = on;    
#endif
}
/*----------------------------------------------------------------------------*/
static int LIS2DH12_SetDataResolution(struct lis2dh12_i2c_data *obj)
{
	int err;
	u8  dat, reso;

	if((err = hwmsen_read_block(obj->client, LIS2DH12_REG_CTL_REG4, &dat,0x01))<0)
	{
		GSE_ERR("write data format fail!!\n");
		return err;
	}

	/*the data_reso is combined by 3 bits: {FULL_RES, DATA_RANGE}*/
	reso  = (dat & 0x30)<<4;
	if(reso >= 0x3)
		reso = 0x2;
	

	if(reso < sizeof(lis2dh12_data_resolution)/sizeof(lis2dh12_data_resolution[0]))
	{        
		obj->reso = &lis2dh12_data_resolution[reso];
		return 0;
	}
	else
	{
		return -EINVAL;
	}
}
/*----------------------------------------------------------------------------*/
static int LIS2DH12_ReadData(struct i2c_client *client, s16 data[LIS2DH12_AXES_NUM])
{
	struct lis2dh12_i2c_data *priv = i2c_get_clientdata(client);        
	//u8 addr = LIS2DH12_REG_DATAX0;
	u8 buf[LIS2DH12_DATA_LEN] = {0};
	int err = 0;

	if(NULL == client)
	{
		err = -EINVAL;
	}
	
	else
	{
		if((hwmsen_read_block(client, LIS2DH12_REG_OUT_X, buf, 0x01))<0)
	    {
		   GSE_ERR("read  G sensor data register err!\n");
		     return -1;
	    }
		if((hwmsen_read_block(client, LIS2DH12_REG_OUT_X+1, &buf[1], 0x01))<0)
	    {
		   GSE_ERR("read  G sensor data register err!\n");
		     return -1;
	    }
		
	    data[LIS2DH12_AXIS_X] = (s16)((buf[0]+(buf[1]<<8))>>4);
	if((hwmsen_read_block(client, LIS2DH12_REG_OUT_Y, &buf[2], 0x01))<0)
	    {
		   GSE_ERR("read  G sensor data register err!\n");
		     return -1;
	    }
	if((hwmsen_read_block(client, LIS2DH12_REG_OUT_Y+1, &buf[3], 0x01))<0)
	    {
		   GSE_ERR("read  G sensor data register err!\n");
		     return -1;
	    }
		
	    data[LIS2DH12_AXIS_Y] =  (s16)((s16)(buf[2] +( buf[3]<<8))>>4);
		
	if((hwmsen_read_block(client, LIS2DH12_REG_OUT_Z, &buf[4], 0x01))<0)
	    {
		   GSE_ERR("read  G sensor data register err!\n");
		     return -1;
	    }

	if((hwmsen_read_block(client, LIS2DH12_REG_OUT_Z+1, &buf[5], 0x01))<0)
	    {
		   GSE_ERR("read  G sensor data register err!\n");
		     return -1;
	    }
		
	    data[LIS2DH12_AXIS_Z] =(s16)((buf[4]+(buf[5]<<8))>>4);

	//GSE_LOG("[%08X %08X %08X %08x %08x %08x]\n",buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);

	
		data[LIS2DH12_AXIS_X] &= 0xfff;
		data[LIS2DH12_AXIS_Y] &= 0xfff;
		data[LIS2DH12_AXIS_Z] &= 0xfff;


		if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
		{
			GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d]\n", data[LIS2DH12_AXIS_X], data[LIS2DH12_AXIS_Y], data[LIS2DH12_AXIS_Z],
		                               data[LIS2DH12_AXIS_X], data[LIS2DH12_AXIS_Y], data[LIS2DH12_AXIS_Z]);
		}

		if(data[LIS2DH12_AXIS_X]&0x800)
		{
				data[LIS2DH12_AXIS_X] = ~data[LIS2DH12_AXIS_X];
				data[LIS2DH12_AXIS_X] &= 0xfff;
				data[LIS2DH12_AXIS_X]+=1;
				data[LIS2DH12_AXIS_X] = -data[LIS2DH12_AXIS_X];
		}
		if(data[LIS2DH12_AXIS_Y]&0x800)
		{
				data[LIS2DH12_AXIS_Y] = ~data[LIS2DH12_AXIS_Y];
				data[LIS2DH12_AXIS_Y] &= 0xfff;
				data[LIS2DH12_AXIS_Y]+=1;
				data[LIS2DH12_AXIS_Y] = -data[LIS2DH12_AXIS_Y];
		}
		if(data[LIS2DH12_AXIS_Z]&0x800)
		{
				data[LIS2DH12_AXIS_Z] = ~data[LIS2DH12_AXIS_Z];
				data[LIS2DH12_AXIS_Z] &= 0xfff;
				data[LIS2DH12_AXIS_Z]+=1;
				data[LIS2DH12_AXIS_Z] = -data[LIS2DH12_AXIS_Z];
		}

		if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
		{
			GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d] after\n", data[LIS2DH12_AXIS_X], data[LIS2DH12_AXIS_Y], data[LIS2DH12_AXIS_Z],
		                               data[LIS2DH12_AXIS_X], data[LIS2DH12_AXIS_Y], data[LIS2DH12_AXIS_Z]);
		}
		
#ifdef CONFIG_LIS2DH12_LOWPASS
		if(atomic_read(&priv->filter))
		{
			if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
			{
				int idx, firlen = atomic_read(&priv->firlen);   
				if(priv->fir.num < firlen)
				{                
					priv->fir.raw[priv->fir.num][LIS2DH12_AXIS_X] = data[LIS2DH12_AXIS_X];
					priv->fir.raw[priv->fir.num][LIS2DH12_AXIS_Y] = data[LIS2DH12_AXIS_Y];
					priv->fir.raw[priv->fir.num][LIS2DH12_AXIS_Z] = data[LIS2DH12_AXIS_Z];
					priv->fir.sum[LIS2DH12_AXIS_X] += data[LIS2DH12_AXIS_X];
					priv->fir.sum[LIS2DH12_AXIS_Y] += data[LIS2DH12_AXIS_Y];
					priv->fir.sum[LIS2DH12_AXIS_Z] += data[LIS2DH12_AXIS_Z];
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
							priv->fir.raw[priv->fir.num][LIS2DH12_AXIS_X], priv->fir.raw[priv->fir.num][LIS2DH12_AXIS_Y], priv->fir.raw[priv->fir.num][LIS2DH12_AXIS_Z],
							priv->fir.sum[LIS2DH12_AXIS_X], priv->fir.sum[LIS2DH12_AXIS_Y], priv->fir.sum[LIS2DH12_AXIS_Z]);
					}
					priv->fir.num++;
					priv->fir.idx++;
				}
				else
				{
					idx = priv->fir.idx % firlen;
					priv->fir.sum[LIS2DH12_AXIS_X] -= priv->fir.raw[idx][LIS2DH12_AXIS_X];
					priv->fir.sum[LIS2DH12_AXIS_Y] -= priv->fir.raw[idx][LIS2DH12_AXIS_Y];
					priv->fir.sum[LIS2DH12_AXIS_Z] -= priv->fir.raw[idx][LIS2DH12_AXIS_Z];
					priv->fir.raw[idx][LIS2DH12_AXIS_X] = data[LIS2DH12_AXIS_X];
					priv->fir.raw[idx][LIS2DH12_AXIS_Y] = data[LIS2DH12_AXIS_Y];
					priv->fir.raw[idx][LIS2DH12_AXIS_Z] = data[LIS2DH12_AXIS_Z];
					priv->fir.sum[LIS2DH12_AXIS_X] += data[LIS2DH12_AXIS_X];
					priv->fir.sum[LIS2DH12_AXIS_Y] += data[LIS2DH12_AXIS_Y];
					priv->fir.sum[LIS2DH12_AXIS_Z] += data[LIS2DH12_AXIS_Z];
					priv->fir.idx++;
					data[LIS2DH12_AXIS_X] = priv->fir.sum[LIS2DH12_AXIS_X]/firlen;
					data[LIS2DH12_AXIS_Y] = priv->fir.sum[LIS2DH12_AXIS_Y]/firlen;
					data[LIS2DH12_AXIS_Z] = priv->fir.sum[LIS2DH12_AXIS_Z]/firlen;
					if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
					{
						GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
						priv->fir.raw[idx][LIS2DH12_AXIS_X], priv->fir.raw[idx][LIS2DH12_AXIS_Y], priv->fir.raw[idx][LIS2DH12_AXIS_Z],
						priv->fir.sum[LIS2DH12_AXIS_X], priv->fir.sum[LIS2DH12_AXIS_Y], priv->fir.sum[LIS2DH12_AXIS_Z],
						data[LIS2DH12_AXIS_X], data[LIS2DH12_AXIS_Y], data[LIS2DH12_AXIS_Z]);
					}
				}
			}
		}	
#endif         
	}
	return err;
}
/*----------------------------------------------------------------------------*/
/*
static int LIS2DH12_ReadOffset(struct i2c_client *client, s8 ofs[LIS2DH12_AXES_NUM])
{    
	int err;

	return err;    
}
*/
/*----------------------------------------------------------------------------*/
static int LIS2DH12_ResetCalibration(struct i2c_client *client)
{
	struct lis2dh12_i2c_data *obj = i2c_get_clientdata(client);	

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	return 0;     
}
/*----------------------------------------------------------------------------*/
static int LIS2DH12_ReadCalibration(struct i2c_client *client, int dat[LIS2DH12_AXES_NUM])
{
    struct lis2dh12_i2c_data *obj = i2c_get_clientdata(client);

    dat[obj->cvt.map[LIS2DH12_AXIS_X]] = obj->cvt.sign[LIS2DH12_AXIS_X]*obj->cali_sw[LIS2DH12_AXIS_X];
    dat[obj->cvt.map[LIS2DH12_AXIS_Y]] = obj->cvt.sign[LIS2DH12_AXIS_Y]*obj->cali_sw[LIS2DH12_AXIS_Y];
    dat[obj->cvt.map[LIS2DH12_AXIS_Z]] = obj->cvt.sign[LIS2DH12_AXIS_Z]*obj->cali_sw[LIS2DH12_AXIS_Z];                        
                                       
    return 0;
}
/*----------------------------------------------------------------------------*/
/*
static int LIS2DH12_ReadCalibrationEx(struct i2c_client *client, int act[LIS2DH12_AXES_NUM], int raw[LIS2DH12_AXES_NUM])
{  
	
	struct lis2dh12_i2c_data *obj = i2c_get_clientdata(client);
	int err;
	int mul;

	if(err = LIS2DH12_ReadOffset(client, obj->offset))
	{
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}    

	mul = obj->reso->sensitivity/lis2dh12_offset_resolution.sensitivity;
	raw[LIS2DH12_AXIS_X] = obj->offset[LIS2DH12_AXIS_X]*mul + obj->cali_sw[LIS2DH12_AXIS_X];
	raw[LIS2DH12_AXIS_Y] = obj->offset[LIS2DH12_AXIS_Y]*mul + obj->cali_sw[LIS2DH12_AXIS_Y];
	raw[LIS2DH12_AXIS_Z] = obj->offset[LIS2DH12_AXIS_Z]*mul + obj->cali_sw[LIS2DH12_AXIS_Z];

	act[obj->cvt.map[LIS2DH12_AXIS_X]] = obj->cvt.sign[LIS2DH12_AXIS_X]*raw[LIS2DH12_AXIS_X];
	act[obj->cvt.map[LIS2DH12_AXIS_Y]] = obj->cvt.sign[LIS2DH12_AXIS_Y]*raw[LIS2DH12_AXIS_Y];
	act[obj->cvt.map[LIS2DH12_AXIS_Z]] = obj->cvt.sign[LIS2DH12_AXIS_Z]*raw[LIS2DH12_AXIS_Z];                        
	                       
	return 0;
}
*/
/*----------------------------------------------------------------------------*/
static int LIS2DH12_WriteCalibration(struct i2c_client *client, int dat[LIS2DH12_AXES_NUM])
{
	struct lis2dh12_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	//int cali[LIS2DH12_AXES_NUM];


	GSE_FUN();
	if(!obj || ! dat)
	{
		GSE_ERR("null ptr!!\n");
		return -EINVAL;
	}
	else
	{        
		s16 cali[LIS2DH12_AXES_NUM];
		cali[obj->cvt.map[LIS2DH12_AXIS_X]] = obj->cvt.sign[LIS2DH12_AXIS_X]*obj->cali_sw[LIS2DH12_AXIS_X];
		cali[obj->cvt.map[LIS2DH12_AXIS_Y]] = obj->cvt.sign[LIS2DH12_AXIS_Y]*obj->cali_sw[LIS2DH12_AXIS_Y];
		cali[obj->cvt.map[LIS2DH12_AXIS_Z]] = obj->cvt.sign[LIS2DH12_AXIS_Z]*obj->cali_sw[LIS2DH12_AXIS_Z]; 
		cali[LIS2DH12_AXIS_X] += dat[LIS2DH12_AXIS_X];
		cali[LIS2DH12_AXIS_Y] += dat[LIS2DH12_AXIS_Y];
		cali[LIS2DH12_AXIS_Z] += dat[LIS2DH12_AXIS_Z];

		obj->cali_sw[LIS2DH12_AXIS_X] += obj->cvt.sign[LIS2DH12_AXIS_X]*dat[obj->cvt.map[LIS2DH12_AXIS_X]];
        obj->cali_sw[LIS2DH12_AXIS_Y] += obj->cvt.sign[LIS2DH12_AXIS_Y]*dat[obj->cvt.map[LIS2DH12_AXIS_Y]];
        obj->cali_sw[LIS2DH12_AXIS_Z] += obj->cvt.sign[LIS2DH12_AXIS_Z]*dat[obj->cvt.map[LIS2DH12_AXIS_Z]];
	} 

	return err;
}
/*----------------------------------------------------------------------------*/
/*
static int LIS2DH12_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[10];    
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = LIS2DH12_REG_DEVID;    

	res = i2c_master_send(client, databuf, 0x1);
	if(res <= 0)
	{
		goto exit_LIS2DH12_CheckDeviceID;
	}
	
	udelay(500);

	databuf[0] = 0x0;        
	res = i2c_master_recv(client, databuf, 0x01);
	if(res <= 0)
	{
		goto exit_LIS2DH12_CheckDeviceID;
	}
	

	if(databuf[0]!=LIS2DH12_FIXED_DEVID)
	{
		return LIS2DH12_ERR_IDENTIFICATION;
	}

	exit_LIS2DH12_CheckDeviceID:
	if (res <= 0)
	{
		return LIS2DH12_ERR_I2C;
	}
	return LIS2DH12_SUCCESS;
}
*/
/*----------------------------------------------------------------------------*/
static int LIS2DH12_SetPowerMode(struct i2c_client *client, bool enable)
{
	u8 databuf[2];    
	int res = 0;
	u8 addr = LIS2DH12_REG_CTL_REG1;
	struct lis2dh12_i2c_data *obj = i2c_get_clientdata(client);
	
	//GSE_LOG("enter Sensor power status is sensor_power = %d\n",sensor_power);

	if(enable == sensor_power)
	{
		GSE_LOG("Sensor power status is newest!\n");
		return LIS2DH12_SUCCESS;
	}

	if((hwmsen_read_block(client, addr, databuf, 0x01))<0)
	{
		GSE_ERR("read power ctl register err!\n");
		return LIS2DH12_ERR_I2C;
	}

	if(enable == true)
	{
		databuf[0] &=  ~LIS2DH12_MEASURE_MODE;
		databuf[0] = databuf[0]| 0x10; 
	}
	else
	{
		databuf[0] = databuf[0] & 0x0F;
	}
	
	res = hwmsen_write_block(client, LIS2DH12_REG_CTL_REG1, databuf, 0x1);

	if(res)
	{
		GSE_LOG("set power mode failed!\n");
		return LIS2DH12_ERR_I2C;
	}
	else if(atomic_read(&obj->trace) & ADX_TRC_INFO)
	{
		GSE_LOG("set power mode ok %d!\n", databuf[1]);
	}

	sensor_power = enable;
	//GSE_LOG("leave Sensor power status is sensor_power = %d\n",sensor_power);
	return LIS2DH12_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int LIS2DH12_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct lis2dh12_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];
	u8 addr = LIS2DH12_REG_CTL_REG4;
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);

	if((hwmsen_read_block(client, addr, databuf, 0x01))<0)
	{
		GSE_ERR("read reg_ctl_reg1 register err!\n");
		return LIS2DH12_ERR_I2C;
	}

	databuf[0] &= ~0x30;
	databuf[0] |=dataformat;

	res = hwmsen_write_block(client, LIS2DH12_REG_CTL_REG4, databuf, 0x1);

	if(res)
	{
		return LIS2DH12_ERR_I2C;
	}
	

	return LIS2DH12_SetDataResolution(obj);    
}
/*----------------------------------------------------------------------------*/
static int LIS2DH12_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	u8 databuf[10];
	u8 addr = LIS2DH12_REG_CTL_REG1;
	int res = 0;

	memset(databuf, 0, sizeof(u8)*10);
	
	if((hwmsen_read_block(client, addr, databuf, 0x01))<0)
	{
		GSE_ERR("read reg_ctl_reg1 register err!\n");
		return LIS2DH12_ERR_I2C;
	}

	databuf[0] &= ~0xF0;
	databuf[0] |= bwrate;

	res = hwmsen_write_block(client, LIS2DH12_REG_CTL_REG1, databuf, 0x1);

	if(res)
	{
		return LIS2DH12_ERR_I2C;
	}
	
	return LIS2DH12_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
//enalbe data ready interrupt
static int LIS2DH12_SetIntEnable(struct i2c_client *client, u8 intenable)
{
	u8 databuf[2];
	u8 addr = LIS2DH12_REG_CTL_REG3;
	int res = 0;

	memset(databuf, 0, sizeof(u8)*2); 

	if((hwmsen_read_block(client, addr, databuf, 0x01))<0)
	{
		GSE_ERR("read reg_ctl_reg1 register err!\n");
		return LIS2DH12_ERR_I2C;
	}

	databuf[0] = 0x00;

	res = hwmsen_write_block(client, LIS2DH12_REG_CTL_REG3, databuf, 0x01);
	if(res)
	{
		return LIS2DH12_ERR_I2C;
	}
	
	return LIS2DH12_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int LIS2DH12_Init(struct i2c_client *client, int reset_cali)
{
	struct lis2dh12_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;
	u8 databuf[2] = {0, 0};
/*
	res = LIS2DH12_CheckDeviceID(client); 
	if(res != LIS2DH12_SUCCESS)
	{
		return res;
	}	
*/	
    // first clear reg1
    databuf[0] = 0x07;    // not low powermode
    res = hwmsen_write_block(client, LIS2DH12_REG_CTL_REG1, databuf, 0x01);
	if(res)
	{
		GSE_ERR("LIS2DH12_Init step 1!\n");
		return res;
	}

	databuf[0] = 0x88;    // not low powermode, BDU SET
    res = hwmsen_write_block(client, LIS2DH12_REG_CTL_REG4, databuf, 0x01);
	if(res)
	{
		GSE_ERR("LIS2DH12_Init step 1!\n");
		return res;
	}

	res = LIS2DH12_SetBWRate(client, LIS2DH12_BW_100HZ);//400 or 100 no other choice
	if(res < 0)
	{
		GSE_ERR("LIS2DH12_Init step 2!\n");
		return res;
	}

	res = LIS2DH12_SetDataFormat(client, LIS2DH12_RANGE_2G);//8g or 2G no oher choise
	if(res < 0) 
	{
		GSE_ERR("LIS2DH12_Init step 3!\n");
		return res;
	}
	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

	res = LIS2DH12_SetIntEnable(client, false);        
	if(res < 0)
	{
		GSE_ERR("LIS2DH12_Init step 4!\n");
		return res;
	}
	
	res = LIS2DH12_SetPowerMode(client, enable_status);//false);
	if(res < 0)
	{
		GSE_ERR("LIS2DH12_Init step 5!\n");
		return res;
	}

	if(0 != reset_cali)
	{ 
		//reset calibration only in power on
		res = LIS2DH12_ResetCalibration(client);
		if(res < 0)
		{
			return res;
		}
	}

#ifdef CONFIG_LIS2DH12_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif

	return LIS2DH12_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int LIS2DH12_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	u8 databuf[10];    

	memset(databuf, 0, sizeof(u8)*10);

	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}
	
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "LIS2DH12 Chip");
	return 0;
}
/*----------------------------------------------------------------------------*/
static int LIS2DH12_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct lis2dh12_i2c_data *obj = (struct lis2dh12_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[LIS2DH12_AXES_NUM];
	int res = 0;
	memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	if(sensor_suspend == 1)
	{
		//GSE_LOG("sensor in suspend read not data!\n");
		return 0;
	}
#if 0
	if(sensor_power == FALSE)
	{
		res = LIS2DH12_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on lis2dh12 error %d!\n", res);
		}
		msleep(20);
	}
#endif
	if((res = LIS2DH12_ReadData(client, obj->data)))
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
		//Out put the mg
		obj->data[LIS2DH12_AXIS_X] = obj->data[LIS2DH12_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		obj->data[LIS2DH12_AXIS_Y] = obj->data[LIS2DH12_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		obj->data[LIS2DH12_AXIS_Z] = obj->data[LIS2DH12_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;		
        
		obj->data[LIS2DH12_AXIS_X] += obj->cali_sw[LIS2DH12_AXIS_X];
		obj->data[LIS2DH12_AXIS_Y] += obj->cali_sw[LIS2DH12_AXIS_Y];
		obj->data[LIS2DH12_AXIS_Z] += obj->cali_sw[LIS2DH12_AXIS_Z];
		
		/*remap coordinate*/
		acc[obj->cvt.map[LIS2DH12_AXIS_X]] = obj->cvt.sign[LIS2DH12_AXIS_X]*obj->data[LIS2DH12_AXIS_X];
		acc[obj->cvt.map[LIS2DH12_AXIS_Y]] = obj->cvt.sign[LIS2DH12_AXIS_Y]*obj->data[LIS2DH12_AXIS_Y];
		acc[obj->cvt.map[LIS2DH12_AXIS_Z]] = obj->cvt.sign[LIS2DH12_AXIS_Z]*obj->data[LIS2DH12_AXIS_Z];

		//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[LIS2DH12_AXIS_X], acc[LIS2DH12_AXIS_Y], acc[LIS2DH12_AXIS_Z]);

		sprintf(buf, "%04x %04x %04x", acc[LIS2DH12_AXIS_X], acc[LIS2DH12_AXIS_Y], acc[LIS2DH12_AXIS_Z]);
		if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)//atomic_read(&obj->trace) & ADX_TRC_IOCTL
		{
			GSE_LOG("gsensor data: %s!\n", buf);
			dumpReg(client);
		}
	}
	
	return 0;
}
/*----------------------------------------------------------------------------*/
static int LIS2DH12_ReadRawData(struct i2c_client *client, char *buf)
{
	struct lis2dh12_i2c_data *obj = (struct lis2dh12_i2c_data*)i2c_get_clientdata(client);
	int res = 0;

	if (!buf || !client)
	{
		return EINVAL;
	}
	
	if((res = LIS2DH12_ReadData(client, obj->data)))
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return EIO;
	}
	else
	{
		sprintf(buf, "%04x %04x %04x", obj->data[LIS2DH12_AXIS_X], 
			obj->data[LIS2DH12_AXIS_Y], obj->data[LIS2DH12_AXIS_Z]);
	
	}
	
	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis2dh12_i2c_client;
	char strbuf[LIS2DH12_BUFSIZE];
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	LIS2DH12_ReadChipInfo(client, strbuf, LIS2DH12_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis2dh12_i2c_client;
	char strbuf[LIS2DH12_BUFSIZE];
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	LIS2DH12_ReadSensorData(client, strbuf, LIS2DH12_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}
/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis2dh12_i2c_client;
	struct lis2dh12_i2c_data *obj;
	int err, len, mul;
	int tmp[LIS2DH12_AXES_NUM];	
	len = 0;

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);

	

	
	if((err = LIS2DH12_ReadCalibration(client, tmp)))
	{
		return -EINVAL;
	}
	else
	{    
		mul = obj->reso->sensitivity/lis2dh12_offset_resolution.sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
			obj->offset[LIS2DH12_AXIS_X], obj->offset[LIS2DH12_AXIS_Y], obj->offset[LIS2DH12_AXIS_Z],
			obj->offset[LIS2DH12_AXIS_X], obj->offset[LIS2DH12_AXIS_Y], obj->offset[LIS2DH12_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
			obj->cali_sw[LIS2DH12_AXIS_X], obj->cali_sw[LIS2DH12_AXIS_Y], obj->cali_sw[LIS2DH12_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
			obj->offset[LIS2DH12_AXIS_X]*mul + obj->cali_sw[LIS2DH12_AXIS_X],
			obj->offset[LIS2DH12_AXIS_Y]*mul + obj->cali_sw[LIS2DH12_AXIS_Y],
			obj->offset[LIS2DH12_AXIS_Z]*mul + obj->cali_sw[LIS2DH12_AXIS_Z],
			tmp[LIS2DH12_AXIS_X], tmp[LIS2DH12_AXIS_Y], tmp[LIS2DH12_AXIS_Z]);
		
		return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = lis2dh12_i2c_client;  
	int err, x, y, z;
	int dat[LIS2DH12_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if((err = LIS2DH12_ResetCalibration(client)))
		{
			GSE_ERR("reset offset err = %d\n", err);
		}	
	}
	else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
	{
		dat[LIS2DH12_AXIS_X] = x;
		dat[LIS2DH12_AXIS_Y] = y;
		dat[LIS2DH12_AXIS_Z] = z;
		if((err = LIS2DH12_WriteCalibration(client, dat)))
		{
			GSE_ERR("write calibration err = %d\n", err);
		}		
	}
	else
	{
		GSE_ERR("invalid format\n");
	}
	
	return count;
}
/*----------------------------------------------------------------------------*/

static ssize_t show_power_status(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = lis2dh12_i2c_client;
	struct lis2dh12_i2c_data *obj;
	u8 data;

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);
	hwmsen_read_block(client,LIS2DH12_REG_CTL_REG1,&data,0x01);
    	return snprintf(buf, PAGE_SIZE, "%x\n", data);
}
/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_LIS2DH12_LOWPASS
	struct i2c_client *client = lis2dh12_i2c_client;
	struct lis2dh12_i2c_data *obj = i2c_get_clientdata(client);
	if(atomic_read(&obj->firlen))
	{
		int idx, len = atomic_read(&obj->firlen);
		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for(idx = 0; idx < len; idx++)
		{
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][LIS2DH12_AXIS_X], obj->fir.raw[idx][LIS2DH12_AXIS_Y], obj->fir.raw[idx][LIS2DH12_AXIS_Z]);
		}
		
		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[LIS2DH12_AXIS_X], obj->fir.sum[LIS2DH12_AXIS_Y], obj->fir.sum[LIS2DH12_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[LIS2DH12_AXIS_X]/len, obj->fir.sum[LIS2DH12_AXIS_Y]/len, obj->fir.sum[LIS2DH12_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_LIS2DH12_LOWPASS
	struct i2c_client *client = lis2dh12_i2c_client;  
	struct lis2dh12_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if(1 != sscanf(buf, "%d", &firlen))
	{
		GSE_ERR("invallid format\n");
	}
	else if(firlen > C_MAX_FIR_LENGTH)
	{
		GSE_ERR("exceeds maximum filter length\n");
	}
	else
	{ 
		atomic_set(&obj->firlen, firlen);
		if(0 == firlen)
		{
			atomic_set(&obj->fir_en, 0);
		}
		else
		{
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct lis2dh12_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct lis2dh12_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}	
	else
	{
		GSE_ERR("invalid content: '%s'\n", buf);
	}
	
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;    
	struct lis2dh12_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}	
	
	if(obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
	            obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);   
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;    
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,             S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensordata,           S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,          store_cali_value);
static DRIVER_ATTR(power,                S_IRUGO, show_power_status,          NULL);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, show_firlen_value,        store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *lis2dh12_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_power,         /*show power reg*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,        
};
/*----------------------------------------------------------------------------*/
static int lis2dh12_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(lis2dh12_attr_list)/sizeof(lis2dh12_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if((err = driver_create_file(driver, lis2dh12_attr_list[idx])))
		{            
			GSE_ERR("driver_create_file (%s) = %d\n", lis2dh12_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int lis2dh12_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(lis2dh12_attr_list)/sizeof(lis2dh12_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, lis2dh12_attr_list[idx]);
	}
	

	return err;
}
#if 0
/*----------------------------------------------------------------------------*/
int lis2dh12_operate(void* self, uint32_t command, void* buff_in, int size_in,
		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value, sample_delay;	
	struct lis2dh12_i2c_data *priv = (struct lis2dh12_i2c_data*)self;
	hwm_sensor_data* gsensor_data;
	char buff[LIS2DH12_BUFSIZE];
	
	//GSE_FUN(f);
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				value = *(int *)buff_in;
				if(value <= 5)
				{
					sample_delay = LIS2DH12_BW_200HZ;
				}
				else if(value <= 10)
				{
					sample_delay = LIS2DH12_BW_100HZ;
				}
				else
				{
					sample_delay = LIS2DH12_BW_50HZ;
				}
				mutex_lock(&lis2dh12_op_mutex);
				err = LIS2DH12_SetBWRate(priv->client, sample_delay);
				if(err != LIS2DH12_SUCCESS ) //0x2C->BW=100Hz
				{
					GSE_ERR("Set delay parameter error!\n");
				}
				mutex_unlock(&lis2dh12_op_mutex);
				if(value >= 50)
				{
					atomic_set(&priv->filter, 0);
				}
				else
				{					
					priv->fir.num = 0;
					priv->fir.idx = 0;
					priv->fir.sum[LIS2DH12_AXIS_X] = 0;
					priv->fir.sum[LIS2DH12_AXIS_Y] = 0;
					priv->fir.sum[LIS2DH12_AXIS_Z] = 0;
					atomic_set(&priv->filter, 1);
				}
			}
			break;

		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				GSE_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{
			    
				value = *(int *)buff_in;
				mutex_lock(&lis2dh12_op_mutex);
				GSE_LOG("Gsensor device enable function enable = %d, sensor_power = %d!\n",value,sensor_power);
				if(((value == 0) && (sensor_power == false)) ||((value == 1) && (sensor_power == true)))
				{
					enable_status = sensor_power;
					GSE_LOG("Gsensor device have updated!\n");
				}
				else
				{
					enable_status = !sensor_power;
					err = LIS2DH12_SetPowerMode( priv->client, !sensor_power);
					GSE_LOG("Gsensor not in suspend lis2dh12_SetPowerMode!, enable_status = %d\n",enable_status);
				}
				mutex_unlock(&lis2dh12_op_mutex);			
			}
			break;

		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				GSE_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				mutex_lock(&lis2dh12_op_mutex);
				gsensor_data = (hwm_sensor_data *)buff_out;
				LIS2DH12_ReadSensorData(priv->client, buff, LIS2DH12_BUFSIZE);
				sscanf(buff, "%x %x %x", &gsensor_data->values[0], 
					&gsensor_data->values[1], &gsensor_data->values[2]);				
				gsensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;				
				gsensor_data->value_divide = 1000;
				mutex_unlock(&lis2dh12_op_mutex);
			}
			break;
		default:
			GSE_ERR("gsensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	
	return err;
}
#endif
/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int lis2dh12_open(struct inode *inode, struct file *file)
{
	file->private_data = lis2dh12_i2c_client;

	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int lis2dh12_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
//acc calibrate proc, add by zte szy 20161128  --start--
static bool is_accel_calibration_valid(int cali_value[LIS2DH12_AXES_NUM])
{
	// cali_value's unit is mg.
	if ((abs(cali_value[LIS2DH12_AXIS_X]) > LIS2DH12_CALI_TOLERANCE) 
		|| (abs(cali_value[LIS2DH12_AXIS_Y]) > LIS2DH12_CALI_TOLERANCE)
		|| (abs(cali_value[LIS2DH12_AXIS_Z]) > 2 * LIS2DH12_CALI_TOLERANCE))
	{
		return false;
	}
	return true;
}

static ssize_t acc_calibrate_read_proc(struct file *file,	char __user *buffer, size_t count, loff_t *offset)
{
	int cnt;	
	s16 rawacc[LIS2DH12_AXES_NUM] = {0};
	s32 raw_sum[LIS2DH12_AXES_NUM] = {0};
	int cali_data[LIS2DH12_AXES_NUM] = {0};
	int i = 0;
	struct lis2dh12_i2c_data *obj = obj_i2c_data;
	int res = 0;

	GSE_FUN();	
	if(*offset != 0)
	{
		GSE_LOG("%s,offset!=0 -> return 0\n", __FUNCTION__);
		return 0;
	}

	for (i = 0; i < LIS2DH12_CALI_LEN; i++)
	{
		res = LIS2DH12_ReadData(lis2dh12_i2c_client, rawacc);
		if(res < 0)
		{        
			GSE_ERR("I2C error at %d: ret value=%d", i, res);
			return -3;
		}
		raw_sum[LIS2DH12_AXIS_X] += rawacc[LIS2DH12_AXIS_X];
		raw_sum[LIS2DH12_AXIS_Y] += rawacc[LIS2DH12_AXIS_Y];
		raw_sum[LIS2DH12_AXIS_Z] += rawacc[LIS2DH12_AXIS_Z];
		msleep(10);
	}
	// calc avarage
	GSE_LOG("acc_calibrate_read_proc sum,x:%d,y:%d,z:%d\n", raw_sum[LIS2DH12_AXIS_X], raw_sum[LIS2DH12_AXIS_Y], raw_sum[LIS2DH12_AXIS_Z]);
	rawacc[LIS2DH12_AXIS_X] = raw_sum[LIS2DH12_AXIS_X] / LIS2DH12_CALI_LEN;
	rawacc[LIS2DH12_AXIS_Y] = raw_sum[LIS2DH12_AXIS_Y] / LIS2DH12_CALI_LEN;
	rawacc[LIS2DH12_AXIS_Z] = raw_sum[LIS2DH12_AXIS_Z] / LIS2DH12_CALI_LEN;
	GSE_LOG("acc_calibrate_read_proc avg,x:%d,y:%d,z:%d\n",rawacc[LIS2DH12_AXIS_X],rawacc[LIS2DH12_AXIS_Y],rawacc[LIS2DH12_AXIS_Z]);
	
	rawacc[LIS2DH12_AXIS_X] = rawacc[LIS2DH12_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	rawacc[LIS2DH12_AXIS_Y] = rawacc[LIS2DH12_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
	rawacc[LIS2DH12_AXIS_Z] = rawacc[LIS2DH12_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;

	cali_data[obj->cvt.map[LIS2DH12_AXIS_X]] = 0 - obj->cvt.sign[LIS2DH12_AXIS_X] * rawacc[LIS2DH12_AXIS_X];
	cali_data[obj->cvt.map[LIS2DH12_AXIS_Y]] = 0 - obj->cvt.sign[LIS2DH12_AXIS_Y] * rawacc[LIS2DH12_AXIS_Y];
	cali_data[obj->cvt.map[LIS2DH12_AXIS_Z]] = GRAVITY_EARTH_1000 - obj->cvt.sign[LIS2DH12_AXIS_Z] * rawacc[LIS2DH12_AXIS_Z];

	//check........
	if (false == is_accel_calibration_valid(cali_data))
	{
		GSE_ERR("calibration value invalid[X:%d,Y:%d,Z:%d].", cali_data[LIS2DH12_AXIS_X], 
			cali_data[LIS2DH12_AXIS_Y], cali_data[LIS2DH12_AXIS_Z]);
		return -2;
	}
	
	LIS2DH12_ResetCalibration(lis2dh12_i2c_client);
	LIS2DH12_WriteCalibration(lis2dh12_i2c_client, cali_data);
	
	cnt = sprintf(buffer, "%d %d %d", cali_data[LIS2DH12_AXIS_X], cali_data[LIS2DH12_AXIS_Y], cali_data[LIS2DH12_AXIS_Z]);
	GSE_LOG("cnt:%d, buffer:%s",cnt, buffer);
	*offset += cnt;
	return cnt;
}

static ssize_t acc_calibrate_write_proc(struct file *file, const char __user *user_buf, size_t len, loff_t *offset)
{
    int cali_data[LIS2DH12_AXES_NUM] = {0};
    char buf[16]={0};

	size_t copyLen = 0;
	
	GSE_LOG("%s: write len = %d\n",__func__, (int)len);
	copyLen = len < 16 ? len : 16;
	if (copy_from_user(buf, user_buf, copyLen))
	{
		GSE_LOG("%s, copy_from_user error\n", __func__);
		return -EFAULT;
	}
		
	sscanf(buf, "%d %d %d", &cali_data[LIS2DH12_AXIS_X], &cali_data[LIS2DH12_AXIS_Y], &cali_data[LIS2DH12_AXIS_Z]);
	GSE_LOG("[x:%d,y:%d,z:%d], copyLen=%d\n", cali_data[LIS2DH12_AXIS_X],
		cali_data[LIS2DH12_AXIS_Y],cali_data[LIS2DH12_AXIS_Z], (int)copyLen);
	
	//check........
	if (false == is_accel_calibration_valid(cali_data))
	{
		GSE_ERR("calibration value invalid[X:%d,Y:%d,Z:%d].", cali_data[LIS2DH12_AXIS_X], 
			cali_data[LIS2DH12_AXIS_Y], cali_data[LIS2DH12_AXIS_Z]);
		return -EFAULT;
	}

	LIS2DH12_ResetCalibration(lis2dh12_i2c_client);
	LIS2DH12_WriteCalibration(lis2dh12_i2c_client, cali_data);
	return len;
}

static const struct file_operations acc_calibrate_proc_fops = {
	.owner		= THIS_MODULE,
	.read       = acc_calibrate_read_proc,
	.write      = acc_calibrate_write_proc,
};

static void create_acc_calibrate_proc_file(void)
{
	acc_calibrate_proc_file = proc_create("driver/acc_calibration", 0666, NULL, &acc_calibrate_proc_fops);
	GSE_FUN(f);

	if (NULL == acc_calibrate_proc_file)
	{
	    GSE_ERR("create acc_calibrate_proc_file fail!\n");
	}
}
//acc calibrate proc, add by zte szy 20161128  --e-n-d--

static long lis2dh12_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)

{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct lis2dh12_i2c_data *obj = (struct lis2dh12_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[LIS2DH12_BUFSIZE];
	void __user *data;
	struct SENSOR_DATA sensor_data;
	long err = 0;
	int cali[3];

	//GSE_FUN(f);
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case GSENSOR_IOCTL_INIT:
			LIS2DH12_Init(client, 0);			
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			LIS2DH12_ReadChipInfo(client, strbuf, LIS2DH12_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;
			}				 
			break;	  

		case GSENSOR_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			LIS2DH12_SetPowerMode(client,true);
			LIS2DH12_ReadSensorData(client, strbuf, LIS2DH12_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;

		case GSENSOR_IOCTL_READ_GAIN:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_IOCTL_READ_OFFSET:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			if(copy_to_user(data, &gsensor_offset, sizeof(struct GSENSOR_VECTOR3D)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

		case GSENSOR_IOCTL_READ_RAW_DATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			LIS2DH12_ReadRawData(client, strbuf);
			if(copy_to_user(data, &strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;	  
			}
			if(atomic_read(&obj->suspend))
			{
				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{
			#if 0
				cali[LIS2DH12_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[LIS2DH12_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[LIS2DH12_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;			  
				err = LIS2DH12_WriteCalibration(client, cali);	
			#endif
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			err = LIS2DH12_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if((err = LIS2DH12_ReadCalibration(client, cali)))
			{
				break;
			}
			
			sensor_data.x = cali[LIS2DH12_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.y = cali[LIS2DH12_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.z = cali[LIS2DH12_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
			break;
		

		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}

#ifdef CONFIG_COMPAT
static long lis2dh12_acc_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    long err = 0;

	void __user *arg32 = compat_ptr(arg);
	
	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;
	
    switch (cmd)
    {
        case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
		        return err;
		    }
			break;
			
        case COMPAT_GSENSOR_IOCTL_SET_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
		        return err;
		    }
			break;
			
        case COMPAT_GSENSOR_IOCTL_GET_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
		        return err;
		    }
			break;
			
        case COMPAT_GSENSOR_IOCTL_CLR_CALI:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		    err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);
		    if (err){
		        GSE_ERR("GSENSOR_IOCTL_CLR_CALI unlocked_ioctl failed.");
		        return err;
		    }
			break;

        default:
            GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
            err = -ENOIOCTLCMD;
        break;

    }

    return err;
}
#endif

/*----------------------------------------------------------------------------*/
static struct file_operations lis2dh12_fops = {
	.owner = THIS_MODULE,
	.open = lis2dh12_open,
	.release = lis2dh12_release,
	.unlocked_ioctl = lis2dh12_unlocked_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = lis2dh12_acc_compat_ioctl,
#endif
};
/*----------------------------------------------------------------------------*/
static struct miscdevice lis2dh12_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &lis2dh12_fops,
};
/*----------------------------------------------------------------------------*/
//#ifndef USE_EARLY_SUSPEND
/*----------------------------------------------------------------------------*/
static int lis2dh12_suspend(struct i2c_client *client, pm_message_t msg) 
{
	struct lis2dh12_i2c_data *obj = i2c_get_clientdata(client);    
	int err = 0;
	//u8 dat;
	GSE_FUN();    
	mutex_lock(&lis2dh12_op_mutex);
	if(msg.event == PM_EVENT_SUSPEND)
	{   
		if(obj == NULL)
		{	
			mutex_unlock(&lis2dh12_op_mutex);
			GSE_ERR("null pointer!!\n");
			return -EINVAL;
		}
		//read old data
		if((err = LIS2DH12_SetPowerMode(obj->client, false)))
			{
				GSE_ERR("write power control fail!!\n");
				mutex_unlock(&lis2dh12_op_mutex);
				return -1; 	   
			}
		
		atomic_set(&obj->suspend, 1);
		LIS2DH12_power(obj->hw, 0);
	}
	sensor_suspend = 1;
	mutex_unlock(&lis2dh12_op_mutex);
	return err;
}
/*----------------------------------------------------------------------------*/
static int lis2dh12_resume(struct i2c_client *client)
{
	struct lis2dh12_i2c_data *obj = i2c_get_clientdata(client);        
	int err;
	GSE_FUN();
	mutex_lock(&lis2dh12_op_mutex);
	if(obj == NULL)
	{
		mutex_unlock(&lis2dh12_op_mutex);
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	LIS2DH12_power(obj->hw, 1);
	err = LIS2DH12_Init(client, 0);
	if(err)
	{
		mutex_unlock(&lis2dh12_op_mutex);
		GSE_ERR("initialize client fail!!\n");
		return err;        
	}
	atomic_set(&obj->suspend, 0);
	sensor_suspend = 0;	
	mutex_unlock(&lis2dh12_op_mutex);
	return 0;
}
/*----------------------------------------------------------------------------*/
#if 0//#else /*CONFIG_HAS_EARLY_SUSPEND is defined*/
/*----------------------------------------------------------------------------*/

static void lis2dh12_early_suspend(struct early_suspend *h) 
{
	struct lis2dh12_i2c_data *obj = container_of(h, struct lis2dh12_i2c_data, early_drv);   
	u8 databuf[2]; 
	int err = 0;
	u8 addr = LIS2DH12_REG_CTL_REG1;

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->suspend, 1); 
	mutex_lock(&lis2dh12_op_mutex);
	GSE_FUN(); 
	if((err = LIS2DH12_SetPowerMode(obj->client, false)))
	{
		GSE_ERR("write power control fail!!\n");
		mutex_unlock(&lis2dh12_op_mutex);
		return;        
	}
	sensor_suspend = 1;
	mutex_unlock(&lis2dh12_op_mutex);
	LIS2DH12_power(obj->hw, 0);
}
/*----------------------------------------------------------------------------*/
static void lis2dh12_late_resume(struct early_suspend *h)
{
	struct lis2dh12_i2c_data *obj = container_of(h, struct lis2dh12_i2c_data, early_drv);         
	int err;

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}

	LIS2DH12_power(obj->hw, 1);
	mutex_lock(&lis2dh12_op_mutex);
	GSE_FUN();
	if((err = LIS2DH12_Init(obj->client, 0)))
	{
		GSE_ERR("initialize client fail!!\n");
		mutex_unlock(&lis2dh12_op_mutex);
		return;        
	}
	sensor_suspend = 0;
	mutex_unlock(&lis2dh12_op_mutex);
	atomic_set(&obj->suspend, 0);    
}
/*----------------------------------------------------------------------------*/
#endif /*CONFIG_HAS_EARLYSUSPEND*/
/*----------------------------------------------------------------------------*/
static int lis2dh12_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
	strcpy(info->type, LIS2DH12_DEV_NAME);
	return 0;
}

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int lis2dh12_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int lis2dh12_enable_nodata(int en)
{
	int res =0;
	bool power = false;
	
	if(1==en)
	{
		power = true;
	}
	if(0==en)
	{
		power = false;
	}
	res = LIS2DH12_SetPowerMode(obj_i2c_data->client, power);
	if(res != LIS2DH12_SUCCESS)
	{
		GSE_ERR("LIS2DH12_SetPowerMode fail!\n");
		return -1;
	}
	GSE_LOG("lis2dh12_enable_nodata OK!\n");
	return 0;
}

static int lis2dh12_set_delay(u64 ns)
{
    int value =0;
	int sample_delay=0;
	int err;
	value = (int)ns/1000/1000;
	if(value <= 5)
	{
		sample_delay = LIS2DH12_BW_200HZ;
	}
	else if(value <= 10)
	{
		sample_delay = LIS2DH12_BW_100HZ;
	}
	else
	{
		sample_delay = LIS2DH12_BW_50HZ;
	}
	mutex_lock(&lis2dh12_op_mutex);
	err = LIS2DH12_SetBWRate(obj_i2c_data->client, sample_delay);
	if(err != LIS2DH12_SUCCESS ) //0x2C->BW=100Hz
	{
		GSE_ERR("Set delay parameter error!\n");
	}
	mutex_unlock(&lis2dh12_op_mutex);
	if(value >= 50)
	{
		atomic_set(&obj_i2c_data->filter, 0);
	}
	else
	{					
		obj_i2c_data->fir.num = 0;
		obj_i2c_data->fir.idx = 0;
		obj_i2c_data->fir.sum[LIS2DH12_AXIS_X] = 0;
		obj_i2c_data->fir.sum[LIS2DH12_AXIS_Y] = 0;
		obj_i2c_data->fir.sum[LIS2DH12_AXIS_Z] = 0;
		atomic_set(&obj_i2c_data->filter, 1);
	}
	
	GSE_LOG("lis2dh12_set_delay (%d)\n",value);
	return 0;
}

static int lis2dh12_get_data(int* x ,int* y,int* z, int* status)
{
	char buff[LIS2DH12_BUFSIZE];
	LIS2DH12_ReadSensorData(obj_i2c_data->client, buff, LIS2DH12_BUFSIZE);
	
	sscanf(buff, "%x %x %x", x, y, z);		
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}


/*----------------------------------------------------------------------------*/
static int lis2dh12_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct lis2dh12_i2c_data *obj;
	struct acc_control_path ctl={0};
	struct acc_data_path data={0};
	//struct acc_drv_obj sobj;
	int err = 0;
	int retry = 0;
	GSE_FUN();

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(obj, 0, sizeof(struct lis2dh12_i2c_data));

	obj->hw = get_cust_acc();
	
	if((err = hwmsen_get_convert(obj->hw->direction, &obj->cvt)))
	{
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);
	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	
#ifdef CONFIG_LIS2DH12_LOWPASS
	if(obj->hw->firlen > C_MAX_FIR_LENGTH)
	{
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	}	
	else
	{
		atomic_set(&obj->firlen, obj->hw->firlen);
	}
	
	if(atomic_read(&obj->firlen) > 0)
	{
		atomic_set(&obj->fir_en, 1);
	}
	
#endif

	lis2dh12_i2c_client = new_client;	

	for(retry = 0; retry < 3; retry++){
	if((err = LIS2DH12_Init(new_client, 1)))
	{
			GSE_ERR("lis2dh12_device init cilent fail time: %d\n", retry);
			continue;
		}
	}
	if(err != 0)
		goto exit_init_failed;
	

	if((err = misc_register(&lis2dh12_device)))
	{
		GSE_ERR("lis2dh12_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if((err = lis2dh12_create_attr(&(lis2dh12_init_info.platform_diver_addr->driver))))
	{
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	ctl.open_report_data= lis2dh12_open_report_data;
	ctl.enable_nodata = lis2dh12_enable_nodata;
	ctl.set_delay  = lis2dh12_set_delay;
	ctl.is_report_input_direct = false;
	
	err = acc_register_control_path(&ctl);
	if(err)
	{
	 	GSE_ERR("register acc control path err\n");
		goto exit_kfree;
	}

	data.get_data = lis2dh12_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if(err)
	{
	 	GSE_ERR("register acc data path err\n");
		goto exit_kfree;
	}
#if 0
#ifdef USE_EARLY_SUSPEND
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 2,
	obj->early_drv.suspend  = lis2dh12_early_suspend,
	obj->early_drv.resume   = lis2dh12_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif 
#endif

	create_acc_calibrate_proc_file();

	GSE_LOG("%s: OK\n", __func__);
	lis2dh12_init_flag = 0;    
	return 0;

	exit_create_attr_failed:
	misc_deregister(&lis2dh12_device);
	exit_misc_device_register_failed:
	exit_init_failed:
	//i2c_detach_client(new_client);
	exit_kfree:
	kfree(obj);
	exit:
	GSE_ERR("%s: err = %d\n", __func__, err);
	lis2dh12_init_flag = -1;        
	return err;
}

/*----------------------------------------------------------------------------*/
static int lis2dh12_i2c_remove(struct i2c_client *client)
{
	int err = 0;	
	
	if((err = lis2dh12_delete_attr(&(lis2dh12_init_info.platform_diver_addr->driver))))
	{
		GSE_ERR("lis2dh12_delete_attr fail: %d\n", err);
	}
	
	if((err = misc_deregister(&lis2dh12_device)))
	{
		GSE_ERR("misc_deregister fail: %d\n", err);
	}

	if((err = hwmsen_detach(ID_ACCELEROMETER)))
	    

	lis2dh12_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
static int  lis2dh12_remove(void)
{
    //struct acc_hw *hw = get_cust_acc();

    GSE_FUN();    
    LIS2DH12_power(hw, 0);    
    i2c_del_driver(&lis2dh12_i2c_driver);
    return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/

static int  lis2dh12_local_init(void)
{
   //struct acc_hw *hw = lis2dh12_get_cust_acc_hw();
	GSE_FUN();

	LIS2DH12_power(hw, 1);
	if(i2c_add_driver(&lis2dh12_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		return -1;
	}
	if(-1 == lis2dh12_init_flag)
	{
	   return -1;
	}
	
	return 0;
}

/*----------------------------------------------------------------------------*/
static int __init lis2dh12_init(void)
{
	const char *name = "mediatek,lis2dh12";

    	GSE_FUN();
	hw = get_accel_dts_func(name, hw);
	if (!hw)
		GSE_ERR("get dts info fail\n");

	//GSE_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num); 
	//i2c_register_board_info(hw->i2c_num, &i2c_LIS2DH12, 1);
	acc_driver_add(&lis2dh12_init_info);
	return 0;    
}
/*----------------------------------------------------------------------------*/
static void __exit lis2dh12_exit(void)
{
	GSE_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(lis2dh12_init);
module_exit(lis2dh12_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LIS2DH12 I2C driver");
MODULE_AUTHOR("Chunlei.Wang@mediatek.com");
