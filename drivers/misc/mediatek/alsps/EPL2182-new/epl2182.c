//lichengmin begin
/* drivers/hwmon/mt6516/amit/epl2182.c - EPL2182 ALS/PS driver
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

#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>

#include <hwmsensor.h>
#include <hwmsen_dev.h>
#include <asm/io.h>

#include "epl2182.h"

#include <linux/wakelock.h>
#include <linux/sched.h>

#include <alsps.h>
//#include <epl_cust_alsps.h>

#include <linux/proc_fs.h>    /* -----20150820 zhangxin add proc file interface-----*/
/******************************************************************************
 * extern functions
*******************************************************************************/
void epl2182_restart_polling(void);

/******************************************************************************
 * configuration
*******************************************************************************/

/* TODO: change delay time*/
#define PS_DELAY 			55
#define ALS_DELAY 			55
/* TODO: parameters for lux equation y = ax + b*/
#define LUX_PER_COUNT		400              // 1100 = 1.1 * 1000

#define PS_DRIVE                     EPL_DRIVE_60MA
#define TXBYTES 				2
#define RXBYTES 				2
#define PACKAGE_SIZE 			2
#define I2C_RETRY_COUNT 		3
#define IPI_WAIT_RSP_TIMEOUT    (HZ/10)     //100ms

static DEFINE_MUTEX(epl2182_mutex);

typedef struct _epl_raw_data
{
	u8 raw_bytes[PACKAGE_SIZE];
	int ps_raw;
	u16 ps_state;
	
	u16 ps_int_state;
	u16 als_ch0_raw;
	u16 als_ch1_raw;
	u16 als_lux;
	bool ps_suspend_flag;
} epl_raw_data;

/*----------------------------------------------------------------------------*/
#define EPL2182_DEV_NAME     "EPL2182"

#define APS_TAG                  "[ALSPS-EPL2182] "
#define APS_FUN(f)               printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)    printk(KERN_ERR  APS_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define APS_LOG(fmt, args...)    printk(KERN_DEBUG APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)    printk(KERN_DEBUG fmt, ##args)
#define FTM_CUST_ALSPS "/data/epl2182"

//#define POWER_NONE_MACRO MT65XX_POWER_NONE

static struct i2c_client *epl2182_i2c_client = NULL;
struct alsps_hw		alsps_cust;
static struct alsps_hw	*hw = &alsps_cust;

static DEFINE_MUTEX(sensor_mutex);

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id epl2182_i2c_id[] = {{"EPL2182",0},{}};
//static struct i2c_board_info __initdata i2c_EPL2182= { I2C_BOARD_INFO("EPL2182", (0X92>>1))};

/*----------------------------------------------------------------------------*/
static int epl2182_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int epl2182_i2c_remove(struct i2c_client *client);
static int epl2182_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);

static int alsps_local_init(void);
static int alsps_remove(void);
/*----------------------------------------------------------------------------*/
static int epl2182_i2c_suspend(struct i2c_client *client, pm_message_t msg);
static int epl2182_i2c_resume(struct i2c_client *client);
static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd);
static int set_psensor_threshold(struct i2c_client *client);

static struct epl2182_priv *g_epl2182_ptr = NULL;
static int isInterrupt = false;
//static int als_gainrange;
//static unsigned int alsps_irq;

static void polling_do_work(struct work_struct *work);
static DECLARE_DELAYED_WORK(polling_work, polling_do_work);
static long long int_top_time = 0;
static int int_flag = 0;
static int als_value_backup = -2;  //zhangxin 20151209  -2:no data, -1:first data;

/*---  zhangxin 20150901  for calibration ---------------*/
#define EPL2182_CROSSTALK_MAX          3000
#define EPL2182_CROSSTALK_DEFAULT	  600
#define EPL2182_CROSSTALK_UPLIMIT	  2000
#define EPL2182_DISABLE_PS		0
#define EPL2182_ENABLE_PS		1
#define EPL2182_DISFLAG_PS		0
#define EPL2182_ENBFLAG_PS		1

static int epl2182_ps_crosstalk = 0;
static int epl2182_ps_crosstalk_incall = 0;
static atomic_t epl2182_ps_calibrating = ATOMIC_INIT(0);

/*----------------------------------------------------------------------------*/
typedef enum
{
    CMC_BIT_ALS    = 1,
    CMC_BIT_PS     = 2,
} CMC_BIT;

/*----------------------------------------------------------------------------*/
struct epl2182_i2c_addr      /*define a series of i2c slave address*/
{
    u8  write_addr;
    u8  ps_thd;     /*PS INT threshold*/
};

/*----------------------------------------------------------------------------*/
struct epl2182_priv
{
    struct alsps_hw  *hw;
    struct i2c_client *client;
    struct work_struct  eint_work;
	struct work_struct data_work;
	 struct workqueue_struct *epl_wq;

    /*i2c address group*/
    struct epl2182_i2c_addr  addr;

    int enable_pflag;
    int enable_lflag;
    struct device_node *irq_node;

    /*misc*/
    atomic_t    trace;
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

    /*data*/
    u16         als;
    u16         ps;
    u16		lux_per_count;
    bool   		als_enable;    /*record current als status*/
    bool    	ps_enable;     /*record current ps status*/
    ulong       enable;         /*record HAL enalbe status*/
    ulong       pending_intr;   /*pending interrupt*/
    //ulong        first_read;   // record first read ps and als

    /* control flag from HAL */
    unsigned int enable_ps_sensor;
    unsigned int ps_sensor_flag;
    int		irq;

    /*data*/
    u16         als_level_num;
    u16         als_value_num;
    u32         als_level[C_CUST_ALS_LEVEL];
    u32         als_value[C_CUST_ALS_LEVEL];
	int			ps_cali;

	unsigned int	ps_thd_val_high;	 /*the cmd value can't be read, stored in ram*/
	unsigned int	ps_thd_val_low; 	/*the cmd value can't be read, stored in ram*/
};

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,alsps"},
	{},
};
#endif

/*----------------------------------------------------------------------------*/
static struct i2c_driver epl2182_i2c_driver =
{
    .probe      = epl2182_i2c_probe,
    .remove     = epl2182_i2c_remove,
    .detect     = epl2182_i2c_detect,
    .suspend    = epl2182_i2c_suspend,
    .resume     = epl2182_i2c_resume,
    .id_table   = epl2182_i2c_id,
    .driver = {
        .name           = EPL2182_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif
    },
};


static struct epl2182_priv *epl2182_obj = NULL;
static epl_raw_data	gRawData;

static int alsps_init_flag =-1; // 0<==>OK -1 <==> fail

static struct alsps_init_info epl2182_init_info = {
		.name = EPL2182_DEV_NAME,
		.init = alsps_local_init,
		.uninit = alsps_remove,

};

//-----add wake_lock to prevent goto sleep before ps be seted-----dinggaoshan-20151014--
static struct wake_lock epl2182_ps_wake_lock;

static DECLARE_WAIT_QUEUE_HEAD(wait_rsp_wq);

//static atomic_t wait_rsp_flag = ATOMIC_INIT(0);

//static struct wake_lock als_lock; /* Bob.chen add for if ps run, the system forbid to goto sleep mode. */

/*
//====================I2C write operation===============//
//regaddr: ELAN epl2182 Register Address.
//bytecount: How many bytes to be written to epl2182 register via i2c bus.
//txbyte: I2C bus transmit byte(s). Single byte(0X01) transmit only slave address.
//data: setting value.
//
// Example: If you want to write single byte to 0x1D register address, show below
//	      elan_epl2182_I2C_Write(client,0x1D,0x01,0X02,0xff);
//
*/
static int elan_epl2182_I2C_Write(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount, uint8_t txbyte, uint8_t data)
{
    uint8_t buffer[2];
    int ret = 0;
    int retry;

    //APS_ERR("[ELAN epl2182] %s\n", __func__);
	mutex_lock(&epl2182_mutex);
    buffer[0] = (regaddr<<3) | bytecount ;
    buffer[1] = data;


    //APS_ERR("---elan_epl2182_I2C_Write register (0x%x) buffer data (%x) (%x)---\n",regaddr,buffer[0],buffer[1]);

    for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
        ret = i2c_master_send(client, buffer, txbyte);
        if (ret >= 0)
        {
            break;
        }

        APS_ERR("epl2182 i2c write error,TXBYTES %d\r\n",ret);
        mdelay(10);
    }

    if(retry>=I2C_RETRY_COUNT)
    {
    	mutex_unlock(&epl2182_mutex);
        APS_ERR(KERN_ERR "[ELAN epl2182 error] %s i2c write retry over %d\n",__func__, I2C_RETRY_COUNT);
        return -EINVAL;
    }
	mutex_unlock(&epl2182_mutex);
    return ret;
}




/*
//====================I2C read operation===============//
*/
static int elan_epl2182_I2C_Read(struct i2c_client *client, uint8_t regaddr, uint8_t bytecount, uint8_t rxbyte, uint8_t *data)
{
    uint8_t buffer[RXBYTES];
    int ret = 0, i =0;
    int retry;

    //APS_DBG("[ELAN epl2182] %s\n", __func__);
	mutex_lock(&epl2182_mutex);
	buffer[0] = (regaddr<<3) | bytecount ;

    for(retry = 0; retry < I2C_RETRY_COUNT; retry++)
    {
    	ret = hwmsen_read_block(client, buffer[0], buffer, rxbyte);
        if (ret >= 0)
            break;

        APS_ERR("epl2182 i2c read error,RXBYTES %d\r\n",ret);
        mdelay(10);
    }

    if(retry>=I2C_RETRY_COUNT)
    {
        APS_ERR(KERN_ERR "[ELAN epl2182 error] %s i2c read retry over %d\n",__func__, I2C_RETRY_COUNT);
		mutex_unlock(&epl2182_mutex);
        return -EINVAL;
    }

    for(i=0; i<PACKAGE_SIZE; i++)
        *data++ = buffer[i];
	mutex_unlock(&epl2182_mutex);
    //APS_DBG("----elan_epl2182_I2C_Read Receive data from (0x%x):byte1 (%x) byte2 (%x)-----\n",regaddr, buffer[0], buffer[1]);

    return ret;
}

static int epl2182_report_interrupt_data(int value) 
{
       APS_LOG("epl2182_report_interrupt_data:%d(0->1, 1->5)\n", value);
	if (value==0)
		return ps_report_interrupt_data(1);
	else if (value==1)
		return ps_report_interrupt_data(5);

      return -1;
}

static int elan_epl2182_psensor_enable(struct epl2182_priv *epl_data, int enable)
{
	//APS_FUN();
	int ret = 0;
	int err = 0;
	uint8_t regdata;
	uint8_t read_data[2];
	int ps_state;
	struct i2c_client *client = epl_data->client;

    //APS_LOG("[ELAN epl2182] %s enable = %d\n", __func__, enable);

    epl_data->enable_pflag = enable;
    ret = elan_epl2182_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE | PS_DRIVE); //PS_DRIVE);

    if(enable)
    {
        epl_data->enable_lflag = false;
        regdata = EPL_SENSING_2_TIME | EPL_PS_MODE | EPL_L_PS_GAIN ;
		//regdata = EPL_SENSING_2_TIME | EPL_PS_MODE | EPL_L_GAIN ; 2014-9-11
        regdata = regdata | (isInterrupt ? EPL_C_SENSING_MODE : EPL_S_SENSING_MODE);
        ret = elan_epl2182_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

        regdata = EPL_PS_INTT_80| EPL_PST_1_TIME | EPL_12BIT_ADC;
        ret = elan_epl2182_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);

		//set_psensor_threshold(client);

		ret = elan_epl2182_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,EPL_C_RESET);
		ret = elan_epl2182_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);
		msleep(PS_DELAY);

		elan_epl2182_I2C_Read(client, REG_16, R_TWO_BYTE, 0x02, read_data);
		gRawData.ps_raw = (read_data[1]<<8) | read_data[0];

		elan_epl2182_I2C_Read(client, REG_13, R_SINGLE_BYTE, 0x01, read_data);
		ps_state = !((read_data[0] & 0x04) >> 2);
		int_flag = ps_state;
		APS_LOG("epl2182 ps state = %d, ps_raw=%d \n", ps_state, gRawData.ps_raw);

		if (isInterrupt)
		{
			if (gRawData.ps_state != ps_state)
			{
				gRawData.ps_state = ps_state;

				if ((err = epl2182_report_interrupt_data(ps_state)))
					APS_ERR("epl2182 call ps_report_interrupt_data fail = %d\n", err);
				//APS_LOG("epl2182 eint work soft gRawData.ps_state = %d\n", gRawData.ps_state);				
				//elan_epl2182_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02, EPL_INT_ACTIVE_LOW | PS_DRIVE);

			}
			
			//comment this "else", config interrupt Reg no metter what the ps_state is, 
			// to aviod [first go through "if", then als=0 and polling_do_work return directly, never interrupt].------dinggaoshan2015.10.27
			//else        
			          
			{
				elan_epl2182_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02, EPL_INT_ACTIVE_LOW | PS_DRIVE);
			}
		}

	}
	else
	{
		regdata = EPL_SENSING_2_TIME | EPL_PS_MODE | EPL_L_PS_GAIN;
		regdata = regdata | EPL_S_SENSING_MODE;
		ret = elan_epl2182_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0X02, regdata);
		ret = elan_epl2182_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02, EPL_INT_DISABLE | PS_DRIVE);//yucong add
	}

   if(ret<0)
    {
        APS_ERR("[ELAN epl2182 error]%s: ps enable %d fail\n",__func__,ret);
    }
    else
    {
        ret = 0;
    }

    return ret;
}


static int elan_epl2182_lsensor_enable(struct epl2182_priv *epl_data, int enable)
{
	//APS_FUN();
    int ret = 0;
    uint8_t regdata;

    struct i2c_client *client = epl_data->client;

    epl_data->enable_lflag = enable;

    if(enable)
    {
    	epl_data->enable_pflag = false;
        regdata = EPL_INT_DISABLE;
        ret = elan_epl2182_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02, regdata);


        regdata = EPL_S_SENSING_MODE | EPL_SENSING_8_TIME | EPL_ALS_MODE | EPL_AUTO_ALS_GAIN;

        ret = elan_epl2182_I2C_Write(client,REG_0,W_SINGLE_BYTE,0X02,regdata);

        regdata = EPL_ALS_INTT_512 | EPL_PST_1_TIME | EPL_10BIT_ADC;
        ret = elan_epl2182_I2C_Write(client,REG_1,W_SINGLE_BYTE,0X02,regdata);

        ret = elan_epl2182_I2C_Write(client,REG_10,W_SINGLE_BYTE,0X02,0x3e);
        ret = elan_epl2182_I2C_Write(client,REG_11,W_SINGLE_BYTE,0x02,0x3e);

        ret = elan_epl2182_I2C_Write(client,REG_7,W_SINGLE_BYTE,0X02,EPL_C_RESET);
        ret = elan_epl2182_I2C_Write(client,REG_7,W_SINGLE_BYTE,0x02,EPL_C_START_RUN);
		msleep(ALS_DELAY);
    }


    if(ret<0)
    {
        APS_ERR("[ELAN epl2182 error]%s: als_enable %d fail\n",__func__,ret);
    }
    else
    {
        ret = 0;
    }

    return ret;
}

//convert raw to lux
static int epl2182_get_als_value(struct epl2182_priv *obj, u16 als)
{
    //int idx;
    int invalid = 0;
    int lux = 0;
    
#if 0
    //if(als < 15)
    //{
        //APS_DBG("epl2182 ALS: %05d => 0\n", als);
   //     return 0;
   // }

    lux = als;//(als * obj->lux_per_count)/1000;

    for(idx = 0; idx < obj->als_level_num; idx++)
    {
        if(lux < obj->hw->als_level[idx])
        {
            break;
        }
    }

    if(idx >= obj->als_value_num)
    {
        APS_ERR("epl2182 exceed range\n");
        idx = obj->als_value_num - 1;
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
		#if defined(MTK_AAL_SUPPORT)
        int level_high = obj->hw->als_level[idx];
    	int level_low = (idx > 0) ? obj->hw->als_level[idx-1] : 0;
        int level_diff = level_high - level_low;
		int value_high = obj->hw->als_value[idx];
        int value_low = (idx > 0) ? obj->hw->als_value[idx-1] : 0;
        int value_diff = value_high - value_low;
        int value = 0;

        if ((level_low >= level_high) || (value_low >= value_high))
            value = value_low;
        else
            value = (level_diff * value_low + (lux - level_low) * value_diff + ((level_diff + 1) >> 1)) / level_diff;

		//APS_DBG("ALS: %d [%d, %d] => %d [%d, %d] \n", als, level_low, level_high, value, value_low, value_high);
		return value;
		#endif
        //APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);
        return obj->hw->als_value[idx];
    }
    else
    {
        APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);
        return -1;
    }
}
#endif

    if (4000 <= als)
    {
        lux = als*625/1000;// * obj->lux_per_count)/1000; (0.25/0.4=0.625)
    }
    else if (1000 <= als) // 420LUX settings
    {
        lux = als*480/1000;// * obj->lux_per_count)/1000; (0.192/0.4=0.48)
    }
    else // 100LUX settings
    {
        lux = als*447/1000;// * obj->lux_per_count)/1000; (0.1788/0.4=0.447)
        //APS_DBG("szy < 1000 lux: %05d, als=%05d\n", lux, als);
    }
	
	//zhangxin 20151209 send data to framework before sleep
    if (-2 == als_value_backup)  return -1;    
	
    if (-1 == als_value_backup)
    {
        //for(idx = 0; idx < obj->als_level_num; idx++)
       // {   
       //     if(lux < obj->hw->als_level[idx])
       //     {
       //         break;
       //     }
       // }
    }
    else
    {
	/*	for(idx = 0; idx < obj->als_level_num; idx++)
		{
            if(idx < 2)
			{
                if(lux < obj->hw->als_level[idx])
				    break;
			}
			else
			{  
				if((lux < (20 + obj->hw->als_level[idx - 1])) && (lux >= obj->hw->als_level[idx - 1]))
				{
				    APS_DBG("critical_als=%d,idx=%d,als_value_backup=%d\n", lux,idx,als_value_backup);
					return als_value_backup;
				}

				if(lux < obj->hw->als_level[idx])
				{
					break;
				}
			}
		}*/

        if ((lux < als_value_backup*12/10) && (lux > als_value_backup*8/10))  
            lux = als_value_backup;
    }

    /*if(idx >= obj->als_value_num)
    {
        APS_ERR("exceed range\n");
        idx = obj->als_value_num - 1;
    }*/

    if (!invalid)
    {
        //gRawData.als_lux = obj->hw->als_value[idx];
        APS_DBG("ALS: %05d => %05d\n", als, lux/*obj->hw->als_value[idx]*/);

    //zhangxin 20151209 send data to framework before sleep
	if (atomic_read(&obj->als_suspend) == 1)     
	{
		APS_DBG("epl2182: when suspend, no als data!\n");
		return -1;
	}

	wake_lock(&epl2182_ps_wake_lock);
	als_value_backup = lux;//obj->hw->als_value[idx];
	wake_unlock(&epl2182_ps_wake_lock);
    //zhangxin 20151209 send data to framework before sleep end
	
        return lux;//obj->hw->als_value[idx];
    }
    else
    {
        APS_ERR("ALS: %05d => %05d (-1)\n", als, lux /*obj->hw->als_value[idx]*/);
        return gRawData.als_lux;
    }
}

static int set_psensor_intr_threshold(uint16_t low_thd, uint16_t high_thd)
{
    int ret = 0;
    struct epl2182_priv *epld = epl2182_obj;
    struct i2c_client *client = epld->client;
    uint8_t high_msb ,high_lsb, low_msb, low_lsb;

    APS_LOG("[%s]: low_thd = %d, high_thd = %d \n",__func__, low_thd, high_thd);

    high_msb = (uint8_t) (high_thd >> 8);
    high_lsb = (uint8_t) (high_thd & 0x00ff);
    low_msb  = (uint8_t) (low_thd >> 8);
    low_lsb  = (uint8_t) (low_thd & 0x00ff);

    elan_epl2182_I2C_Write(client,REG_2,W_SINGLE_BYTE,0x02,high_lsb);
    elan_epl2182_I2C_Write(client,REG_3,W_SINGLE_BYTE,0x02,high_msb);
    elan_epl2182_I2C_Write(client,REG_4,W_SINGLE_BYTE,0x02,low_lsb);
    elan_epl2182_I2C_Write(client,REG_5,W_SINGLE_BYTE,0x02,low_msb);

    return ret;
}



/*----------------------------------------------------------------------------*/
static void epl2182_dumpReg(struct i2c_client *client)
{
    APS_LOG("chip id REG 0x00 value = %8x\n", i2c_smbus_read_byte_data(client, 0x00));
    APS_LOG("chip id REG 0x01 value = %8x\n", i2c_smbus_read_byte_data(client, 0x08));
    APS_LOG("chip id REG 0x02 value = %8x\n", i2c_smbus_read_byte_data(client, 0x10));
    APS_LOG("chip id REG 0x03 value = %8x\n", i2c_smbus_read_byte_data(client, 0x18));
    APS_LOG("chip id REG 0x04 value = %8x\n", i2c_smbus_read_byte_data(client, 0x20));
    APS_LOG("chip id REG 0x05 value = %8x\n", i2c_smbus_read_byte_data(client, 0x28));
    APS_LOG("chip id REG 0x06 value = %8x\n", i2c_smbus_read_byte_data(client, 0x30));
    APS_LOG("chip id REG 0x07 value = %8x\n", i2c_smbus_read_byte_data(client, 0x38));
    APS_LOG("chip id REG 0x09 value = %8x\n", i2c_smbus_read_byte_data(client, 0x48));
    APS_LOG("chip id REG 0x0D value = %8x\n", i2c_smbus_read_byte_data(client, 0x68));
    APS_LOG("chip id REG 0x0E value = %8x\n", i2c_smbus_read_byte_data(client, 0x70));
    APS_LOG("chip id REG 0x0F value = %8x\n", i2c_smbus_read_byte_data(client, 0x71));
    APS_LOG("chip id REG 0x10 value = %8x\n", i2c_smbus_read_byte_data(client, 0x80));
    APS_LOG("chip id REG 0x11 value = %8x\n", i2c_smbus_read_byte_data(client, 0x88));
    APS_LOG("chip id REG 0x13 value = %8x\n", i2c_smbus_read_byte_data(client, 0x98));

}


/*----------------------------------------------------------------------------*/
int hw8k_init_device(struct i2c_client *client)
{
    APS_LOG("hw8k_init_device.........\r\n");

    epl2182_i2c_client=client;

    APS_LOG("epl2182 I2C Addr==[0x%x],line=%d\n",epl2182_i2c_client->addr,__LINE__);

    return 0;
}

/*----------------------------------------------------------------------------*/
int epl2182_get_addr(struct alsps_hw *hw, struct epl2182_i2c_addr *addr)
{
    if(!hw || !addr)
    {
        return -EFAULT;
    }
    addr->write_addr= hw->i2c_addr[0];
    return 0;
}


/*----------------------------------------------------------------------------*/
static void epl2182_power(struct alsps_hw *hw, unsigned int on)
{
#if 0
    static unsigned int power_on = 0;

    APS_LOG("power %s\n", on ? "on" : "off");

    if(hw->power_id != POWER_NONE_MACRO)
    {
        if(power_on == on)
        {
            APS_LOG("ignore power control: %d\n", on);
        }
        else if(on)
        {
            if(!hwPowerOn(hw->power_id, hw->power_vol, "EPL2182"))
            {
                APS_ERR("power on fails!!\n");
            }
        }
        else
        {
            if(!hwPowerDown(hw->power_id, "EPL2182"))
            {
                APS_ERR("power off fail!!\n");
            }
        }
    }
    power_on = on;
#endif
}

/*----------------------------------------------------------------------------*/

int epl2182_read_als(struct i2c_client *client, u16 *data)
{
    struct epl2182_priv *obj = i2c_get_clientdata(client);
	uint8_t read_data[2];
    if(client == NULL)
    {
        APS_DBG("CLIENT CANN'T EQUL NULL\n");
        return -1;
    }

    elan_epl2182_I2C_Read(obj->client,REG_14,R_TWO_BYTE,0x02,read_data);
    gRawData.als_ch0_raw = (read_data[1]<<8) | read_data[0];

    elan_epl2182_I2C_Read(obj->client,REG_16,R_TWO_BYTE,0x02,read_data);
    gRawData.als_ch1_raw = (read_data[1]<<8) | read_data[0];

    *data =  gRawData.als_ch1_raw;

    APS_LOG("epl2182 read als raw data = %d, chan0=%d\n", gRawData.als_ch1_raw, gRawData.als_ch0_raw );
    return 0;
}


/*----------------------------------------------------------------------------*/
long epl2182_read_ps(struct i2c_client *client, u16 *data)
{
	struct epl2182_priv *obj = i2c_get_clientdata(client);
	uint8_t read_data[2];

	if(client == NULL)
	{
       	APS_DBG("CLIENT CANN'T EQUL NULL\n");
        	return -1;
    	}

	elan_epl2182_I2C_Read(obj->client, REG_16, R_TWO_BYTE, 0x02, read_data);
	gRawData.ps_raw = (read_data[1] << 8) | read_data[0];

	elan_epl2182_I2C_Read(obj->client, REG_13, R_SINGLE_BYTE, 0x01, read_data);
	gRawData.ps_state = !((read_data[0] & 0x04) >> 2);

	*data = gRawData.ps_raw;

	APS_LOG("epl2182 read ps raw data = %d\n", gRawData.ps_raw);
	APS_LOG("epl2182 read ps binary data = %d\n", gRawData.ps_state);

	return 0;
}

/*----------------------------------------------------------------------------*/
static irqreturn_t epl2182_eint_func(int irq, void *desc)
{
    struct epl2182_priv *obj = epl2182_obj;
    APS_ERR("epl2182_eint_func\n");

    int_top_time = sched_clock();

    if(!obj)
    {
        return IRQ_HANDLED;
    }

    if (EPL2182_ENBFLAG_PS == obj->ps_sensor_flag)
    {   
        obj->ps_sensor_flag = EPL2182_DISFLAG_PS;
        disable_irq_nosync(obj->irq); //mt_eint_mask(CUST_EINT_ALS_NUM);
        APS_LOG("epl2182_eint_func disable_irq_nosync flag=%d\n", obj->ps_sensor_flag);

        schedule_work(&obj->eint_work);
    }
    else 
    {
        APS_LOG("epl2182_eint_func disable_irq_nosync error flag=%d\n", obj->ps_sensor_flag);
    }

    return IRQ_HANDLED;
}
/*--------------------print Reg in mtklog-----------------------------------------*/
/*
static void epl2182_show_reg_inLog(void)
{
	struct i2c_client *client = epl2182_obj->client;
	
	if(!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return 0;
	}

	APS_LOG("chip id REG 0x00 value = %8x\n", i2c_smbus_read_byte_data(client, 0x00));
	APS_LOG("chip id REG 0x01 value = %8x\n", i2c_smbus_read_byte_data(client, 0x08));
	APS_LOG("chip id REG 0x02 value = %8x\n", i2c_smbus_read_byte_data(client, 0x10));
	APS_LOG("chip id REG 0x03 value = %8x\n", i2c_smbus_read_byte_data(client, 0x18));
	APS_LOG("chip id REG 0x04 value = %8x\n", i2c_smbus_read_byte_data(client, 0x20));
	APS_LOG("chip id REG 0x05 value = %8x\n", i2c_smbus_read_byte_data(client, 0x28));
	APS_LOG("chip id REG 0x06 value = %8x\n", i2c_smbus_read_byte_data(client, 0x30));
	APS_LOG("chip id REG 0x07 value = %8x\n", i2c_smbus_read_byte_data(client, 0x38));
	APS_LOG("chip id REG 0x09 value = %8x\n", i2c_smbus_read_byte_data(client, 0x48));
	APS_LOG("chip id REG 0x0D value = %8x\n", i2c_smbus_read_byte_data(client, 0x68));
	APS_LOG("chip id REG 0x0E value = %8x\n", i2c_smbus_read_byte_data(client, 0x70));
	APS_LOG("chip id REG 0x0F value = %8x\n", i2c_smbus_read_byte_data(client, 0x71));
	APS_LOG("chip id REG 0x10 value = %8x\n", i2c_smbus_read_byte_data(client, 0x80));
	APS_LOG("chip id REG 0x11 value = %8x\n", i2c_smbus_read_byte_data(client, 0x88));
	APS_LOG("chip id REG 0x13 value = %8x\n", i2c_smbus_read_byte_data(client, 0x98));

}
*/

/*----------------------------------------------------------------------------*/
static void epl2182_eint_work(struct work_struct *work)
{
	struct epl2182_priv *epld = (struct epl2182_priv *)container_of(work, struct epl2182_priv, eint_work); //epl2182_obj;
	int err;
	uint8_t read_data[2];
	int flag;

//	epl2182_show_reg_inLog();
	mutex_lock(&sensor_mutex);
	if(epld->enable_pflag==0)
        goto exit;

	elan_epl2182_I2C_Read(epld->client, REG_16, R_TWO_BYTE, 0x02, read_data);
	gRawData.ps_raw = (read_data[1]<<8) | read_data[0];

	elan_epl2182_I2C_Read(epld->client, REG_13, R_SINGLE_BYTE, 0x01, read_data);
	flag = !((read_data[0] & 0x04) >> 2);

	APS_LOG("epl2182_eint_work:gRawData.ps_raw=%d, ps_state = %d, flag(0:near, 1:far) = %d\n", gRawData.ps_raw, gRawData.ps_state, flag);

	gRawData.ps_state = flag;//update ps state
	APS_LOG("interrupt hardward = %d\n", gRawData.ps_state);

        //let up layer to know
        if((err = epl2182_report_interrupt_data(flag)))
        {
            APS_ERR("epl2182 call ps_report_interrupt_data fail = %d\n", err);
        }

exit:
	elan_epl2182_I2C_Write(epld->client, REG_9, W_SINGLE_BYTE, 0x02, EPL_INT_ACTIVE_LOW);
	elan_epl2182_I2C_Write(epld->client, REG_7, W_SINGLE_BYTE, 0x02, EPL_DATA_UNLOCK);
	mutex_unlock(&sensor_mutex);

	//mt_eint_unmask(CUST_EINT_ALS_NUM);
	if (EPL2182_DISFLAG_PS == epld->ps_sensor_flag)
	{         
		epld->ps_sensor_flag = EPL2182_ENBFLAG_PS;
		enable_irq(epl2182_obj->irq);
		APS_LOG("epl2182_eint_work enable_irq flag=%d\n", epld->ps_sensor_flag);
	}
	else 
	{
		APS_LOG("epl2182_eint_work enable_irq error flag=%d\n", epld->ps_sensor_flag);
	}
}
#if 0
/*----------------------------------------------------------------------------*/
int epl2182_setup_eint(struct i2c_client *client)
{
    struct epl2182_priv *obj = i2c_get_clientdata(client);

    APS_LOG("epl2182_setup_eint\n");

    g_epl2182_ptr = obj;

    //isProbeOk = false;
    if (request_irq(alsps_irq, epl2182_eint_func, IRQF_TRIGGER_LOW, "ALS-eint", NULL)) {
	   APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
	   return -EINVAL;
    }

    //disable_irq_nosync(alsps_irq);
    irq_set_irq_wake(client->irq, 1);

    return 0;
}
#endif
int epl2182_setup_eint(struct i2c_client *client)
{
	struct epl2182_priv *obj = i2c_get_clientdata(client);
	int ret;
	u32 ints[2] = { 0, 0 };
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_cfg;
	struct platform_device *alsps_pdev = get_alsps_platformdev();


	APS_LOG("epl2182_setup_eint\n");


	g_epl2182_ptr = obj;

	/*configure to GPIO function, external interrupt */

/* gpio setting */
	pinctrl = devm_pinctrl_get(&alsps_pdev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		APS_ERR("Cannot find alsps pinctrl!\n");
	}
	pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		APS_ERR("Cannot find alsps pinctrl default!\n");
	}

	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");

	}
	pinctrl_select_state(pinctrl, pins_cfg);

	if (epl2182_obj->irq_node) {
		of_property_read_u32_array(epl2182_obj->irq_node, "debounce", ints,
					   ARRAY_SIZE(ints));
		gpio_request(ints[0], "p-sensor");
		gpio_set_debounce(ints[0], ints[1]);
		APS_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		epl2182_obj->irq = irq_of_parse_and_map(epl2182_obj->irq_node, 0);
		APS_LOG("epl2182_obj->irq = %d\n", epl2182_obj->irq);
		if (!epl2182_obj->irq) {
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
		if (request_irq(epl2182_obj->irq, epl2182_eint_func, IRQF_TRIGGER_NONE, "ALS-eint", NULL)) {
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
        
		epl2182_obj->ps_sensor_flag= EPL2182_DISFLAG_PS;	// default to 0
		disable_irq_nosync(epl2182_obj->irq);
		APS_LOG("epl2182_setup_eint disable_irq_nosync flag=%d\n", epl2182_obj->ps_sensor_flag);
		//enable_irq(epl2182_obj->irq);
	} else {
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}

	//enable_irq(epl2182_obj->irq);

	return 0;
}

/*----------------------------------------------------------------------------*/
static int epl2182_init_client(struct i2c_client *client)
{
    struct epl2182_priv *obj = i2c_get_clientdata(client);
    int err=0;

    APS_LOG("epl2182_init_client, I2C Addr==[0x%x],line=%d, isInterrupt=%d\n", epl2182_i2c_client->addr, __LINE__, isInterrupt);

    if(isInterrupt)
    {
        //mt_eint_mask(CUST_EINT_ALS_NUM);
        if (EPL2182_ENBFLAG_PS == obj->ps_sensor_flag)
        {   
             obj->ps_sensor_flag = EPL2182_DISFLAG_PS;
             disable_irq_nosync(epl2182_obj->irq);
             APS_LOG("epl2182_init_client disable_irq_nosync flag=%d\n", obj->ps_sensor_flag);
        }
        else 
        {
             APS_LOG("epl2182_init_client disable_irq_nosync error flag=%d\n", obj->ps_sensor_flag);
        }

        err = epl2182_setup_eint(client);
        if(err)
        {
            APS_ERR("setup eint: %d\n", err);
            return err;
        }
        APS_LOG("epl2182 interrupt setup\n");
    }

    if((err = hw8k_init_device(client)) != 0)
    {
        APS_ERR("init dev: %d\n", err);
        return err;
    }

    return err;
}

static void epl2182_check_ps_data(struct work_struct *work)
{
	int flag;
	uint8_t read_data[2];
	int err = 0;
	struct epl2182_priv *epld = epl2182_obj;

	elan_epl2182_I2C_Read(epld->client,REG_13,R_SINGLE_BYTE,0x01,read_data);
	flag = !((read_data[0]&0x04)>>2);
	if(flag != int_flag){
		APS_ERR("epl2182 call hwmsen_get_interrupt_data fail = %d\n", err);
		goto exit;
	}
	else{
		//let up layer to know
		APS_LOG("epl2182 int_flag state = %d, %s\n", int_flag, __func__);
		if (0 != (err = epl2182_report_interrupt_data(int_flag)))
		{
			APS_ERR("epl2182 call ps_report_interrupt_data fail = %d\n", err);
			goto exit;
		}
	}
exit:
	return;
}

static ssize_t epl2182_show_als(struct device_driver *ddri, char *buf)
{
	if (!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return 0;
	}
    
	return snprintf(buf, PAGE_SIZE, "%d\n", gRawData.als_ch1_raw); 
}

/*----------------------------------------------------------------------------*/
static ssize_t epl2182_show_reg(struct device_driver *ddri, char *buf)
{
    struct i2c_client *client = epl2182_obj->client;
    ssize_t len = 0;

    if(!epl2182_obj)
    {
        APS_ERR("epl2182_obj is null!!\n");
        return 0;
    }

    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x00 value = %8x\n", i2c_smbus_read_byte_data(client, 0x00));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x01 value = %8x\n", i2c_smbus_read_byte_data(client, 0x08));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x02 value = %8x\n", i2c_smbus_read_byte_data(client, 0x10));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x03 value = %8x\n", i2c_smbus_read_byte_data(client, 0x18));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x04 value = %8x\n", i2c_smbus_read_byte_data(client, 0x20));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x05 value = %8x\n", i2c_smbus_read_byte_data(client, 0x28));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x06 value = %8x\n", i2c_smbus_read_byte_data(client, 0x30));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x07 value = %8x\n", i2c_smbus_read_byte_data(client, 0x38));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x09 value = %8x\n", i2c_smbus_read_byte_data(client, 0x48));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0D value = %8x\n", i2c_smbus_read_byte_data(client, 0x68));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0E value = %8x\n", i2c_smbus_read_byte_data(client, 0x70));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x0F value = %8x\n", i2c_smbus_read_byte_data(client, 0x71));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x10 value = %8x\n", i2c_smbus_read_byte_data(client, 0x80));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x11 value = %8x\n", i2c_smbus_read_byte_data(client, 0x88));
    len += snprintf(buf+len, PAGE_SIZE-len, "chip id REG 0x13 value = %8x\n", i2c_smbus_read_byte_data(client, 0x98));

    return len;

}

/*----------------------------------------------------------------------------*/
static ssize_t epl2182_show_status(struct device_driver *ddri, char *buf)
{
    ssize_t len = 0;
    struct epl2182_priv *epld = epl2182_obj;
	uint8_t read_data[2];
    if(!epl2182_obj)
    {
        APS_ERR("epl2182_obj is null!!\n");
        return 0;
    }
    elan_epl2182_I2C_Write(epld->client,REG_7,W_SINGLE_BYTE,0x02,EPL_DATA_LOCK);

    elan_epl2182_I2C_Read(epld->client,REG_16,R_TWO_BYTE,0x02,read_data);
    gRawData.ps_raw = (read_data[1]<<8) | read_data[0];
    APS_LOG("ch1 raw_data = %d\n", gRawData.ps_raw);

    elan_epl2182_I2C_Write(epld->client,REG_7,W_SINGLE_BYTE,0x02,EPL_DATA_UNLOCK);
    len += snprintf(buf+len, PAGE_SIZE-len, "ch1 raw is %d\n",gRawData.ps_raw);
    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl2182_store_als_int_time(struct device_driver *ddri, const char *buf, size_t count)
{
    if(!epl2182_obj)
    {
        APS_ERR("epl2182_obj is null!!\n");
        return 0;
    }

    sscanf(buf, "0x7");
    APS_LOG("als int time is %d\n", EPL_ALS_INTT_512>>4);
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t epl2182_store_ps_int_time(struct device_driver *ddri, const char *buf, size_t count)
{
    if(!epl2182_obj)
    {
        APS_ERR("epl2182_obj is null!!\n");
        return 0;
    }
    sscanf(buf, "0x4");
    APS_LOG("ps int time is %d\n", EPL_PS_INTT_80>>4);
    return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl2182_store_ps_interrupt(struct device_driver *ddri, const char *buf, size_t count)
{
	if (!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return 0;
	}
	sscanf(buf, "%d", &isInterrupt);
	APS_LOG("ps int time is %d\n", isInterrupt);


	epl2182_restart_polling();
	return count;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl2182_show_ps_threshold(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	//struct epl2182_priv *obj = epl2182_obj;
	len += snprintf(buf + len, PAGE_SIZE - len, "EPL2182-Threshold(H/L): %d/%d \r\n", epl2182_obj->ps_thd_val_high, epl2182_obj->ps_thd_val_low);
	return len;
}

/*----------------------------------------------------------------------------*/
static ssize_t epl2182_store_ps_threshold(struct device_driver *ddri, const char *buf, size_t count)
{
	sscanf(buf, "%d,%d", (int *)&(epl2182_obj->ps_thd_val_high), (int *)&(epl2182_obj->ps_thd_val_low));
	set_psensor_intr_threshold(epl2182_obj->ps_thd_val_low, epl2182_obj->ps_thd_val_high);
	return count;
}

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(alsrawlux,     S_IWUSR | S_IRUGO, epl2182_show_als,   NULL); 
static DRIVER_ATTR(status, S_IWUSR | S_IRUGO, epl2182_show_status, NULL);
static DRIVER_ATTR(reg, S_IWUSR | S_IRUGO, epl2182_show_reg, NULL);
static DRIVER_ATTR(als_int_time, S_IWUSR | S_IRUGO, NULL, epl2182_store_als_int_time);
static DRIVER_ATTR(ps_int_time, S_IWUSR | S_IRUGO, NULL, epl2182_store_ps_int_time);
static DRIVER_ATTR(ps_interrupt, S_IWUSR | S_IRUGO, NULL, epl2182_store_ps_interrupt);
static DRIVER_ATTR(ps_threshold, S_IWUSR | S_IRUGO, epl2182_show_ps_threshold, epl2182_store_ps_threshold);

/*----------------------------------------------------------------------------*/
static struct driver_attribute * epl2182_attr_list[] =
{
	&driver_attr_alsrawlux,
	&driver_attr_status,
	&driver_attr_reg,
	&driver_attr_als_int_time,
	&driver_attr_ps_int_time,
	&driver_attr_ps_interrupt,
	&driver_attr_ps_threshold,
};

/*----------------------------------------------------------------------------*/
static int epl2182_create_attr(struct device_driver *driver)
{
    int idx, err = 0;
    int num = (int)(sizeof(epl2182_attr_list)/sizeof(epl2182_attr_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        err = driver_create_file(driver, epl2182_attr_list[idx]);
        if(err)
        {
            APS_ERR("driver_create_file (%s) = %d\n", epl2182_attr_list[idx]->attr.name, err);
            break;
        }
    }
    return err;
}



/*----------------------------------------------------------------------------*/
static int epl2182_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(epl2182_attr_list)/sizeof(epl2182_attr_list[0]));

    if (!driver)
        return -EINVAL;

    for (idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, epl2182_attr_list[idx]);
    }

    return err;
}



/******************************************************************************
 * Function Configuration
******************************************************************************/
static int epl2182_open(struct inode *inode, struct file *file)
{
    file->private_data = epl2182_i2c_client;

    APS_FUN();

    if (!file->private_data)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    return nonseekable_open(inode, file);
}

/*----------------------------------------------------------------------------*/
static int epl2182_release(struct inode *inode, struct file *file)
{
    APS_FUN();
    file->private_data = NULL;
    return 0;
}

/*----------------------------------------------------------------------------*/



static int set_psensor_threshold(struct i2c_client *client)
{
	struct epl2182_priv *obj = i2c_get_clientdata(client);
	int databuf[2];
	int res = 0;
	databuf[0] = obj->ps_thd_val_low;
	databuf[1] = obj->ps_thd_val_high;//threshold value need to confirm

	res = set_psensor_intr_threshold(databuf[0], databuf[1]);

	return res;
}

void epl2182_restart_polling(void)
{
    //struct epl2182_priv *epld = epl2182_obj;

    cancel_delayed_work(&polling_work);

    schedule_delayed_work(&polling_work, msecs_to_jiffies(50)); //liqiang modified
}

static void polling_do_work(struct work_struct *work)
{
    struct epl2182_priv *epld = epl2182_obj;
    struct i2c_client *client = epld->client;

    bool enable_als=test_bit(CMC_BIT_ALS, &epld->enable);
    bool enable_ps=test_bit(CMC_BIT_PS, &epld->enable);

	APS_LOG("als / ps enable: %d / %d \n", enable_als, enable_ps);

    cancel_delayed_work(&polling_work);
#if 0
    if((enable_als && enable_ps) || (enable_ps && isInterrupt == 0) || (enable_als && enable_ps == false)){
        queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(PS_DELAY * 2 + ALS_DELAY));
        //queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(350));
    }
#else
    queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(PS_DELAY * 2 + ALS_DELAY + 30));
    //queue_delayed_work(epld->epl_wq, &polling_work,msecs_to_jiffies(350));
#endif

    // epl2182_ps_calibrating incall,  polling work do nothing++++dinggaoshan.20150922
    if (atomic_read(&epl2182_ps_calibrating) == 1)
    {
        return;
    }
    // ----------end--------------dinggaoshan.20150922

	//0816
	if (enable_als && atomic_read(&epld->als_suspend) == 0 && test_bit(CMC_BIT_PS, &epld->pending_intr) == 0)
	{
		mutex_lock(&sensor_mutex);
		elan_epl2182_lsensor_enable(epld, 1);
		epl2182_read_als(client, &gRawData.als_ch1_raw);
		mutex_unlock(&sensor_mutex);
		if (test_bit(CMC_BIT_ALS, &epld->enable))
		{  
			gRawData.als_lux = gRawData.als_ch1_raw*LUX_PER_COUNT / 1000;
			if (als_value_backup ==-2)  als_value_backup = -1;  //zhangxin 20151209 send data to framework before sleep
		}   
		else
		    APS_LOG("epl2182 read als while als closed!\n");
	}
	//0816
	if (enable_ps && atomic_read(&epld->ps_suspend) == 0 && test_bit(CMC_BIT_PS, &epld->pending_intr) == 0)
	{
	    //zhangxin 20151024 add for make system sleep by cancel wakelock.
		if (epld->enable_pflag == 1)
		{
			APS_LOG("WARNING: no need to elan_epl2182_psensor_enable(), just return \n");  
			return;
	    	}
	    //-----wake_lock to prevent goto sleep before ps be seted ok-----dinggaoshan-20151014--
	    wake_lock(&epl2182_ps_wake_lock);
		
		mutex_lock(&sensor_mutex);
		elan_epl2182_psensor_enable(epld, 1);
		mutex_unlock(&sensor_mutex);

		//------unlock after ps be seted ok-----------dinggaoshan-20151014
		wake_unlock(&epl2182_ps_wake_lock);
		if (isInterrupt)
		{
		}
		else
		{
			mutex_lock(&sensor_mutex);
			epl2182_read_ps(epl2182_obj->client, &epl2182_obj->ps);
			mutex_unlock(&sensor_mutex);
			}
    }

	if(gRawData.ps_suspend_flag)
	{
		cancel_delayed_work(&polling_work);
		return;
	}

	if (!enable_als &&  !enable_ps)
	{
		epld->enable_pflag = 0;
		epld->enable_lflag = 0;
		cancel_delayed_work(&polling_work);
		APS_LOG("disable sensor\n");
		elan_epl2182_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02, EPL_INT_DISABLE);
		elan_epl2182_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0X02, EPL_S_SENSING_MODE);
	 //09-25 add by Charles  add for suspend -resume TP closing Issue
            }

}

/*----------------------------------------------------------------------------*/
static long epl2182_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct i2c_client *client = (struct i2c_client*)file->private_data;
    struct epl2182_priv *obj = i2c_get_clientdata(client);
    int err = 0;
    void __user *ptr = (void __user*) arg;
    int dat;
    uint32_t enable;
    bool enable_als=test_bit(CMC_BIT_ALS, &obj->enable);
    bool enable_ps=test_bit(CMC_BIT_PS, &obj->enable);      

	int ps_result;
	int ps_cali;
	int threshold[2];

    APS_LOG("---epl2182_ioctll- cmd = %x........\r\n", cmd);

    switch (cmd)
    {
        case ALSPS_SET_PS_MODE:
            if(copy_from_user(&enable, ptr, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
			

            if(enable)
            {
                set_bit(CMC_BIT_PS, &obj->enable);
            }
            else
            {
                clear_bit(CMC_BIT_PS, &obj->enable);
            }
            epl2182_restart_polling();
            break;


        case ALSPS_GET_PS_MODE:
            enable=test_bit(CMC_BIT_PS, &obj->enable);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;


        case ALSPS_GET_PS_DATA:
#if 0
            if((err = elan_epl2182_psensor_enable(obj, 1))!=0)
            {
                APS_ERR("enable ps fail: %d\n", err);
                return -1;
            }
            epl2182_read_ps(obj->client, &obj->ps);
#else
            if(enable_ps == 0)
            {
                set_bit(CMC_BIT_PS, &obj->enable);
                epl2182_restart_polling();
            }
#endif
           dat = gRawData.ps_state;                          

            APS_LOG("ioctl ps state value = %d \n", dat);

            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;


        case ALSPS_GET_PS_RAW_DATA:
#if 0
            if((err = elan_epl2182_psensor_enable(obj, 1))!=0)
            {
                APS_ERR("enable ps fail: %d\n", err);
                return -1;
		    }
            epl2182_read_ps(obj->client, &obj->ps);
            dat = obj->ps;
#else
            if(enable_ps == 0)
            {
                set_bit(CMC_BIT_PS, &obj->enable);
                epl2182_restart_polling();
            }
	

           dat = gRawData.ps_raw;
#endif
            APS_LOG("ioctl ps raw value = %d \n", dat);
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
                set_bit(CMC_BIT_ALS, &obj->enable);
            }
            else
            {
                clear_bit(CMC_BIT_ALS, &obj->enable);
            }
            break;



        case ALSPS_GET_ALS_MODE:
            enable=test_bit(CMC_BIT_ALS, &obj->enable);
            if(copy_to_user(ptr, &enable, sizeof(enable)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;



        case ALSPS_GET_ALS_DATA:
#if 0
            if((err = elan_epl2182_lsensor_enable(obj, 1))!=0)
            {
                APS_ERR("disable als fail: %d\n", err);
                return -1;
            }

            epl2182_read_als(obj->client, &obj->als);
            dat = epl2182_get_als_value(obj, obj->als);
#else
            if(enable_als == 0)
            {
                set_bit(CMC_BIT_ALS, &obj->enable);
                epl2182_restart_polling();
            }

            dat = epl2182_get_als_value(obj, gRawData.als_lux);
#endif
            //APS_LOG("ioctl get als data = %d, gRawData.als_lux=%d, ch1=%d\n", dat, gRawData.als_lux, gRawData.als_ch1_raw);

#if 0
            if(obj->enable_pflag && isInterrupt)
            {
                if((err = elan_epl2182_psensor_enable(obj, 1))!=0)
                {
                    APS_ERR("disable ps fail: %d\n", err);
                    return -1;
                }
            }
#endif
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;


        case ALSPS_GET_ALS_RAW_DATA:
#if 0
            if((err = elan_epl2182_lsensor_enable(obj, 1))!=0)
            {
                APS_ERR("disable als fail: %d\n", err);
                return -1;
            }

            epl2182_read_als(obj->client, &obj->als);
            dat = obj->als;
#else
            if(enable_als == 0)
            {
                set_bit(CMC_BIT_ALS, &obj->enable);
                epl2182_restart_polling();
             }
			
            dat = gRawData.als_ch1_raw;
#endif
            APS_DBG("ioctl get als raw data = %d\n", dat);

#if 0
            if(obj->enable_pflag && isInterrupt)
            {
                if((err = elan_epl2182_psensor_enable(obj, 1))!=0)
                {
                    APS_ERR("disable ps fail: %d\n", err);
                    return -1;
                }
            }
#endif
            if(copy_to_user(ptr, &dat, sizeof(dat)))
            {
                err = -EFAULT;
                goto err_out;
            }
            break;
			/*----------------------------------for factory mode test---------------------------------------*/
						case ALSPS_GET_PS_TEST_RESULT:
							if((err = epl2182_read_ps(obj->client, &obj->ps)))
							{
								goto err_out;
							}
							if(obj->ps > obj->ps_thd_val_high)
								{
									ps_result = 0;
								}
							else	ps_result = 1;

							if(copy_to_user(ptr, &ps_result, sizeof(ps_result)))
							{
								err = -EFAULT;
								goto err_out;
							}
							break;


						case ALSPS_IOCTL_CLR_CALI:
							if(copy_from_user(&dat, ptr, sizeof(dat)))
							{
								err = -EFAULT;
								goto err_out;
							}
							if(dat == 0)
								obj->ps_cali = 0;
							break;

						case ALSPS_IOCTL_GET_CALI:
							ps_cali = obj->ps_cali ;
							APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
							if(copy_to_user(ptr, &ps_cali, sizeof(ps_cali)))
							{
								err = -EFAULT;
								goto err_out;
							}
							break;

						case ALSPS_IOCTL_SET_CALI:
							if(copy_from_user(&ps_cali, ptr, sizeof(ps_cali)))
							{
								err = -EFAULT;
								goto err_out;
							}

							obj->ps_cali = ps_cali;

							APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
							break;

						case ALSPS_SET_PS_THRESHOLD:
							if(copy_from_user(threshold, ptr, sizeof(threshold)))
							{
								err = -EFAULT;
								goto err_out;
							}
							APS_ERR("%s set threshold high: 0x%x, low: 0x%x\n", __func__, threshold[0],threshold[1]);
							obj->ps_thd_val_high = threshold[0]+obj->ps_cali;
							obj->ps_thd_val_low = threshold[1]+obj->ps_cali;//need to confirm

		set_psensor_threshold(obj->client);

		break;

						case ALSPS_GET_PS_THRESHOLD_HIGH:
							APS_ERR("%s get threshold high before cali: 0x%x\n", __func__, obj->ps_thd_val_high);
							threshold[0] = obj->ps_thd_val_high - obj->ps_cali;
							APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
							APS_ERR("%s get threshold high: 0x%x\n", __func__, threshold[0]);
							if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
							{
								err = -EFAULT;
								goto err_out;
							}
							break;

						case ALSPS_GET_PS_THRESHOLD_LOW:
							APS_ERR("%s get threshold low before cali: 0x%x\n", __func__, obj->ps_thd_val_low);
							threshold[0] = obj->ps_thd_val_low - obj->ps_cali;
							APS_ERR("%s set ps_calix%x\n", __func__, obj->ps_cali);
							APS_ERR("%s get threshold low: 0x%x\n", __func__, threshold[0]);
							if(copy_to_user(ptr, &threshold[0], sizeof(threshold[0])))
							{
								err = -EFAULT;
								goto err_out;
							}
							break;
						/*------------------------------------------------------------------------------------------*/



        default:
            APS_ERR("%s not supported = 0x%04x", __FUNCTION__, cmd);
            err = -ENOIOCTLCMD;
            break;
    }

err_out:
    return err;
}


/*----------------------------------------------------------------------------*/
static struct file_operations epl2182_fops =
{
    .owner = THIS_MODULE,
    .open = epl2182_open,
    .release = epl2182_release,
    .unlocked_ioctl = epl2182_unlocked_ioctl,
};


/*----------------------------------------------------------------------------*/
static struct miscdevice epl2182_device =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "als_ps",
    .fops = &epl2182_fops,
};


/*----------------------------------------------------------------------------*/
static int epl2182_i2c_suspend(struct i2c_client *client, pm_message_t msg)
{
    struct epl2182_priv *obj = i2c_get_clientdata(client);
    int err = 0;
    APS_FUN();
	//epl2182_power(epl2182_obj->hw, 1);

    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    atomic_set(&obj->als_suspend, 1);

#if 0
    if(msg.event == PM_EVENT_SUSPEND)
    {
        if(!obj)
        {
            APS_ERR("null pointer!!\n");
            return -EINVAL;
        }

        atomic_set(&obj->als_suspend, 1);
        if((err = elan_epl2182_lsensor_enable(obj, 0))!=0)
        {
            APS_ERR("disable als: %d\n", err);
            return err;
        }

        atomic_set(&obj->ps_suspend, 1);
        if((err = elan_epl2182_psensor_enable(obj, 0))!=0)
        {
            APS_ERR("disable ps:  %d\n", err);
            return err;
        }

        epl2182_power(obj->hw, 0);
    }
#endif

    return err;

}



/*----------------------------------------------------------------------------*/
static int epl2182_i2c_resume(struct i2c_client *client)
{
    struct epl2182_priv *obj = i2c_get_clientdata(client);//liqiang
    int err = 0;
    APS_FUN();

    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    atomic_set(&obj->als_suspend, 0);

    atomic_set(&obj->ps_suspend, 0);

    epl2182_restart_polling();
    
#if 0//liqiang 2014
    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return -EINVAL;
    }

    epl2182_power(obj->hw, 1);

    msleep(50);

    if(err = epl2182_init_client(client))
    {
        APS_ERR("initialize client fail!!\n");
        return err;
    }

    atomic_set(&obj->als_suspend, 0);
    if(test_bit(CMC_BIT_ALS, &obj->enable))
    {
        if((err = elan_epl2182_lsensor_enable(obj, 1))!=0)
        {
            APS_ERR("enable als fail: %d\n", err);
        }
    }
    atomic_set(&obj->ps_suspend, 0);
    if(test_bit(CMC_BIT_PS,  &obj->enable))
    {
        if((err = elan_epl2182_psensor_enable(obj, 1))!=0)
        {
            APS_ERR("enable ps fail: %d\n", err);
        }
    }


    if(obj->hw->polling_mode_ps == 0)
        epl2182_setup_eint(client);
#endif
    return err;
}


#if 0
/*----------------------------------------------------------------------------*/
static void epl2182_early_suspend(struct early_suspend *h)
{
    /*early_suspend is only applied for ALS*/
    struct epl2182_priv *obj = container_of(h, struct epl2182_priv, early_drv);
    int err;
    APS_FUN();

    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return;
    }

    atomic_set(&obj->als_suspend, 1);
	/*
    if(isInterrupt)
	{
		 gRawData.ps_suspend_flag = true;
	}
	*/
	/*
	if(test_bit(CMC_BIT_PS, &obj->enable) == 0)
	{
		atomic_set(&obj->ps_suspend, 1);
		msleep(PS_DELAY * 2 + ALS_DELAY + 30);		
		elan_epl2182_lsensor_enable(obj, 1);//confirm INT pin doesn't keep in Low voltage with active als mode ONCE and switch to idle mode		
    	}
    	*/
}



/*----------------------------------------------------------------------------*/
static void epl2182_late_resume(struct early_suspend *h)
{
    /*late_resume is only applied for ALS*/
    struct epl2182_priv *obj = container_of(h, struct epl2182_priv, early_drv);
    int err;
    APS_FUN();

    if(!obj)
    {
        APS_ERR("null pointer!!\n");
        return;
    }

    atomic_set(&obj->als_suspend, 0);



    atomic_set(&obj->ps_suspend, 0);
	/*
    if(isInterrupt)
		gRawData.ps_suspend_flag = false;
	*/

	epl2182_restart_polling();
}
#endif

/*--------------------------------------------------------------------------------*/
static int als_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int als_enable_nodata(int en)
{
	//int res = 0;
	if(!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return -1;
	}
	APS_LOG("epl2182_obj als enable value = %d\n", en);

        als_value_backup = -2;  //zhangxin 20151209 send data to framework before sleep

	if(en)
	{
		set_bit(CMC_BIT_ALS, &epl2182_obj->enable);
	    epl2182_restart_polling();
	}
	else
	{
		clear_bit(CMC_BIT_ALS, &epl2182_obj->enable);
	}
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int als_set_delay(u64 ns)
{
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int als_get_data(int* value, int* status)
{
	int err = 0;

	if(!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return -1;
	}

	err = epl2182_get_als_value(epl2182_obj, gRawData.als_lux);

	if (err<0) return err;   //zhangxin 20151209 send data to framework before sleep

	*value = err;
       *status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int ps_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	return 0;
}
/*--------------------------------------------------------------------------------*/
// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL
static int ps_enable_nodata(int en)
{
	//int res = 0;
	if(!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return -1;
	}
	APS_LOG("epl2182_obj ps enable value = %d\n", en);

	if(en)
	{
		if(isInterrupt)
		{
			//mt_eint_unmask(CUST_EINT_ALS_NUM);
			if (EPL2182_DISFLAG_PS == epl2182_obj->ps_sensor_flag)
			{         
				epl2182_obj->ps_sensor_flag = EPL2182_ENBFLAG_PS;
				enable_irq(epl2182_obj->irq);
				APS_LOG("ps_enable_nodata enable_irq flag=%d\n", epl2182_obj->ps_sensor_flag);
			}
			else 
			{
				APS_LOG("ps_enable_nodata enable_irq error flag=%d\n", epl2182_obj->ps_sensor_flag);
			}
            		gRawData.ps_int_state = 2;
		}
		//gRawData.ps_state  = -1; //zhangxin 20150824 modify for light-quickly when screen-off in talk.
		//queue_delayed_work(epl2182_obj->epl_wq, &polling_work,msecs_to_jiffies(5));
		set_bit(CMC_BIT_PS, &epl2182_obj->enable);
		epl2182_restart_polling();
 	}
	else
	{
            if (EPL2182_ENBFLAG_PS == epl2182_obj->ps_sensor_flag)
            {   
                 epl2182_obj->ps_sensor_flag = EPL2182_DISFLAG_PS;
                 disable_irq_nosync(epl2182_obj->irq);
                 APS_LOG("ps_enable_nodata disable_irq_nosync flag=%d\n", epl2182_obj->ps_sensor_flag);
            }
            else 
            {
                 APS_LOG("ps_enable_nodata disable_irq_nosync error flag=%d\n", epl2182_obj->ps_sensor_flag);
            }
            
		clear_bit(CMC_BIT_PS, &epl2182_obj->enable);
	}
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int ps_set_delay(u64 ns)
{
	return 0;
}
/*--------------------------------------------------------------------------------*/
static int ps_get_data(int* value, int* status)
{
	int err = 0;
    
 	if(!epl2182_obj)
	{
		APS_ERR("epl2182_obj is null!!\n");
		return -1;
	}

	APS_LOG("[%s]: gRawData.ps_state=%d, gRawData.ps_raw=%d \r\n", __func__, gRawData.ps_state, gRawData.ps_raw);
	
	if (gRawData.ps_state==0)                  *value = 4;		
	else if (gRawData.ps_state==1)          *value = 101;

	*status = SENSOR_STATUS_ACCURACY_MEDIUM;

	return err;
}


#if 1       // proc file for emode 
static struct proc_dir_entry *epl2182_proc_file = NULL;
static void epl2182_enable_ps_when_calibrate(int enable)
{
	int ret;
	uint8_t regdata = 0;
	struct i2c_client *client = epl2182_obj->client;

	APS_DBG("%s\n", __func__);
	if(enable)
	{
		ret = elan_epl2182_I2C_Write(client, REG_9, W_SINGLE_BYTE, 0x02, EPL_INT_DISABLE|PS_DRIVE);

		//default setting
		regdata = EPL_SENSING_2_TIME | EPL_PS_MODE | EPL_L_PS_GAIN | EPL_C_SENSING_MODE ;
		ret = elan_epl2182_I2C_Write(client, REG_0, W_SINGLE_BYTE, 0X02, regdata);

		//default setting
		regdata = EPL_PS_INTT_80| EPL_PST_1_TIME | EPL_12BIT_ADC;
		ret = elan_epl2182_I2C_Write(client, REG_1, W_SINGLE_BYTE, 0X02, regdata);

		//set_psensor_intr_threshold(epld->ps_th_l, epld->ps_th_h);

		ret = elan_epl2182_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0X02, EPL_C_RESET);
		ret = elan_epl2182_I2C_Write(client, REG_7, W_SINGLE_BYTE, 0x02, EPL_C_START_RUN);
    }
}

static int epl2182_calibrate(void)
{
    struct epl2182_priv *data = epl2182_obj;
    int i, temp_pdata[10];
    int cross_talk, sum, max, min;
	uint8_t read_data[2]={0};

    APS_DBG("%s, in\n",__func__);
    if(NULL == data)
    {
        APS_DBG("epl2182_calibrate, data is NULL!\n");
        return -1;
    }

	atomic_set(&epl2182_ps_calibrating, 1);

	cancel_delayed_work(&polling_work);
	
//    if(0 == data->enable_pflag)  //zhangxin 20150917 modify for calibration fail.
    {
        epl2182_enable_ps_when_calibrate(1);
    }
    
    sum = 0;
    max = 0;
    min = 0XFFFF;
	for (i = 0; i < 10; i++) {
		msleep(PS_DELAY);
		
		elan_epl2182_I2C_Read(data->client,REG_16,R_TWO_BYTE,0x02, read_data);
		temp_pdata[i] = (read_data[1]<< 8) |read_data[0];
		APS_DBG("%s,data[%d]=%d\n", __FUNCTION__, i, temp_pdata[i]);

		elan_epl2182_I2C_Write(data->client, REG_7, W_SINGLE_BYTE, 0x02,
				      EPL_DATA_UNLOCK);
	
		sum += temp_pdata[i];
		if(max < temp_pdata[i]) 
		{ 
		    max = temp_pdata[i];
		}
		if(min > temp_pdata[i]) 
		{ 
		    min = temp_pdata[i];
		}
	}

    if(0 == data->enable_pflag)
    {
		if(0 == data->enable_lflag)
		{
            //Idle mode
       		elan_epl2182_I2C_Write(data->client, REG_9, W_SINGLE_BYTE, 0x02,
				      EPL_INT_DISABLE);
    		elan_epl2182_I2C_Write(data->client, REG_0, W_SINGLE_BYTE, 0X02,
				      EPL_S_SENSING_MODE);

		}
		else
		{
        //Nothing to do
		}
    }
	schedule_delayed_work(&polling_work, msecs_to_jiffies(50));
	atomic_set(&epl2182_ps_calibrating, 0);

	cross_talk = sum / 10;

	APS_DBG("%s,cross_talk=%d, max=%d, min=%d\n", __FUNCTION__, cross_talk, max, min);

    return cross_talk;
}
static void check_prox_mean(int prox_mean ,unsigned int *detection_threshold,unsigned int *hsyteresis_threshold)
{
    int prox_threshold_hi_param, prox_threshold_lo_param;
	// some dev's crosstalk between 20 and 100, distance is too far, add a branch--20151019dinggaoshan++++
	if (prox_mean < 100)
	{
        prox_threshold_hi_param = prox_mean + 180;
		prox_threshold_lo_param = prox_mean + 140;
	}
	//-------------------------------<end little than 100 branch>------------20151019dinggaoshan----
    else if(prox_mean < 400)
    {
    	prox_threshold_hi_param = prox_mean * 5/2; /*35/10*/
    	prox_threshold_lo_param = prox_mean * 2;   /*25/10*/
    }
	else
    {
    	prox_threshold_hi_param = prox_mean * 2;   //*3
    	prox_threshold_lo_param = prox_mean * 8/5; //*2
    }
	

	*detection_threshold = prox_threshold_hi_param;
	*hsyteresis_threshold = prox_threshold_lo_param;

	APS_DBG("cross_talk=%d, high_threshold=%d, low_threshold=%d\n", prox_mean, prox_threshold_hi_param,prox_threshold_lo_param);
}
static ssize_t epl2182_read_proc(struct file *file, char __user *user_buf, size_t len, loff_t *offset)
{
    ssize_t count;

	if(*offset)
	{   return 0;}

	epl2182_ps_crosstalk = epl2182_calibrate();
	if (epl2182_ps_crosstalk<=0)   epl2182_ps_crosstalk = EPL2182_CROSSTALK_DEFAULT;
	if (epl2182_ps_crosstalk >= EPL2182_CROSSTALK_MAX)	epl2182_ps_crosstalk = EPL2182_CROSSTALK_MAX;

	check_prox_mean(epl2182_ps_crosstalk,&epl2182_obj->ps_thd_val_high, &epl2182_obj->ps_thd_val_low);
	APS_DBG("%s, epl2182_ps_crosstalk=%d, threshold[%d, %d]\n", __func__,epl2182_ps_crosstalk, epl2182_obj->ps_thd_val_high, epl2182_obj->ps_thd_val_low);
	set_psensor_intr_threshold(epl2182_obj->ps_thd_val_low, epl2182_obj->ps_thd_val_high);
	
	count = sprintf(user_buf,"%d\n",epl2182_ps_crosstalk);
	*offset += count;
	return count;
}

static ssize_t epl2182_write_proc(struct file *file, const char __user *user_buf, size_t len, loff_t *offset)
{
	char buf[16];
	int value = 0;
	if (copy_from_user(buf, user_buf, len))
	{
		APS_DBG("%s, copy_from_user error\n", __func__);
		return -EFAULT;
	}
    
	sscanf(buf, "%d", &value);

	epl2182_ps_crosstalk = epl2182_calibrate();

	if(value > 0)
	{
		if(epl2182_ps_crosstalk > value+300)
		{
			epl2182_ps_crosstalk = value;
		}
	}
	if (epl2182_ps_crosstalk<=0)   epl2182_ps_crosstalk = EPL2182_CROSSTALK_DEFAULT;
	if (epl2182_ps_crosstalk >= EPL2182_CROSSTALK_MAX)	epl2182_ps_crosstalk = EPL2182_CROSSTALK_MAX;

	epl2182_ps_crosstalk_incall = epl2182_ps_crosstalk;
	check_prox_mean(epl2182_ps_crosstalk,&epl2182_obj->ps_thd_val_high, &epl2182_obj->ps_thd_val_low);
	APS_DBG("%s, epl2182_ps_crosstalk=%d, threshold[%d, %d]\n", __func__,epl2182_ps_crosstalk, epl2182_obj->ps_thd_val_high, epl2182_obj->ps_thd_val_low);
	set_psensor_intr_threshold(epl2182_obj->ps_thd_val_low, epl2182_obj->ps_thd_val_high);
	
	return len;
}

static const struct file_operations epl2182_proc_fops = {
	.owner		= THIS_MODULE,
	//.open       = epl2182_calibrate_open,
	.read       = epl2182_read_proc,
	.write      = epl2182_write_proc,
};
#endif

#if 1     // proc file for  proximity-check tools
static struct proc_dir_entry *prox_calibration_value = NULL;
static ssize_t prox_calibration_value_read(struct file *file,	char __user *buffer, size_t count, loff_t *offset)
{
    //int p_data = 0;
    int cnt;	
    APS_FUN();
    
    if(*offset != 0)
    {
        APS_DBG("%s,return 0\n", __FUNCTION__);
        return 0;
    }

	cancel_delayed_work(&polling_work);

	elan_epl2182_psensor_enable(epl2182_obj, 1);
   
	cnt = sprintf(buffer, "%d %d %d %d %d\n", epl2182_ps_crosstalk, epl2182_ps_crosstalk_incall, 
		                                                          epl2182_obj->ps_thd_val_high, epl2182_obj->ps_thd_val_low, gRawData.ps_raw);
	
	schedule_delayed_work(&polling_work, msecs_to_jiffies(50));
	
    *offset += cnt;    
    return cnt;
}

static const struct file_operations prox_calibration_value_fops = {
	.owner		= THIS_MODULE,	
	.read       = prox_calibration_value_read,	
};
#endif 

#if   1    // proc file for calibration in call
static struct proc_dir_entry *calibration_inCall = NULL;
static ssize_t calibration_inCall_read(struct file *file,	char __user *buffer, size_t count, loff_t *offset)
{
	int len;
	int crosstalk=0;

	APS_FUN();
	
    if(*offset != 0)
    {
        APS_DBG("%s,return 0\n", __FUNCTION__);
        return 0;
    }

	crosstalk = epl2182_calibrate();
	APS_DBG("%s, ori-cross_talk=%d, now-crosstalk=%d\n", __func__,epl2182_ps_crosstalk, crosstalk);

	if (crosstalk<=0) 
	{
		crosstalk= epl2182_ps_crosstalk;
	}
	else if (crosstalk<=epl2182_ps_crosstalk)
	{
		epl2182_ps_crosstalk = crosstalk;
	}
	else if (crosstalk>(epl2182_ps_crosstalk+EPL2182_CROSSTALK_UPLIMIT))
	{
		crosstalk = epl2182_ps_crosstalk+EPL2182_CROSSTALK_UPLIMIT;
		APS_DBG("now-cross_talk >> ori-crosstalk much, so ori-crosstalk not change!!\n");
	}

	check_prox_mean(crosstalk,&epl2182_obj->ps_thd_val_high, &epl2182_obj->ps_thd_val_low);
	APS_DBG("%s, set crosstalk=%d, threshold[%d, %d]\n", __func__, crosstalk, epl2182_obj->ps_thd_val_high, epl2182_obj->ps_thd_val_low);
	set_psensor_intr_threshold(epl2182_obj->ps_thd_val_low, epl2182_obj->ps_thd_val_high);

	epl2182_ps_crosstalk_incall = crosstalk;

	len = sprintf(buffer, "%d", crosstalk);
	*offset += len;
	return len;
}

static const struct file_operations calibration_inCall_fops = {
	.owner		= THIS_MODULE,
	.read       = calibration_inCall_read,
};
#endif 

static void create_epl2182_proc_file(void)
{
	epl2182_proc_file = proc_create("driver/alsps_threshold", 0666, NULL, &epl2182_proc_fops);

    if(NULL == epl2182_proc_file)
	{
	    printk(KERN_ERR "create_epl2182_proc_file fail!\n");
	}

       prox_calibration_value = proc_create("driver/prox_calibration_value", 0666, NULL, &prox_calibration_value_fops);
       if(NULL == prox_calibration_value)
	{
	    printk(KERN_ERR "create_epl2182_proc_file prox_calibration_value fail!\n");
	}  
	   
	calibration_inCall = proc_create("driver/calibration_inCall", 0444, NULL, &calibration_inCall_fops);
    if(NULL == calibration_inCall)
	{
	    printk(KERN_ERR "create_epl2182_proc_file calibration_inCall  fail!\n");
	}
}
/* -----20150820 zhangxin add proc file interface end-----*/

/*----------------------------------------------------------------------------*/

static int epl2182_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    strcpy(info->type, EPL2182_DEV_NAME);
    return 0;
}

static int of_get_epl2182_platform_data(struct device *dev)
{
	//struct device_node *node = NULL;

	epl2182_obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, ALS-eint");
#if 0
	if (epl2182_obj->irq_node) {
		//alsps_int_gpio_number = of_get_named_gpio(node, "int-gpio", 0);
		alsps_irq = irq_of_parse_and_map(epl2182_obj->irq_node, 0);
		if (alsps_irq < 0) {
			APS_ERR("alsps request_irq IRQ LINE NOT AVAILABLE!.");
			return -1;
		}
		APS_ERR("alsps_irq : %d\n", alsps_irq);
	}
#endif
	return 0;
}

/*----------------------------------------------------------------------------*/
static int epl2182_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct epl2182_priv *obj;
    struct als_control_path als_ctl={0};
	struct als_data_path als_data={0};
	struct ps_control_path ps_ctl={0};
	struct ps_data_path ps_data={0};
    int err = 0;
	int temp_value = 0;
	
    APS_FUN();

    epl2182_dumpReg(client);

	temp_value = i2c_smbus_read_byte_data(client, 0x98);

	printk("[darren-als] elan sensor temp_value = 0x%x\n",temp_value);
	
	if(temp_value != 0x68)
	{
		printk("[darren-als] elan sensor is fail\n");
		err = -ENOTSUPP;
		goto exit;
	}

    if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
        goto exit;
    }

    memset(obj, 0, sizeof(*obj));

    epl2182_obj = obj;
    obj->hw = hw;

    epl2182_get_addr(obj->hw, &obj->addr);

    epl2182_obj->als_level_num = sizeof(epl2182_obj->hw->als_level)/sizeof(epl2182_obj->hw->als_level[0]);
    epl2182_obj->als_value_num = sizeof(epl2182_obj->hw->als_value)/sizeof(epl2182_obj->hw->als_value[0]);
    BUG_ON(sizeof(epl2182_obj->als_level) != sizeof(epl2182_obj->hw->als_level));
    memcpy(epl2182_obj->als_level, epl2182_obj->hw->als_level, sizeof(epl2182_obj->als_level));
    BUG_ON(sizeof(epl2182_obj->als_value) != sizeof(epl2182_obj->hw->als_value));
    memcpy(epl2182_obj->als_value, epl2182_obj->hw->als_value, sizeof(epl2182_obj->als_value));

    INIT_WORK(&obj->eint_work, epl2182_eint_work);
    obj->epl_wq = create_singlethread_workqueue("elan_sensor_wq");

    INIT_WORK(&obj->data_work, epl2182_check_ps_data);

    init_waitqueue_head(&wait_rsp_wq);
	
    mutex_init(&sensor_mutex);

    gRawData.ps_suspend_flag = false;
    obj->client = client;
    obj->client->timing = 400;

    i2c_set_clientdata(client, obj);

    atomic_set(&obj->als_debounce, 2000);
    atomic_set(&obj->als_deb_on, 0);
    atomic_set(&obj->als_deb_end, 0);
    atomic_set(&obj->ps_debounce, 1000);
    atomic_set(&obj->ps_deb_on, 0);
    atomic_set(&obj->ps_deb_end, 0);
    atomic_set(&obj->ps_mask, 0);
    atomic_set(&obj->trace, 0x00);
    atomic_set(&obj->als_suspend, 0);
	
    obj->ps_thd_val_high = obj->hw ->ps_threshold_high;
    obj->ps_thd_val_low = obj->hw ->ps_threshold_low;

    obj->ps_cali = 0;
    obj->ps_enable = 0;
    obj->als_enable = 0;
    obj->lux_per_count = LUX_PER_COUNT;
    obj->enable = 0;
    obj->pending_intr = 0;
    obj->enable_ps_sensor = EPL2182_DISABLE_PS;	// default to 0
    obj->ps_sensor_flag= EPL2182_DISFLAG_PS;	// default to 0
	
    gRawData.ps_state = -1;

    atomic_set(&obj->i2c_retry, 3);

    of_get_epl2182_platform_data(&client->dev);
    //obj->irq = alsps_irq;

    epl2182_i2c_client = client;

/******zhangaifeng@wind-mobi.com***********test begain*************/
    err =elan_epl2182_I2C_Write(client,REG_0,W_SINGLE_BYTE,0x02, EPL_S_SENSING_MODE);
    if(err <0)
    {
		goto exit_init_failed;
	}

    err =elan_epl2182_I2C_Write(client,REG_9,W_SINGLE_BYTE,0x02,EPL_INT_DISABLE);
    if(err <0)
    {
		goto exit_init_failed;
	}
/*************zhangaifeng@wind-mobi.com*****test end******************/

    err = epl2182_init_client(client);
    if(err)
    {
        goto exit_init_failed;
    }

    err = misc_register(&epl2182_device);
    if(err)
    {
        APS_ERR("epl2182_device register failed\n");
        goto exit_misc_device_register_failed;
    }

    err = epl2182_create_attr(&epl2182_init_info.platform_diver_addr->driver);
    if(err)
    {
        APS_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }
	isInterrupt=true;

	als_ctl.open_report_data= als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay  = als_set_delay;
	als_ctl.is_report_input_direct = false;
	als_ctl.is_support_batch = false;

	err = als_register_control_path(&als_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}


	ps_ctl.open_report_data= ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
	ps_ctl.is_report_input_direct = false;
	ps_ctl.is_polling_mode = false;
	ps_ctl.is_support_batch = false;

	err = ps_register_control_path(&ps_ctl);
	if(err)
	{
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if(err)
	{
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

#if 0
//liqiang
#if CHECK_CT_ENABLE
	wind_check_ct.ct_high_threshold = CT_H_TH;
	wind_check_ct.ct_low_threshold = CT_L_TH;
#endif
//liqiang
#endif

/* -----20150820 zhangxin add proc file interface-----*/
    create_epl2182_proc_file();
/* -----20150820 zhangxin add proc file interface-----*/

//-----initial wake_lock for PS-----dinggaoshan-20151014--
    wake_lock_init(&epl2182_ps_wake_lock, WAKE_LOCK_SUSPEND, "epl2182_ps_wake_locks");

    if(isInterrupt)
        epl2182_setup_eint(client);

    alsps_init_flag = 0;
    APS_LOG("%s: OK\n", __func__);
    return 0;

exit_create_attr_failed:
exit_sensor_obj_attach_fail:
    misc_deregister(&epl2182_device);
exit_misc_device_register_failed:
exit_init_failed:
    alsps_init_flag = -1;
    kfree(obj);
exit:
    epl2182_i2c_client = NULL;
    APS_ERR("%s: err = %d\n", __func__, err);
    alsps_init_flag = -1;
    return err;
}



/*----------------------------------------------------------------------------*/
static int epl2182_i2c_remove(struct i2c_client *client)
{
    int err;

    if((err = epl2182_delete_attr(&epl2182_init_info.platform_diver_addr->driver)))
    {
        APS_ERR("epl2182_delete_attr fail: %d\n", err);
    }

    if((err = misc_deregister(&epl2182_device)))
    {
        APS_ERR("misc_deregister fail: %d\n", err);
    }

    epl2182_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));

    return 0;
}



/*----------------------------------------------------------------------------*/
static int alsps_local_init(void)
{
    //struct epl_alsps_hw *hw = get_cust_alsps_hw_epl2182();
	printk("[darren-als] fwq loccal init  alsps_init_flag = %d\n",alsps_init_flag);

	epl2182_power(hw, 1);
	if(i2c_add_driver(&epl2182_i2c_driver))
	{
		APS_ERR("add driver error\n");
		printk("[darren-als]add driver epl2182 error\n");
		return -1;
	}
	if(-1 == alsps_init_flag)
	{
	   return -1;
	}
	printk("[darren-als]  alsps_init_flag = %d\n",alsps_init_flag);
	return 0;
}
/*----------------------------------------------------------------------------*/
static int alsps_remove()
{
    //struct epl_alsps_hw *hw = get_cust_alsps_hw_epl2182();
    APS_FUN();
    epl2182_power(hw, 0);

    APS_ERR("EPL2182 remove \n");
    i2c_del_driver(&epl2182_i2c_driver);
    return 0;
}
/*----------------------------------------------------------------------------*/
static int __init epl2182_init(void)
{
    APS_FUN();

    hw = get_alsps_dts_func("mediatek,epl2182", hw);
    if (!hw) {
	   APS_ERR("get dts info fail\n");
	   return 0;
    }

    alsps_driver_add(&epl2182_init_info);
    return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit epl2182_exit(void)
{
    APS_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(epl2182_init);
module_exit(epl2182_exit);
/*----------------------------------------------------------------------------*/
MODULE_AUTHOR("yucong.xiong@mediatek.com");
MODULE_DESCRIPTION("EPL2182 ALSPS driver");
MODULE_LICENSE("GPL");
//lichengmin end





