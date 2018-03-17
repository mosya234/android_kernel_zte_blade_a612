/*
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

 
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>

#include "cust_alsps.h"
#include "ltr559.h"
#include "alsps.h"
#include "../inc/alsps_calibration.h" // lijiangshuo add for ps calibration 20160104

#define LTR559_DEV_NAME	"ltr559"
#define APS_TAG				"[ltr559] "

#define APS_FUN(f)			printk(KERN_INFO APS_TAG"%s\n", __FUNCTION__)
#define APS_ERR(fmt, args...)	pr_err(APS_TAG fmt, ##args)
#define APS_LOG(fmt, args...)	pr_debug(APS_TAG fmt, ##args)
#define APS_DBG(fmt, args...)	pr_debug(APS_TAG fmt, ##args)

#define PS_CAL_NUM		5//10

typedef enum {
	CMC_BIT_ALS	= 1,
	CMC_BIT_PS		= 2,
} CMC_BIT;

struct ltr559_priv {
	struct alsps_hw	*hw;
	struct i2c_client	*client;
	struct work_struct	eint_work;
	int	als_enabled;
	int	ps_enabled;

	 /*misc*/
	atomic_t	als_debounce;	/*debounce time after enabling als*/
	atomic_t	als_deb_on;		/*indicates if the debounce is on*/
	atomic_t	als_deb_end;		/*the jiffies representing the end of debounce*/
	atomic_t	ps_mask;		/*mask ps: always return far away*/
	atomic_t	ps_debounce;	/*debounce time after enabling ps*/
	atomic_t	ps_deb_on;		/*indicates if the debounce is on*/
	atomic_t	ps_deb_end;		/*the jiffies representing the end of debounce*/
	atomic_t	ps_suspend;
	atomic_t	als_suspend;

	atomic_t	init_done;
	struct device_node	*irq_node;
	int		irq;

	/*data*/
	u16		als;
	u16		ps;
	u8		_align;
	u16		als_level_num;
	u16		als_value_num;
	u32		als_level[C_CUST_ALS_LEVEL-1];
	u32		als_value[C_CUST_ALS_LEVEL];

	atomic_t	als_cmd_val;		/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_cmd_val;		/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val;		/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_high_def;	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_thd_val_low_def;	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_3cm_delta;	/*the cmd value can't be read, stored in ram*/
	atomic_t	ps_5cm_delta;	/*the cmd value can't be read, stored in ram*/
	u32	ps_thd_val_high;
	u32	ps_thd_val_low;
	u32 ps_power_on_max_ct;
	ulong	enable;			/*enable mask*/
	ulong	pending_intr;		/*pending interrupt*/

#if defined(CONFIG_HAS_EARLYSUSPEND)
    struct early_suspend    early_drv;	/*early suspend*/
#endif
	struct mutex	ps_cal_mutex;
};

 struct PS_CALI_DATA_STRUCT
{
	int crosstalk;
	int close;
	int far_away;
	int valid;
} ;
static struct PS_CALI_DATA_STRUCT ps_cali={0, 0, 0, 0};

static int intr_flag_value = 0;

static int als_gainrange;

static int final_prox_val;
static int final_lux_val;
static int previous_lux_val;

static DEFINE_MUTEX(ltr559_mutex);

static struct ltr559_priv	*ltr559_obj = NULL;
static struct i2c_client	*ltr559_i2c_client = NULL;
struct alsps_hw		alsps_cust;
static struct alsps_hw	*hw = &alsps_cust;
struct platform_device *alspsPltFmDev;

static int  ltr559_local_init(void);
static int ltr559_remove(void);

static struct alsps_init_info ltr559_init_info = {
	.name = "ltr559",
	.init = ltr559_local_init,
	.uninit = ltr559_remove,
};

struct alsps_hw *get_cust_alsps(void)
{
	return &alsps_cust;
}

static int ltr559_i2c_read_reg(u8 regnum)
{
	u8 buffer[1], reg_value[1];
	int res = 0;

	buffer[0]= regnum;
	res = i2c_master_send(ltr559_obj->client, buffer, 0x1);
	if(res <= 0)
	{
		APS_ERR("read reg send res = %d\n",res);
		return res;
	}
	
	res = i2c_master_recv(ltr559_obj->client, reg_value, 0x1);
	if(res <= 0)
	{
		APS_ERR("read reg recv res = %d\n",res);
		return res;
	}
	
	return reg_value[0];
}

static int8_t I2C_Read(uint8_t *rxData, uint8_t length)
{
	int8_t index;
	struct i2c_msg data[] = {
		{
			.addr = ltr559_obj->client->addr,
			.flags = 0,
			.len = 1,
			.buf = rxData,
		},
		{
			.addr = ltr559_obj->client->addr,
			.flags = I2C_M_RD,
			.len = length,
			.buf = rxData,
		},
	};

	for (index = 0; index < 5; index++) {
		if (i2c_transfer(ltr559_obj->client->adapter, data, 2) > 0)
			break;

		mdelay(10);
	}

	if (index >= 5) {
		//pr_alert("%s I2C Read Fail !!!!\n", __func__);
		return -EIO;
	}

	return 0;
}

static int ltr559_i2c_write_reg(u8 regnum, u8 value)
{
	u8 databuf[2];    
	int res = 0;

	databuf[0] = regnum;   
	databuf[1] = value;
	res = i2c_master_send(ltr559_obj->client, databuf, 0x2);

	if (res < 0)
	{
		APS_ERR("wirte reg send res = %d\n",res);
		return res;
	}
	else
		return 0;
}

static int ltr559_ps_read(struct i2c_client *client, u16 *data)
{
	int psval_lo, psval_hi, psdata;

	psval_lo = ltr559_i2c_read_reg(LTR559_PS_DATA_0);
	if (psval_lo < 0){ 
		APS_DBG("psval_lo error\n");
		psdata = psval_lo;
		goto out;
	}
	psval_hi = ltr559_i2c_read_reg(LTR559_PS_DATA_1);

	if (psval_hi < 0){
	    APS_DBG("psval_hi error\n");
		psdata = psval_hi;
		goto out;
	}
	
	psdata = ((psval_hi & 7)* 256) + psval_lo;

	*data = psdata;
	return 0;
    
out:
	final_prox_val = psdata;
	
	return psdata;
}

static int ltr559_ps_set_thres(void)
{
	int res;
	u8 databuf[2];
	struct i2c_client *client = ltr559_obj->client;
	struct ltr559_priv *obj = ltr559_obj;	

	databuf[0] = LTR559_PS_THRES_LOW_0; 
	databuf[1] = (u8)((obj->ps_thd_val_low) & 0x00FF);
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return ltr559_ERR_I2C;
	}
	databuf[0] = LTR559_PS_THRES_LOW_1; 
	databuf[1] = (u8)((obj->ps_thd_val_low >> 8) & 0x00FF);
	
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return ltr559_ERR_I2C;
	}
	databuf[0] = LTR559_PS_THRES_UP_0;	
	databuf[1] = (u8)((obj->ps_thd_val_high) & 0x00FF);
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return ltr559_ERR_I2C;
	}
	databuf[0] = LTR559_PS_THRES_UP_1;	
	databuf[1] = (u8)((obj->ps_thd_val_high >> 8) & 0x00FF);
	res = i2c_master_send(client, databuf, 0x2);
	if(res <= 0)
	{
		goto EXIT_ERR;
		return ltr559_ERR_I2C;
	}

	APS_DBG("set ps low: %d high: %d\n", obj->ps_thd_val_low, obj->ps_thd_val_high);

	res = 0;
	return res;
	
EXIT_ERR:
	APS_ERR("set thres: %d\n", res);
	return res;
}

/* lijiangshuo add for ps calibration 20160104 start */
/* Note: only get the calibration crosstalk, without setting registers */
static u16 ltr559_ps_get_calibration_crosstalk(void)
{
	int cnt = 0, ret = -1;
	u16 ps_raw_data = 0, ps_sum = 0, ps_avg = 0;
	struct i2c_client *client = ltr559_obj->client;
	u8 regdata1 = 0, regdata2 = 0, regdata = 0;	

	regdata2 = ltr559_i2c_read_reg(LTR559_PS_MEAS_RATE);
	ret  = ltr559_i2c_write_reg(LTR559_PS_MEAS_RATE, 0x0F);
	if(ret < 0)
	{
		APS_ERR("PS: ps_calibration set ps_measure_rate failed!\n");
		return ret;
	}

	if(ltr559_obj->ps_enabled == 0)
	{
		regdata1 = ltr559_i2c_read_reg(LTR559_PS_CONTR);
		ret = ltr559_i2c_write_reg(LTR559_PS_CONTR, (regdata1|0x3));
		if(ret < 0)
		{
			APS_ERR("PS: enable ps err: %d\n", ret);
			return ret;
		}
		mdelay(WAKEUP_DELAY);
	} else
		disable_irq_nosync(ltr559_obj->irq);
	
	while(cnt < PS_CAL_NUM)
	{
		regdata = ltr559_i2c_read_reg(LTR559_ALS_PS_STATUS);
		if((regdata & 0x1) == 1) // ps new data
		{
			ret = ltr559_ps_read(client, &ps_raw_data);
			if(ret)
			{
				APS_ERR("ltr559_ps_read error %d\n", ret);
				return ret;
			}
			//APS_LOG("%s ps_raw_data[%d] = %d\n", __func__, cnt, ps_raw_data);
			if(ps_raw_data != 0)
			{
				ps_sum += ps_raw_data;
				cnt++;
			}
			else
				APS_ERR("%s ps_raw_data = 0! Unbelievable!!\n", __func__);		
		}
		mdelay(20);
	}

	ps_avg = ps_sum / cnt;
	if(ps_avg == 0)
		APS_ERR("%s ps_avg = 0! Unbelievable!!\n", __func__);

	ret = ltr559_i2c_write_reg(LTR559_PS_MEAS_RATE, regdata2);
	if(ret < 0)
	{
		APS_ERR("PS: ps_calibration set ps_meas_rate back failed!\n");
		return ret;
	}

	if(ltr559_obj->ps_enabled == 0)
	{
		ret = ltr559_i2c_write_reg(LTR559_PS_CONTR, regdata1);
		if(ret < 0)
		{
			APS_ERR("PS: disable ps err: %d\n", ret);
			return ret;
		}
		mdelay(WAKEUP_DELAY);
	} else
		enable_irq(ltr559_obj->irq);

	ps_cali.crosstalk = ps_avg;
	APS_LOG("%s crosstalk=%d\n", __func__, ps_cali.crosstalk);
	return ps_avg;
}

static void ltr559_ps_check_calibration_result(u16 ps_avg, u16 value, int type)
{
	int current_ct = ltr559_obj->ps_thd_val_high-atomic_read(&ltr559_obj->ps_3cm_delta);;

	ps_cali.valid = 1;
	ps_cali.close = ps_avg + atomic_read(&ltr559_obj->ps_3cm_delta);
	ps_cali.far_away = ps_avg + atomic_read(&ltr559_obj->ps_5cm_delta);
	
	switch(type)
	{
	case PS_CAL_FACTORY: // factory cal absolutely no shelter, need to decide wether is out of range
		if((ps_avg >= 2047) || (ps_cali.close >= 2047))
		{
			APS_ERR("%s too big ps_cal_crosstalk %d. No factory calibration done!\n", __func__, ps_avg);
			ps_cali.valid = 2;
		}
		break;
	case PS_CAL_STARTUP: // startup cal, compare ps_avg with ps_factory_cal_value
		if(((value > 0)&&(ps_avg > value)&&((ps_avg - value) > 500)) 
			|| ((value == 0)&&(ps_avg > ltr559_obj->ps_power_on_max_ct)))
		{
			ps_cali.valid = 0;
			ps_cali.close = value + atomic_read(&ltr559_obj->ps_3cm_delta);
			ps_cali.far_away = value + atomic_read(&ltr559_obj->ps_5cm_delta);
			APS_LOG("%s ps_cal_startup failed! ps_avg %d exceed %d\n", __func__, ps_avg, value);
		}
		break;
	case PS_CAL_CALL: // call cal, compare ps_avg with current-used value
		if((ps_avg > current_ct)&&((ps_avg - current_ct) > 100))
		{
			ps_cali.valid = 0;
			ps_cali.close = ltr559_obj->ps_thd_val_high;
			ps_cali.far_away = ltr559_obj->ps_thd_val_low;
			APS_LOG("%s ps_avg %d exceed the start up cal value %d too much(%d)!\n",
				__func__, ps_avg, value, value);
		}
		break;
	default:
		APS_ERR("%s with wrong type %d\n!", __func__, type);
		ps_cali.valid = 0;
		break;
	}
}

static int ltr559_ps_calibration(int type, int value)
{
	u16 ps_avg = 0;

	mutex_lock(&ltr559_obj->ps_cal_mutex);

	ps_avg = ltr559_ps_get_calibration_crosstalk();
	ltr559_ps_check_calibration_result(ps_avg, value, type);

	mutex_unlock(&ltr559_obj->ps_cal_mutex);
	return 0;
}

static int ltr559_ps_threshold_setting(int type, int value[])
{
	int ret = 0;

	switch(type)
	{
	case PS_THRESHOLD_SET:
		if(ps_cali.valid == 1)
		{
			ltr559_obj->ps_thd_val_high = ps_cali.close;
			ltr559_obj->ps_thd_val_low = ps_cali.far_away;
		}
		ret = ltr559_ps_set_thres();
		if(ret != 0)
			APS_ERR("%s ps_set_thres_reg failed!\n", __func__);
		break;
	case PS_CAL_RESULT_GET:
		value[0] = ps_cali.crosstalk;
		value[1] = ps_cali.valid;
		value[2] = ps_cali.close;
		value[3] = ps_cali.far_away;
		break;
	default:
		APS_ERR("%s with wrong type %d\n!", __func__, type);
		break;
	}

	return ret;
}

#if 0
static int ltr559_ps_power_on_calibration(void)
{
	u16 ct;

	ct = ltr559_ps_get_calibration_crosstalk();
	ltr559_ps_check_calibration_result(ct, ltr559_obj->ps_power_on_max_ct, PS_CAL_STARTUP);

	APS_LOG("%s ps_avg=%d cali_valid=%d high_threshold=%d low_threshold=%d\n", 
		__func__, ps_cali.crosstalk, ps_cali.valid, ps_cali.close, ps_cali.far_away);
	printk("ljs ps_avg=%d\n", ps_cali.crosstalk);
	ltr559_ps_threshold_setting(PS_THRESHOLD_SET, NULL);
	return 0;
}
#endif
/* lijiangshuo add for ps calibration 20160104 end */

static int ltr559_ps_enable(struct i2c_client *client, int enable)
{
	u8 regdata;	
	int err;

	if(ltr559_obj->ps_enabled == enable)
	{
		APS_LOG("%s ps_enabled already %d\n", __func__, enable);
		return 0;
	}
	
	APS_LOG("%s enable=%d\n", __func__, enable);

	regdata = ltr559_i2c_read_reg(LTR559_PS_CONTR);

	if (enable == 1) {
		regdata |= 0x03;
	} else {
		regdata &= 0xfc;
	}

	if(enable == 1)
	{
		err = ltr559_i2c_write_reg(LTR559_PS_MEAS_RATE, 0x02);
		if(err < 0)
		{
			APS_ERR("PS: ps_enable ps_measure_rate set failed!\n");
			return err;
		}

		err = ltr559_i2c_write_reg(LTR559_INTERRUPT, 0x1);
		if(err < 0)
		{
			APS_ERR("PS: ps_enable interrupt set err %d\n", err);
			return err;
		}
	}

	err = ltr559_i2c_write_reg(LTR559_PS_CONTR, regdata);
	if(err<0)
	{
		APS_ERR("PS: enable ps err: %d en: %d \n", err, enable);
		return err;
	}
	mdelay(WAKEUP_DELAY);

	if(enable)
	{
		enable_irq(ltr559_obj->irq);
		ltr559_obj->ps_enabled = 1;
	}
	else
	{
		disable_irq_nosync(ltr559_obj->irq);
		ltr559_obj->ps_enabled = 0;
	}
	
	return 0;
}

static int ltr559_als_enable(struct i2c_client *client, int enable)
{
	int err = 0;	
	u8 regdata = 0;	

	if(ltr559_obj->als_enabled == enable)
	{
		APS_LOG("%s als_enabled already %d\n", __func__, enable);
		return 0;
	}

	APS_LOG("%s enable=%d\n", __func__, enable);

	regdata = ltr559_i2c_read_reg(LTR559_ALS_CONTR);
	if (enable == 1) {
		regdata |= 0x01;
	} else {
		regdata &= 0xfe;
	}

	err = ltr559_i2c_write_reg(LTR559_ALS_CONTR, regdata);
	if(err < 0)
	{
		APS_ERR("ALS: enable als err: %d en: %d \n", err, enable);
		return err;
	}
	
	mdelay(WAKEUP_DELAY);

	if(enable)
		ltr559_obj->als_enabled = 1;
	else
		ltr559_obj->als_enabled = 0;
	
	return 0;
}

static int ltr559_als_read(struct i2c_client *client, u16* data)
{
	int alsval_ch0_lo, alsval_ch0_hi, alsval_ch0;
	int alsval_ch1_lo, alsval_ch1_hi, alsval_ch1;
	int  luxdata_int = 0;
	int ratio = 1;
	uint8_t buffer[4];

#if 0
	alsval_ch1_lo = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH1_0);
	alsval_ch1_hi = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH1_1);
	alsval_ch1 = (alsval_ch1_hi * 256) + alsval_ch1_lo;

	alsval_ch0_lo = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH0_0);
	alsval_ch0_hi = ltr559_i2c_read_reg(LTR559_ALS_DATA_CH0_1);
	alsval_ch0 = (alsval_ch0_hi * 256) + alsval_ch0_lo;
#else
	buffer[0] = LTR559_ALS_DATA_CH1_0;
	(void)I2C_Read(buffer, 4);

	alsval_ch1_lo = buffer[0];
	alsval_ch1_hi = buffer[1];
	alsval_ch1 = (alsval_ch1_hi * 256) + alsval_ch1_lo;
	
	alsval_ch0_lo = buffer[2];
	alsval_ch0_hi = buffer[3];
    alsval_ch0 = (alsval_ch0_hi * 256) + alsval_ch0_lo;
#endif

	if((alsval_ch1==0)||(alsval_ch0==0))
		ratio = 0;
	else
	{
		ratio = (alsval_ch1*100) /(alsval_ch0+alsval_ch1);

		if(ratio == 0)
			luxdata_int = previous_lux_val;
		else if (ratio < 45)
			luxdata_int = (((17743 * alsval_ch0)+(11059 * alsval_ch1))/als_gainrange)/1000;
		else if ((ratio < 64) && (ratio >= 45))
			luxdata_int = (((42785 * alsval_ch0)-(19548 * alsval_ch1))/als_gainrange)/1000;
		else if ((ratio < 99) && (ratio >= 64))
			luxdata_int = (((5926 * alsval_ch0)+(1185 * alsval_ch1))/als_gainrange)/1000;
		else
			//luxdata_int = 0;
			luxdata_int = previous_lux_val;
	}
	
	luxdata_int = luxdata_int*2/3; // lijiangshuo modify for P637F10_EU 20161107
	*data = luxdata_int;
	final_lux_val = luxdata_int;
	previous_lux_val = final_lux_val;
	return 0;
}

static ssize_t ltr559_show_als_enable(struct device_driver *ddri, char *buf)
{
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", ltr559_obj->als_enabled);     
}

static ssize_t ltr559_store_als_enable(struct device_driver *ddri, const char *buf, size_t count)
{
	int enable, ret;
	
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d ", &enable))
		ret = ltr559_als_enable(ltr559_obj->client, enable);
	else
		APS_DBG("invalid content: %s, length = %zu\n", buf, count);

	return count;    
}

static ssize_t ltr559_show_ps_enable(struct device_driver *ddri, char *buf)
{
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}

	return snprintf(buf, PAGE_SIZE, "%d\n", ltr559_obj->ps_enabled);     
}

static ssize_t ltr559_store_ps_enable(struct device_driver *ddri, const char *buf, size_t count)
{
	int enable, ret;
	
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d ", &enable))
		ret = ltr559_ps_enable(ltr559_obj->client, enable);
	else
		APS_DBG("invalid content: %s, length = %zu\n", buf, count);

	return count;    
}

static ssize_t ltr559_show_als(struct device_driver *ddri, char *buf)
{
	int res;
	
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	res = ltr559_als_read(ltr559_obj->client, &ltr559_obj->als);
	return snprintf(buf, PAGE_SIZE, "%d\n", ltr559_obj->als);
}

static ssize_t ltr559_show_ps(struct device_driver *ddri, char *buf)
{
	int  res;
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	res = ltr559_ps_read(ltr559_obj->client, &ltr559_obj->ps);
	return snprintf(buf, PAGE_SIZE, "%d\n", ltr559_obj->ps);
}

static ssize_t ltr559_show_ps_threshold(struct device_driver *ddri, char *buf)
{
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	
	return snprintf(buf, PAGE_SIZE, "high=%d low=%d\n", ltr559_obj->ps_thd_val_high, ltr559_obj->ps_thd_val_low);     
}

static ssize_t ltr559_show_status(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;
	
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	
	if(ltr559_obj->hw)
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d, (%d %d)\n", 
			ltr559_obj->hw->i2c_num, ltr559_obj->hw->power_id, ltr559_obj->hw->power_vol);
	else
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");

	len += snprintf(buf+len, PAGE_SIZE-len, "MISC: %d %d\n", atomic_read(&ltr559_obj->als_suspend), atomic_read(&ltr559_obj->ps_suspend));
	return len;
}

static ssize_t ltr559_store_status(struct device_driver *ddri, const char *buf, size_t count)
{
	int status1, ret;
	
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "%d ", &status1))
		ret = ltr559_ps_enable(ltr559_obj->client, status1);
	else
		APS_DBG("invalid content: %s, length = %zu\n", buf, count);

	return count;    
}

static ssize_t ltr559_show_reg(struct device_driver *ddri, char *buf)
{
	int i,len = 0;
	int reg[] = {0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8a,0x8b,0x8c,
		0x8d,0x8e,0x8f,0x90,0x91,0x92,0x93,0x94,0x95,0x97,0x98,0x99,0x9a,0x9e};
	for(i=0;i<27;i++)
		len += snprintf(buf+len, PAGE_SIZE-len, "reg:0x%04X value: 0x%04X\n", reg[i],ltr559_i2c_read_reg(reg[i]));	

	return len;
}

static ssize_t ltr559_store_reg(struct device_driver *ddri, const char *buf, size_t count)
{
	int ret, value;
	u8 reg;
	
	if(!ltr559_obj)
	{
		APS_ERR("ltr559_obj is null!!\n");
		return 0;
	}
	
	if(2 == sscanf(buf, "%hhx %x ", &reg, &value))
	{
		APS_DBG("before write reg: %x, reg_value = %x  write value=%x\n", reg,ltr559_i2c_read_reg(reg),value);
		ret=ltr559_i2c_write_reg(reg,value);
		APS_DBG("after write reg: %x, reg_value = %x\n", reg,ltr559_i2c_read_reg(reg));
	}
	else
		APS_DBG("invalid content: '%s', length = %zu\n", buf, count);

	return count;    
}

static DRIVER_ATTR(als_enable, 0664, ltr559_show_als_enable, ltr559_store_als_enable);
static DRIVER_ATTR(ps_enable, 0664, ltr559_show_ps_enable, ltr559_store_ps_enable);
static DRIVER_ATTR(als, 0664, ltr559_show_als, NULL);
static DRIVER_ATTR(ps, 0664, ltr559_show_ps, NULL);
static DRIVER_ATTR(ps_threshold, 0664, ltr559_show_ps_threshold, NULL);
static DRIVER_ATTR(status, 0664, ltr559_show_status, ltr559_store_status);
static DRIVER_ATTR(reg, 0664, ltr559_show_reg, ltr559_store_reg);

static struct driver_attribute *ltr559_attr_list[] = {
	&driver_attr_als_enable,
	&driver_attr_ps_enable,
	&driver_attr_als,
	&driver_attr_ps,    
	&driver_attr_ps_threshold,
	&driver_attr_status,
	&driver_attr_reg,
};

static int ltr559_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(ltr559_attr_list)/sizeof(ltr559_attr_list[0]));

	if (driver == NULL)
		return -EINVAL;

	for(idx = 0; idx < num; idx++)
	{
		err = driver_create_file(driver, ltr559_attr_list[idx]);
		if(err)
		{            
			APS_ERR("driver_create_file (%s) = %d\n", ltr559_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}

static int ltr559_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(ltr559_attr_list)/sizeof(ltr559_attr_list[0]));

	if (!driver)
		return -EINVAL;

	for (idx = 0; idx < num; idx++) 
		driver_remove_file(driver, ltr559_attr_list[idx]);
	
	return err;
}

void ltr559_eint_func(void)
{
	struct ltr559_priv *obj = ltr559_obj;

	if(!obj)
		return;

	schedule_work(&obj->eint_work);
}

#if defined(CONFIG_OF)
static irqreturn_t ltr559_eint_handler(int irq, void *desc)
{
	disable_irq_nosync(ltr559_obj->irq);
	ltr559_eint_func();
	return IRQ_HANDLED;
}
#endif

int ltr559_setup_eint(struct i2c_client *client)
{
	int ret;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_cfg;
	u32 ints[2] = {0, 0};

	alspsPltFmDev = get_alsps_platformdev();

	pinctrl = devm_pinctrl_get(&alspsPltFmDev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		APS_ERR("Cannot find alsps pinctrl!\n");
	}

	pins_cfg = pinctrl_lookup_state(pinctrl, "pin_cfg");
	if (IS_ERR(pins_cfg)) {
		ret = PTR_ERR(pins_cfg);
		APS_ERR("Cannot find alsps pinctrl pin_cfg!\n");
	}

	if (ltr559_obj->irq_node) {
		of_property_read_u32_array(ltr559_obj->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "p-sensor");
		gpio_set_debounce(ints[0], ints[1]);
		pinctrl_select_state(pinctrl, pins_cfg);

		ltr559_obj->irq = irq_of_parse_and_map(ltr559_obj->irq_node, 0);
		if (!ltr559_obj->irq) {
			APS_ERR("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}

//		if (request_irq(ltr559_obj->irq, ltr559_eint_handler, IRQF_TRIGGER_FALLING, "ALS-eint", NULL)) {
// lijiangshuo modify for ps not work occasionally 20160201
		if (request_irq(ltr559_obj->irq, ltr559_eint_handler, IRQF_TRIGGER_LOW, "ALS-eint", NULL)) {
			APS_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		disable_irq_nosync(ltr559_obj->irq);
	} else {
		APS_ERR("null irq node!!\n");
		return -EINVAL;
	}
	
    return 0;
}

static void ltr559_power(struct alsps_hw *hw, unsigned int on) 
{
	return;
}

static int ltr559_check_and_clear_intr(struct i2c_client *client) 
{
	int res, intp, intl;
	u8 buffer[2];	
	u8 temp;

	buffer[0] = LTR559_ALS_PS_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
		goto EXIT_ERR;

	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
		goto EXIT_ERR;

	temp = buffer[0];
	res = 1;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x02))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x08))
	{
		res = 0;
		intl = 1;		
	}

	if(0 == res)
	{
		if((1 == intp) && (0 == intl))
			buffer[1] = buffer[0] & 0xfD;
		else if((0 == intp) && (1 == intl))
			buffer[1] = buffer[0] & 0xf7;
		else
		{
			APS_LOG("Check ALS/PS interrup error\n");
			buffer[1] = buffer[0] & 0xf5;
		}
	}

	return res;

EXIT_ERR:
	APS_ERR("ltr559_check_and_clear_intr fail\n");
	return 1;
}

static int ltr559_check_intr(struct i2c_client *client) 
{
	int res, intp, intl;
	u8 buffer[2];

	buffer[0] = LTR559_ALS_PS_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
		goto EXIT_ERR;

	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
		goto EXIT_ERR;

	res = 1;
	intp = 0;
	intl = 0;
	if(0 != (buffer[0] & 0x02))
	{
		res = 0;
		intp = 1;
	}
	if(0 != (buffer[0] & 0x08))
	{
		res = 0;
		intl = 1;		
	}

	if(0 == res)
	{
		if((1 == intp) && (0 == intl))
			buffer[1] = buffer[0] & 0xfD;
		else if((0 == intp) && (1 == intl))
			buffer[1] = buffer[0] & 0xf7;
		else
		{
			APS_LOG("Check ALS/PS interrup error\n");
			buffer[1] = buffer[0] & 0xf5;
		}
	}

	return res;

EXIT_ERR:
	APS_ERR("ltr559_check_intr fail\n");
	return 1;
}

static int ltr559_clear_intr(struct i2c_client *client) 
{
	int res;
	u8 buffer[2];
	
	buffer[0] = LTR559_ALS_PS_STATUS;
	res = i2c_master_send(client, buffer, 0x1);
	if(res <= 0)
		goto EXIT_ERR;

	res = i2c_master_recv(client, buffer, 0x1);
	if(res <= 0)
		goto EXIT_ERR;

	buffer[1] = buffer[0] & 0x01;
	buffer[0] = LTR559_ALS_PS_STATUS	;

	res = i2c_master_send(client, buffer, 0x2);
	if(res <= 0)
		goto EXIT_ERR;
	else
		res = 0;

	return res;

EXIT_ERR:
	APS_ERR("ltr559_check_intr fail\n");
	return 1;
}

static int ltr559_devinit(void)
{
	int res;
	int init_ps_gain;
	int init_als_gain;
	u8 databuf[2];	
	struct i2c_client *client = ltr559_obj->client;
	struct ltr559_priv *obj = ltr559_obj;   
	
	mdelay(PON_DELAY);

	init_ps_gain = MODE_PS_Gain16;
	res = ltr559_i2c_write_reg(LTR559_PS_CONTR, init_ps_gain); 
	if(res < 0)
	{
		APS_LOG("ltr559 set ps gain error\n");
		return res;
	}
	
	mdelay(WAKEUP_DELAY);
    
	res = ltr559_i2c_write_reg(LTR559_PS_N_PULSES, 4); 
	if(res < 0)
	{
		APS_LOG("ltr559 set ps pulse error\n");
		return res;
	} 

	als_gainrange = ALS_RANGE_8K;
	init_als_gain = als_gainrange;
	switch (init_als_gain)
	{
		case ALS_RANGE_64K:
			res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range1);
			break;
		case ALS_RANGE_32K:
			res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range2);
			break;
		case ALS_RANGE_16K:
			res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range3);
			break;
		case ALS_RANGE_8K:
			res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range4);
			break;
		case ALS_RANGE_1300:
			res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range5);
			break;
		case ALS_RANGE_600:
			res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range6);
			break;
		default:
			res = ltr559_i2c_write_reg(LTR559_ALS_CONTR, MODE_ALS_Range1);			
			APS_ERR("proxmy sensor gainrange %d!\n", init_als_gain);
			break;
	}
 
	// LTR559_ALS_MEAS_RATE set 0x01
	res = ltr559_i2c_write_reg(LTR559_ALS_MEAS_RATE, 0x01);
	if(res < 0)
	{
	    APS_LOG("ltr559 set als rate error1\n");
	    return res;
	}
	
	/*for interrup work mode support */
	if(0 == obj->hw->polling_mode_ps)
	{	
		ltr559_ps_set_thres();	
		databuf[0] = LTR559_INTERRUPT;	
		databuf[1] = 0x01;
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}

		databuf[0] = LTR559_INTERRUPT_PERSIST;	
//		databuf[1] = 0x00;
		databuf[1] = 0x10; // lijiangshuo modify for ps not work occasionally 20160201
		res = i2c_master_send(client, databuf, 0x2);
		if(res <= 0)
		{
			goto EXIT_ERR;
			return ltr559_ERR_I2C;
		}
	}

	if((res = ltr559_setup_eint(client))!=0)
	{
		APS_ERR("setup eint: %d\n", res);
		return res;
	}
	
	if((res = ltr559_check_and_clear_intr(client)))
		APS_ERR("check/clear intr: %d\n", res);

	res = 0;

EXIT_ERR:
	APS_ERR("init dev: %d\n", res);
	return res;
}

static int ltr559_get_als_value(struct ltr559_priv *obj, u16 als)
{
	int idx;
	int invalid = 0;

	return als;
	
	for(idx = 0; idx < obj->als_level_num; idx++)
	{
		if(als <= obj->hw->als_level[idx])
			break;
	}
	
	if(idx >= obj->als_value_num)
	{
		APS_ERR("exceed range\n"); 
		idx = obj->als_value_num - 1;
	}
	
	if(1 == atomic_read(&obj->als_deb_on))
	{
		unsigned long endt = atomic_read(&obj->als_deb_end);
		if(time_after(jiffies, endt))
			atomic_set(&obj->als_deb_on, 0);
		
		if(1 == atomic_read(&obj->als_deb_on))
			invalid = 1;
	}

	if(!invalid)
	{
//		APS_DBG("ALS: %05d => %05d\n", als, obj->hw->als_value[idx]);	
		return obj->hw->als_value[idx];
	}
	else
	{
		APS_ERR("ALS: %05d => %05d (-1)\n", als, obj->hw->als_value[idx]);    
		return -1;
	}
}

static int ltr559_get_ps_value(struct ltr559_priv *obj, u16 ps)
{
	int val, invalid = 0;
	static int val_temp = 1;

	if(ps > obj->ps_thd_val_high)
	{
		val = 0; /* close*/
		val_temp = 0;
		intr_flag_value = 1;
	}
	else if(ps < obj->ps_thd_val_low)
	{
		val = 1; /*far away*/
		val_temp = 1;
		intr_flag_value = 0;
	}
	else
		val = val_temp;	
			
	
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
	else if (obj->als > 50000)
	{
		//invalid = 1;
		APS_DBG("ligh too high will result to failt proximiy\n");
		return 1;  /*far away*/
	}

	if(!invalid)
	{
//		APS_DBG("PS:  %05d => %05d\n", ps, val);
		return val;
	}	
	else
		return -1;
}

static int als_open_report_data(int open)
{
	return 0;
}

static int als_enable_nodata(int en)
{
	int res = 0;
	APS_LOG("%s enable=%d\n", __func__, en);

	mutex_lock(&ltr559_mutex);
	if (en)
		set_bit(CMC_BIT_ALS, &ltr559_obj->enable);
	else
		clear_bit(CMC_BIT_ALS, &ltr559_obj->enable);
	mutex_unlock(&ltr559_mutex);
	
	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return -1;
	}
	res = ltr559_als_enable(ltr559_obj->client, en);
	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}
	return 0;
}

static int als_set_delay(u64 ns)
{
	return 0;
}

static int als_get_data(int *value, int *status)
{
	int err = 0;
	struct ltr559_priv *obj = NULL;

	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return -1;
	}
	obj = ltr559_obj;
	
	err = ltr559_als_read(obj->client, &obj->als);
	if (err)
		err = -1;
	else {
		*value = ltr559_get_als_value(obj, obj->als);
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return err;
}

static int ps_open_report_data(int open)
{
	return 0;
}

static int ps_enable_nodata(int en)
{
	int res = 0;
	APS_LOG("%s enable=%d\n", __func__, en);

	mutex_lock(&ltr559_mutex);
	if (en)
		set_bit(CMC_BIT_PS, &ltr559_obj->enable);
	else
		clear_bit(CMC_BIT_PS, &ltr559_obj->enable);
	mutex_unlock(&ltr559_mutex);
	
	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return -1;
	}

	res = ltr559_ps_enable(ltr559_obj->client, en);
	if (res) {
		APS_ERR("als_enable_nodata is failed!!\n");
		return -1;
	}

	return 0;
}

static int ps_set_delay(u64 ns)
{
	return 0;
}

static int ps_get_data(int *value, int *status)
{
	int err = 0;

	if (!ltr559_obj) {
		APS_ERR("ltr559_obj is null!!\n");
		return -1;
	}

	err = ltr559_ps_read(ltr559_obj->client, &ltr559_obj->ps);
	if (err)
		err = -1;
	else {
		*value = ltr559_get_ps_value(ltr559_obj, ltr559_obj->ps);
		if (*value < 0)
			err = -1;
		*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	}

	return err;
}

static void ltr559_eint_work(struct work_struct *work)
{
	struct ltr559_priv *obj = (struct ltr559_priv *)container_of(work, struct ltr559_priv, eint_work);
	int err;
	u8 databuf[2];
	int res = 0;
	int value = 1;
	
	err = ltr559_check_intr(obj->client);
	if(err != 0)
		APS_ERR("ltr559_eint_work check intrs: %d\n", err);
	else
	{
		ltr559_ps_read(obj->client, &obj->ps);
	    	if(obj->ps < 0)
	    	{
	    		err = -1;
	    		goto err;
	    	}
				
		value = ltr559_get_ps_value(obj, obj->ps);
		if(intr_flag_value){
			printk("%s ps_data(%d) > ps_threhold_high(%d), report NEAR!\n", __func__, obj->ps, obj->ps_thd_val_high);
			databuf[0] = LTR559_PS_THRES_LOW_0;	
			databuf[1] = (u8)((obj->ps_thd_val_low) & 0x00FF);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
				goto err;

			databuf[0] = LTR559_PS_THRES_LOW_1;	
			databuf[1] = (u8)(((obj->ps_thd_val_low) & 0xFF00) >> 8);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
				goto err;

			databuf[0] = LTR559_PS_THRES_UP_0;	
			databuf[1] = (u8)(0x00FF);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
				goto err;

			databuf[0] = LTR559_PS_THRES_UP_1; 
			databuf[1] = (u8)((0xFF00) >> 8);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
				goto err;
		}
		else{	
			printk("%s ps_data(%d) < ps_threhold_low(%d), report FAR!\n", __func__, obj->ps, obj->ps_thd_val_low);
			databuf[0] = LTR559_PS_THRES_LOW_0;	
			databuf[1] = (u8)(0 & 0x00FF);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
				goto err;

			databuf[0] = LTR559_PS_THRES_LOW_1;	
			databuf[1] = (u8)((0 & 0xFF00) >> 8);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
				goto err;

			databuf[0] = LTR559_PS_THRES_UP_0;	
			databuf[1] = (u8)((obj->ps_thd_val_high) & 0x00FF);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
				goto err;

			databuf[0] = LTR559_PS_THRES_UP_1; 
			databuf[1] = (u8)(((obj->ps_thd_val_high) & 0xFF00) >> 8);
			res = i2c_master_send(obj->client, databuf, 0x2);
			if(res <= 0)
				goto err;
		}
		ps_report_interrupt_data(value);
	}
	
	ltr559_clear_intr(obj->client);
err:
	enable_irq(ltr559_obj->irq);
	return;
}

static int ltr559_open(struct inode *inode, struct file *file)
{
	file->private_data = ltr559_i2c_client;

	if (!file->private_data)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	return nonseekable_open(inode, file);
}

static int ltr559_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static long ltr559_unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)       
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct ltr559_priv *obj = i2c_get_clientdata(client);  
	int err = 0;
	void __user *ptr = (void __user*) arg;
	int dat;
	uint32_t enable;
	APS_DBG("cmd= %d\n", cmd);
	
	switch (cmd)
	{
	case ALSPS_SET_PS_MODE:
		if(copy_from_user(&enable, ptr, sizeof(enable)))
		{
			err = -EFAULT;
			goto err_out;
		}
		err = ltr559_ps_enable(obj->client, enable);
		if(err < 0)
		{
			APS_ERR("enable ps fail: %d en: %d\n", err, enable); 
			goto err_out;
		}
		set_bit(CMC_BIT_PS, &obj->enable);
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
//		APS_DBG("ALSPS_GET_PS_DATA\n"); 
		obj->ps = ltr559_ps_read(obj->client, &obj->ps);
		if(obj->ps < 0)
			goto err_out;

		dat = ltr559_get_ps_value(obj, obj->ps);
		if(copy_to_user(ptr, &dat, sizeof(dat)))
		{
			err = -EFAULT;
			goto err_out;
		}  
		break;
	case ALSPS_GET_PS_RAW_DATA:    
		obj->ps = ltr559_ps_read(obj->client, &obj->ps);
		if(obj->ps < 0)
			goto err_out;

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
		err = ltr559_als_enable(obj->client, enable);
		if(err < 0)
		{
			APS_ERR("enable als fail: %d en: %d\n", err, enable); 
			goto err_out;
		}
		set_bit(CMC_BIT_ALS, &obj->enable);
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
		err = ltr559_als_read(obj->client, &obj->als);
		if(err < 0)
			goto err_out;

		dat = ltr559_get_als_value(obj, obj->als);
		if(copy_to_user(ptr, &dat, sizeof(dat)))
		{
			err = -EFAULT;
			goto err_out;
		}              
		break;
	case ALSPS_GET_ALS_RAW_DATA:    
		obj->als = ltr559_als_read(obj->client, &obj->als);
		if(obj->als < 0)
			goto err_out;

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

static struct file_operations ltr559_fops = {
	.open	= ltr559_open,
	.release	= ltr559_release,
	.unlocked_ioctl = ltr559_unlocked_ioctl,
};

static struct miscdevice ltr559_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als_ps",
	.fops = &ltr559_fops,
};

static int ltr559_i2c_suspend(struct i2c_client *client, pm_message_t msg) 
{
#if 0
	struct ltr559_priv *obj = i2c_get_clientdata(client);    
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
		err = ltr559_als_enable(obj->client, 0);
		if(err < 0)
		{
			APS_ERR("disable als: %d\n", err);
			return err;
		}

		atomic_set(&obj->ps_suspend, 1);
		err = ltr559_ps_enable(obj->client, 0);
		if(err < 0)
		{
			APS_ERR("disable ps:  %d\n", err);
			return err;
		}
		
		ltr559_power(obj->hw, 0);
	}
#endif
	return 0;
}

static int ltr559_i2c_resume(struct i2c_client *client)
{
#if 0
	struct ltr559_priv *obj = i2c_get_clientdata(client);        
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return -EINVAL;
	}

	ltr559_power(obj->hw, 1);
	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
	    err = ltr559_als_enable(obj->client, 1);
	    if (err < 0)
		{
			APS_ERR("enable als fail: %d\n", err);        
		}
	}
	atomic_set(&obj->ps_suspend, 0);
	if(test_bit(CMC_BIT_PS,  &obj->enable))
	{
		err = ltr559_ps_enable(obj->client, 1);
	    if (err < 0)
		{
			APS_ERR("enable ps fail: %d\n", err);                
		}
	}
#endif
	return 0;
}

/*early_suspend is only applied for ALS*/
#if defined(CONFIG_HAS_EARLYSUSPEND)
static void ltr559_early_suspend(struct early_suspend *h) 
{
#if 0
	struct ltr559_priv *obj = container_of(h, struct ltr559_priv, early_drv);   
	int err;
	APS_FUN();    

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}
	
	atomic_set(&obj->als_suspend, 1); 
	err = ltr559_als_enable(obj->client, 0);
	if(err < 0)
	{
		APS_ERR("disable als fail: %d\n", err); 
	}
#endif
}

static void ltr559_late_resume(struct early_suspend *h)
{
#if 0
	struct ltr559_priv *obj = container_of(h, struct ltr559_priv, early_drv);         
	int err;
	APS_FUN();

	if(!obj)
	{
		APS_ERR("null pointer!!\n");
		return;
	}

	atomic_set(&obj->als_suspend, 0);
	if(test_bit(CMC_BIT_ALS, &obj->enable))
	{
	    err = ltr559_als_enable(obj->client, 1);
		if(err < 0)
		{
			APS_ERR("enable als fail: %d\n", err);        

		}
	}
#endif
}
#endif

static int ltr559_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
	strcpy(info->type, LTR559_DEV_NAME);
	return 0;
}

static int ltr559_init_flag =  -1;
static int ltr559_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ltr559_priv *obj;
	struct als_control_path als_ctl = {0};
	struct als_data_path als_data = {0};
	struct ps_control_path ps_ctl = {0};
	struct ps_data_path ps_data = {0};
	int err = 0;
	APS_FUN();
	
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(obj, 0, sizeof(*obj));
	ltr559_obj = obj;

	obj->hw = hw;

	INIT_WORK(&obj->eint_work, ltr559_eint_work);
	obj->client = client;
	i2c_set_clientdata(client, obj);
	obj->als_enabled = 0;
	obj->ps_enabled = 0;
	atomic_set(&obj->als_debounce, 300);
	atomic_set(&obj->als_deb_on, 0);
	atomic_set(&obj->als_deb_end, 0);
	atomic_set(&obj->ps_debounce, 300);
	atomic_set(&obj->ps_deb_on, 0);
	atomic_set(&obj->ps_deb_end, 0);
	atomic_set(&obj->ps_mask, 0);
	atomic_set(&obj->als_suspend, 0);
	atomic_set(&obj->ps_thd_val_high_def,  obj->hw->ps_threshold_high_def);
	atomic_set(&obj->ps_thd_val_low_def,  obj->hw->ps_threshold_low_def);
	atomic_set(&obj->ps_3cm_delta,  obj->hw->ps_3cm_delta);
	atomic_set(&obj->ps_5cm_delta,  obj->hw->ps_5cm_delta);
	atomic_set(&obj->ps_thd_val,  obj->hw->ps_threshold);
	mutex_init(&obj->ps_cal_mutex);
	obj->ps_thd_val_high = atomic_read(&obj->ps_thd_val_high_def);
	obj->ps_thd_val_low = atomic_read(&obj->ps_thd_val_low_def);
	obj->ps_power_on_max_ct = obj->hw->ps_power_on_max_ct;

	obj->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint");
	obj->enable = 0;
	obj->pending_intr = 0;
	obj->als_level_num = sizeof(obj->hw->als_level)/sizeof(obj->hw->als_level[0]);
	obj->als_value_num = sizeof(obj->hw->als_value)/sizeof(obj->hw->als_value[0]);   

	BUG_ON(sizeof(obj->als_level) != sizeof(obj->hw->als_level));
	memcpy(obj->als_level, obj->hw->als_level, sizeof(obj->als_level));
	BUG_ON(sizeof(obj->als_value) != sizeof(obj->hw->als_value));
	memcpy(obj->als_value, obj->hw->als_value, sizeof(obj->als_value));
	set_bit(CMC_BIT_ALS, &obj->enable);
	set_bit(CMC_BIT_PS, &obj->enable);

	ltr559_i2c_client = client;
	err = ltr559_devinit();
	if(err)
		goto exit_init_failed;

	err  = misc_register(&ltr559_device);
	if(err)
	{
		APS_ERR("ltr559_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	
	/* Register sysfs attribute */
	err = ltr559_create_attr(&(ltr559_init_info.platform_diver_addr->driver));
	if(err)
	{
		printk(KERN_ERR "create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}

	als_ctl.open_report_data = als_open_report_data;
	als_ctl.enable_nodata = als_enable_nodata;
	als_ctl.set_delay = als_set_delay;
	als_ctl.is_report_input_direct = false;
	als_ctl.is_support_batch = false;
	err = als_register_control_path(&als_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	als_data.get_data = als_get_data;
	als_data.vender_div = 100;
	err = als_register_data_path(&als_data);
	if (err) {
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_ctl.open_report_data = ps_open_report_data;
	ps_ctl.enable_nodata = ps_enable_nodata;
	ps_ctl.set_delay  = ps_set_delay;
/* lijiangshuo add for ps_calibration 20160104 start */
	ps_ctl.ps_calibration = ltr559_ps_calibration;//ltr559_ps_get_calibration_crosstalk;
	ps_ctl.ps_threshold_setting = ltr559_ps_threshold_setting;//ltr559_ps_set_calibration_threshold;
/* lijiangshuo add for ps_calibration 20160104 end */
	ps_ctl.is_report_input_direct = false;
	ps_ctl.is_support_batch = false;
	err = ps_register_control_path(&ps_ctl);
	if (err) {
		APS_ERR("register fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	ps_data.get_data = ps_get_data;
	ps_data.vender_div = 100;
	err = ps_register_data_path(&ps_data);
	if (err) {
		APS_ERR("tregister fail = %d\n", err);
		goto exit_sensor_obj_attach_fail;
	}

	err = batch_register_support_info(ID_LIGHT, als_ctl.is_support_batch, 1, 0);
	if (err)
		APS_ERR("register light batch support err = %d\n", err);

	err = batch_register_support_info(ID_PROXIMITY, ps_ctl.is_support_batch, 1, 0);
	if (err)
		APS_ERR("register proximity batch support err = %d\n", err);

#if defined(CONFIG_HAS_EARLYSUSPEND)
	obj->early_drv.level	= EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend	= ltr559_early_suspend,
	obj->early_drv.resume	= ltr559_late_resume,	 
	register_early_suspend(&obj->early_drv);
#endif

	ltr559_init_flag = 0;

	//ltr559_ps_power_on_calibration();

	APS_LOG("%s: OK\n", __func__);
	return 0;

exit_sensor_obj_attach_fail:
	ltr559_delete_attr(&(ltr559_init_info.platform_diver_addr->driver));
exit_create_attr_failed:
	misc_deregister(&ltr559_device);
exit_misc_device_register_failed:
	free_irq(ltr559_obj->irq, NULL);
exit_init_failed:
	ltr559_i2c_client = NULL;
	ltr559_obj = NULL;
	kfree(obj);
exit:
	APS_ERR("%s: err = %d\n", __func__, err);
	return err;
}

static int ltr559_i2c_remove(struct i2c_client *client)
{
	int err;

	free_irq(ltr559_obj->irq, NULL);

	err = ltr559_delete_attr(&(ltr559_init_info.platform_diver_addr->driver));
	if(err)
		APS_ERR("ltr559_delete_attr fail: %d\n", err);

	err = misc_deregister(&ltr559_device);
	if(err)
		APS_ERR("misc_deregister fail: %d\n", err);    
	
	ltr559_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{.compatible = "mediatek,alsps_ltr559"},
	{},
};
#endif

static const struct i2c_device_id ltr559_i2c_id[] = {
	{LTR559_DEV_NAME,0},
	{}
};

static struct i2c_driver ltr559_i2c_driver = {	
	.probe	= ltr559_i2c_probe,
	.remove	= ltr559_i2c_remove,
	.detect	= ltr559_i2c_detect,
	.suspend	= ltr559_i2c_suspend,
	.resume	= ltr559_i2c_resume,
	.id_table	= ltr559_i2c_id,
	.driver	= {
		.name	= LTR559_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif
	},
};

static int  ltr559_local_init(void)
{
	ltr559_power(hw, 1);
	
	if (i2c_add_driver(&ltr559_i2c_driver)) {
		APS_ERR("add driver error\n");
		return -1;
	}
	
	if (-1 == ltr559_init_flag)
		return -1;

	return 0;
}

static int ltr559_remove(void)
{
	APS_FUN();    
	ltr559_power(hw, 0);    
	i2c_del_driver(&ltr559_i2c_driver);

	return 0;
}


static int __init ltr559_init(void)
{
	const char *name = "mediatek,ltr559";

	hw = get_alsps_dts_func(name, hw);
	if (!hw)
		APS_ERR("get dts info fail\n");

	alsps_driver_add(&ltr559_init_info);

	return 0;
}

static void __exit ltr559_exit(void)
{
	APS_FUN();
}

module_init(ltr559_init);
module_exit(ltr559_exit);

MODULE_AUTHOR("XX Xx");
MODULE_DESCRIPTION("LTR-559ALS Driver");
MODULE_LICENSE("GPL");
