#include "inc/alsps_calibration.h"
#ifdef SAVE_PARAM_BY_DRIVER
#include "../ztecfg/ztecfg.h"
#endif

// only for ps start up calibration
static void alsps_calibration(struct work_struct *work)
{
	struct alsps_context *cxt = alsps_context_obj;
	int ps_cali_res[4] = {0, 0, 0, 0};
	
	if(!cxt->ps_ctl.ps_calibration || !cxt->ps_ctl.ps_threshold_setting)
	{
		ALSPS_LOG("no ps_calibration available!\n");
		return;
	}
	
	cxt->ps_ctl.ps_calibration(PS_CAL_STARTUP, cxt->alsps_factory_calibration_value);
	cxt->ps_ctl.ps_threshold_setting(PS_THRESHOLD_SET, NULL);
	cxt->ps_ctl.ps_threshold_setting(PS_CAL_RESULT_GET, ps_cali_res);
	
	ALSPS_LOG("ps start_up calibration ct=%d valid=%d hi=%d lo=%d factory_value=%d\n",
		ps_cali_res[0], ps_cali_res[1], ps_cali_res[2], ps_cali_res[3], cxt->alsps_factory_calibration_value);
}

static int alsps_calibration_open(struct inode *inode, struct file *file)
{
	file->private_data = alsps_context_obj;

	if (file->private_data == NULL) {
		ALSPS_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}

static int alsps_calibration_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static ssize_t alsps_calibration_read_proc(struct file *filp, char __user *user_buff, size_t len, loff_t *off)
{
	struct alsps_context *cxt = alsps_context_obj;
	int ps_cali_res[4] = {0, 0, 0, 0};
	int cnt;
	char buff[32];

	if(!cxt->ps_ctl.ps_calibration || !cxt->ps_ctl.ps_threshold_setting)
		ALSPS_LOG("no ps_calibration available!\n");
	else
	{
		cxt->ps_ctl.ps_calibration(PS_CAL_FACTORY, 0);
		cxt->ps_ctl.ps_threshold_setting(PS_THRESHOLD_SET, NULL);
		cxt->ps_ctl.ps_threshold_setting(PS_CAL_RESULT_GET, ps_cali_res);
	}

#ifdef SAVE_PARAM_BY_DRIVER
	if(ps_cali_res[1] == 1)
		write_params_to_ztecfg(TYPE_PS, ps_cali_res[0], ps_cali_res[2], ps_cali_res[3]);
#endif

	cnt = sprintf(buff, "%d %d %d %d\n", ps_cali_res[0], ps_cali_res[1], ps_cali_res[2], ps_cali_res[3]);

	ALSPS_LOG("%s ct=%d valid=%d hi=%d lo=%d\n",
		__func__, ps_cali_res[0], ps_cali_res[1], ps_cali_res[2], ps_cali_res[3]);

	return simple_read_from_buffer(user_buff, len, off, buff, cnt);
}

static ssize_t alsps_calibration_write_proc(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	int value = -1;
	struct alsps_context *cxt = alsps_context_obj;
#ifdef SAVE_PARAM_BY_DRIVER
	struct ztecfg_data params = {0};
#endif

	sscanf(buff, "%d", &value);
	if(value == 1)
	{
#ifdef SAVE_PARAM_BY_DRIVER
		read_params_from_ztecfg(&params);
		cxt->alsps_factory_calibration_value = params.ps_cal_crosstalk;
#endif
		queue_delayed_work(alsps_context_obj->alsps_calibration_wq, &alsps_context_obj->alsps_calibration_work, HZ/10); 
		ALSPS_LOG("%s,factory_calibration_value=%d\n",__func__,cxt->alsps_factory_calibration_value);
	}
	else
		ALSPS_LOG("%s wrong params %d!", __func__, value);
	
	return len;
}

static const struct file_operations alsps_calibration_fops = {
	.owner	= THIS_MODULE,
	.open	= alsps_calibration_open,
	.release	= alsps_calibration_release,
	.read	= alsps_calibration_read_proc,
	.write	= alsps_calibration_write_proc,
};

static ssize_t alsps_call_calibration_read_proc(struct file *filp, char __user *user_buff, size_t len, loff_t *off)
{
	struct alsps_context *cxt = alsps_context_obj;
	int ps_cali_res[4] = {0, 0, 0, 0};
	int cnt;
	char buff[32];

	if(!cxt->ps_ctl.ps_calibration || !cxt->ps_ctl.ps_threshold_setting)
		ALSPS_LOG("no ps_calibration available!\n");
	else
	{
		cxt->ps_ctl.ps_calibration(PS_CAL_CALL, 0);
		cxt->ps_ctl.ps_threshold_setting(PS_THRESHOLD_SET, NULL);
		cxt->ps_ctl.ps_threshold_setting(PS_CAL_RESULT_GET, ps_cali_res);
	}
	
	cnt = sprintf(buff, "%d %d %d %d\n", ps_cali_res[0], ps_cali_res[1], ps_cali_res[2], ps_cali_res[3]);

	ALSPS_LOG("%s ct=%d valid=%d hi=%d lo=%d\n",
		__func__, ps_cali_res[0], ps_cali_res[1], ps_cali_res[2], ps_cali_res[3]);

	return simple_read_from_buffer(user_buff, len, off, buff, cnt);
}

static ssize_t alsps_call_calibration_write_proc(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	return len;
}

static const struct file_operations alsps_call_calibration_fops = {
	.owner	= THIS_MODULE,
	.open	= alsps_calibration_open,
	.release	= alsps_calibration_release,
	.read	= alsps_call_calibration_read_proc,
	.write	= alsps_call_calibration_write_proc,
};

int alsps_calibration_init(void)
{
	struct proc_dir_entry *alsps_proc_file; 
	struct proc_dir_entry *alsps_call_calibration_proc_file; 
	
	INIT_DELAYED_WORK(&alsps_context_obj->alsps_calibration_work, alsps_calibration);
	alsps_context_obj->alsps_calibration_wq = create_singlethread_workqueue("alsps_calibration");
	if(alsps_context_obj->alsps_calibration_wq == NULL){
		ALSPS_ERR("create alsps_calibration workqueue failed!\n");
		return -1;
	}

	alsps_proc_file = proc_create("driver/ps_threshold", 0646, NULL, &alsps_calibration_fops);
	if(alsps_proc_file == NULL)
	{
		ALSPS_ERR("create /driver/ps_threshold/ file failed!\n");
		return -1;
	}
	ALSPS_LOG("driver/ps_threshold created Success!\n");

	alsps_call_calibration_proc_file = proc_create("driver/calibration_inCall", 0646, NULL, &alsps_call_calibration_fops);
	if(alsps_call_calibration_proc_file == NULL)
	{
		ALSPS_ERR("create /driver/calibration_inCall/ file failed!\n");
		return -1;
	}
	ALSPS_LOG("driver/calibration_inCall created Success!\n");

	return 0;
}
