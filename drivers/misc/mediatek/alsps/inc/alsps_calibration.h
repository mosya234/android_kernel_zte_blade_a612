#ifndef __ALSPS_CALIBRATION_H__
#define __ALSPS_CALIBRATION_H__

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/proc_fs.h>

#include <hwmsensor.h>
#include <hwmsen_dev.h>
#include <sensors_io.h>
#include <hwmsen_helper.h>
#include <batch.h>

#include "alsps.h"
#include "cust_alsps.h"

typedef enum
{
	PS_CAL_FACTORY = 20, // ps calibrate in factory mode, absolutely no shelter
	PS_CAL_STARTUP, // ps start up calibration
	PS_CAL_CALL, // ps call calibration
} PS_CAL_TYPE;

typedef enum
{
	PS_THRESHOLD_SET,
	PS_CAL_RESULT_GET,
}PS_THRESHOLD_TYPE;

extern struct alsps_context *alsps_context_obj;

int alsps_calibration_init(void);

#endif
