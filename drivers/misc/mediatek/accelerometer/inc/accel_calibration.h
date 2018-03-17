#ifndef __ACCEL_CALIBRATION_H__
#define __ACCEL_CALIBRATION_H__

#include <linux/uaccess.h>
#include <sensors_io.h>
#include <linux/proc_fs.h>

#include "cust_acc.h"
#include "accel.h"

extern struct acc_context *acc_context_obj;

int acc_calibration_init(void);

#endif

