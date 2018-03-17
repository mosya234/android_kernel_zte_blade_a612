/*! @file vfsSpiDrv.c
*******************************************************************************
**  SPI Driver Interface Functions
**
**  This file contains the SPI driver interface functions.
**
**  Copyright (C) 2011-2013 Validity Sensors, Inc.
**  This program is free software; you can redistribute it and/or
**  modify it under the terms of the GNU General Public License
**  as published by the Free Software Foundation; either version 2
**  of the License, or (at your option) any later version.
**  
**  This program is distributed in the hope that it will be useful,
**  but WITHOUT ANY WARRANTY; without even the implied warranty of
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
**  GNU General Public License for more details.
**  
**  You should have received a copy of the GNU General Public License
**  along with this program; if not, write to the Free Software
**  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
**  
*/

#include "vfsspidrv.h"

#include <linux/kernel.h>
#include <linux/cdev.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/debugfs.h>
#include <linux/kthread.h>
#include <linux/fb.h>
#include <linux/notifier.h>

#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioctl.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/slab.h>

#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/i2c/twl.h>
#include <linux/wait.h>
#ifdef WOF
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/types.h>
#endif 
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <linux/compat.h>

#include <asm-generic/siginfo.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include "../fingerprint_core.h"
#include <linux/proc_fs.h>
#define VALIDITY_PART_NAME "validity_fingerprint"

#ifdef WOF
#define VFSSPI_WAKE_TIME   (5 * HZ)
#endif

/* This orientation is based on the position of the sensor 
 * starting from the Top Direction */
#define VFS_SENSOR_ORIENTATION_URDL	1 /* This is the usual configuation when the sensor is mounted Frontside */ 
#define VFS_SENSOR_ORIENTATION_LURD	2
#define VFS_SENSOR_ORIENTATION_DLUR	3
#define VFS_SENSOR_ORIENTATION_RDLU	4
#define VFS_SENSOR_ORIENTATION_ULDR	5  /* This is the usual configuation when the sensor is mounted backside */
#define VFS_SENSOR_ORIENTATION_LDRU	6
#define VFS_SENSOR_ORIENTATION_DRUL	7
#define VFS_SENSOR_ORIENTATION_RULD	8


static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_mutex);
static struct class *vfsspi_device_class;
static int gpio_irq;
struct vfsspi_device_data *g_vfsSpiDev = NULL;	
static int vfs_finger_detected = 0;

struct mt_chip_conf spi_conf_mt65xx = {
	.setuptime = 10,
	.holdtime = 10,
	.high_time = 7, //for mt6582, 104000khz/(4+4) = 13000khz
	.low_time =7,
	.cs_idletime = 2,
	.ulthgh_thrsh = 0,

	.cpol = 0,
	.cpha = 0,

	.rx_mlsb = 1,
	.tx_mlsb = 1,

	.tx_endian = 0,
	.rx_endian = 0,

	.com_mod = DMA_TRANSFER,//DMA_TRANSFER,
	.pause = 1,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};




#if 0
#ifdef CONFIG_OF
static struct of_device_id validity_metallica_table[] = {
	{ .compatible = "validity,metallica",},
	{ },
};
#else
#define validity_metallica_table NULL
#endif
#endif

static int vfsspi_enableIrq(struct vfsspi_device_data *);

/*
 * vfsspi_devData - The spi driver private structure
 * @devt:Device ID
 * @vfs_spi_lock:The lock for the spi device
 * @spi:The spi device
 * @device_entry:Device entry list
 * @buffer_mutex:The lock for the transfer buffer
 * @is_opened:Indicates that driver is opened
 * @buffer:buffer for transmitting data
 * @null_buffer:buffer for transmitting zeros
 * @stream_buffer:buffer for transmitting data stream
 * @stream_buffer_size:streaming buffer size
 * @drdy_pin:DRDY GPIO pin number
 * @sleep_pin:Sleep GPIO pin number
 * @user_pid:User process ID, to which the kernel signal
 *	indicating DRDY event is to be sent
 * @signal_id:Signal ID which kernel uses to indicating
 *	user mode driver that DRDY is asserted
 * @current_spi_speed:Current baud rate of SPI master clock
 */
struct vfsspi_device_data {
	dev_t devt;
	struct cdev cdev;
	spinlock_t vfs_spi_lock;
	struct spi_device *spi;
	struct list_head device_entry;
	struct mutex buffer_mutex;
	unsigned int is_opened;
	unsigned char *buffer;
	unsigned char *null_buffer;
	unsigned char *stream_buffer;
	size_t stream_buffer_size;
	unsigned int drdy_pin;
	unsigned int sleep_pin;
#if DO_CHIP_SELECT
	unsigned int cs_pin;
#endif
	int user_pid;
	int signal_id;
	int eUserPID;
	int eSignalID;
	struct notifier_block fb_notifier;
	unsigned int current_spi_speed;
	unsigned int is_drdy_irq_enabled;
	unsigned int drdy_ntf_type;
	struct mutex kernel_lock;
	#ifdef WOF
    struct wake_lock         wake_lock;
    int                      wake_lock_acquired;
    struct timer_list        wake_unlock_timer;
    struct work_struct       irq_worker;
	#endif
};

struct vfsspi_device_data *vfsSpiDevTmp = NULL;//power consume controll
#ifdef VFSSPI_32BIT
/*
 * Used by IOCTL compat command:
 *         VFSSPI_IOCTL_RW_SPI_MESSAGE
 *
 * @rx_buffer:pointer to retrieved data
 * @tx_buffer:pointer to transmitted data
 * @len:transmitted/retrieved data size
 */
struct vfsspi_compat_ioctl_transfer {
	compat_uptr_t rx_buffer;
	compat_uptr_t tx_buffer;
	unsigned int len;
};
#endif

//for screen status +++
int screen_status = 3; //0= off, 1=on, 3=unknown

static ssize_t screen_status_show(struct device *dev, struct device_attribute *attr,
                char *buf)
{
        //int pos_ind= 0;
	int ret;
        printk("[FP] screen status = %d\n", screen_status);
	ret = sprintf(buf, "%d\n", screen_status);
        //return screen_status;
	return ret;
}

DEVICE_ATTR(screenState, (S_IWUSR|S_IRUGO), screen_status_show, NULL);
//for screen status  ---
static int vfsspi_sendDrdyEventFd(struct vfsspi_device_data *vfsSpiDev);
static int vfsspi_sendDrdyNotify(struct vfsspi_device_data *vfsSpiDev);

#ifdef WOF
static void vfsspi_wake_unlock(struct vfsspi_device_data *data)
{
    pr_debug("%s: enter\n", __func__);
    if (data->wake_lock_acquired) {
        wake_unlock(&data->wake_lock);
        data->wake_lock_acquired = 0;
    }
}
static void vfsspi_wake_unlock_timer_handler(unsigned long ptr)
{
    struct vfsspi_device_data *data = (struct vfsspi_device_data*)ptr;
    pr_debug("%s: enter\n", __func__);
		vfsspi_enableIrq(data);

    vfsspi_wake_unlock(data);
}
static void vfsspi_wake_lock_delayed_unlock(struct vfsspi_device_data *data)
{
    pr_debug("%s: enter\n", __func__);
    if (!data->wake_lock_acquired) {
        wake_lock(&data->wake_lock);
        data->wake_lock_acquired = 1;
    }
    mod_timer(&data->wake_unlock_timer, jiffies + VFSSPI_WAKE_TIME);
}
static void vfsspi_irq_worker(struct work_struct *arg)
{
    struct vfsspi_device_data *vfsspi_device = container_of(arg, struct vfsspi_device_data, irq_worker);
    vfsspi_wake_lock_delayed_unlock(vfsspi_device);
    vfsspi_sendDrdyNotify(vfsspi_device);
    printk("%s: exit\n", __func__);
}
#endif
static int vfsspi_send_drdy_signal(struct vfsspi_device_data *vfsspi_device)
{
	struct task_struct *t;
	int ret = 0;

	DPRINTK("vfsspi_send_drdy_signal\n");

	if (vfsspi_device->user_pid != 0) {
		rcu_read_lock();
		/* find the task_struct associated with userpid */
		DPRINTK("Searching task with PID=%08x\n",
			vfsspi_device->user_pid);
		t = pid_task(find_pid_ns(vfsspi_device->user_pid, &init_pid_ns),
			     PIDTYPE_PID);
		if (t == NULL) {
			DPRINTK("No such pid\n");
			rcu_read_unlock();
			return -ENODEV;
		}
		rcu_read_unlock();
		/* notify DRDY signal to user process */
		ret = send_sig_info(vfsspi_device->signal_id,
				    (struct siginfo *)1, t);
		if (ret < 0)
			DPRINTK("Error sending signal\n");

	} else {
		DPRINTK("pid not received yet\n");
	}

	return ret;
}
void vfsspi_screen_notify(void)
{
	struct task_struct *t;
#if 0
	struct file *efd_file = NULL;
	struct eventfd_ctx *efd_ctx = NULL;
#endif
	int ret = 0;

	printk("vfsspi screen status change to %d\n",screen_status);
#if 1 //for signal
	if(vfsSpiDevTmp!=NULL){
		if (vfsSpiDevTmp->eUserPID != 0) {
			rcu_read_lock();
			/* find the task_struct associated with userpid */
			printk("Searching task with PID=%08x\n", vfsSpiDevTmp->eUserPID);
			t = pid_task(find_pid_ns(vfsSpiDevTmp->eUserPID, &init_pid_ns),
				     PIDTYPE_PID);
			if (t == NULL) {
				printk("No such pid\n");
				rcu_read_unlock();
				return;
			}
			rcu_read_unlock();
			/* notify screen signal to user process */
			ret =
			    send_sig_info(vfsSpiDevTmp->eSignalID, (struct siginfo *)1,
					  t);
			if (ret < 0)
				printk("Error sending screen off signal\n");
			else
				printk("pid not received yet\n");
		}
	}
#else //for event
	if(vfsSpiDevTmp!=NULL){
	    if (vfsSpiDevTmp->eUserPID != 0) {
	        rcu_read_lock();
	        /* find the task_struct associated with userpid */
	        printk("Searching task with PID=%08x\n", vfsSpiDevTmp->eUserPID);
	        t = pid_task(find_pid_ns(vfsSpiDevTmp->eUserPID, &init_pid_ns),
	            PIDTYPE_PID);
	        if (t == NULL) {
	            printk("No such pid\n");
	            rcu_read_unlock();
	            return -ENODEV;
	        }
	        efd_file = fcheck_files(t->files, vfsSpiDevTmp->eSignalID);
	        rcu_read_unlock();

	        if (efd_file == NULL) {
	            printk("No such efd_file\n");
	            return -ENODEV;
	        }
	        
	        efd_ctx = eventfd_ctx_fileget(efd_file);
	        if (efd_ctx == NULL) {
	            printk("eventfd_ctx_fileget is failed\n");
	            return -ENODEV;
	        }

	        /* notify DRDY eventfd to user process */
	        eventfd_signal(efd_ctx, 1);

	        /* Release eventfd context */
	        eventfd_ctx_put(efd_ctx);
	    }
	}
#endif
}

#if 0 //disable for merge to one function
void vfsspi_screen_on(void)
{
	struct task_struct *t;
	int ret = 0;

	printk("vfsspi screen on\n");

#if 1 //for signal
	if(vfsSpiDevTmp!=NULL){
		if (vfsSpiDevTmp->eUserPID != 0) {
			rcu_read_lock();
			/* find the task_struct associated with userpid */
			printk("Searching task with PID=%08x\n", vfsSpiDevTmp->eUserPID);
			t = pid_task(find_pid_ns(vfsSpiDevTmp->eUserPID, &init_pid_ns),
				     PIDTYPE_PID);
			if (t == NULL) {
				printk("No such pid\n");
				rcu_read_unlock();
				return;
			}
			rcu_read_unlock();
			/* notify screen signal to user process */
			ret =
			    send_sig_info(vfsSpiDevTmp->eSignalID, (struct siginfo *)1,
					  t);
			if (ret < 0)
				printk("Error sending screen on signal\n");
			else
				printk("pid not received yet\n");
		}
	}
#else //for event
	if(vfsSpiDevTmp!=NULL){
		if (vfsSpiDevTmp->eUserPID != 0) {
			rcu_read_lock();
			/* find the task_struct associated with userpid */
			printk("Searching task with PID=%08x\n", vfsSpiDevTmp->eUserPID);
			t = pid_task(find_pid_ns(vfsSpiDevTmp->eUserPID, &init_pid_ns),
				PIDTYPE_PID);
			if (t == NULL) {
				printk("No such pid\n");
				rcu_read_unlock();
				return -ENODEV;
			}
			efd_file = fcheck_files(t->files, vfsSpiDevTmp->eSignalID);
			rcu_read_unlock();
	
			if (efd_file == NULL) {
				printk("No such efd_file\n");
				return -ENODEV;
			}
			
			efd_ctx = eventfd_ctx_fileget(efd_file);
			if (efd_ctx == NULL) {
				printk("eventfd_ctx_fileget is failed\n");
				return -ENODEV;
			}
	
			/* notify DRDY eventfd to user process */
			eventfd_signal(efd_ctx, 1);
	
			/* Release eventfd context */
			eventfd_ctx_put(efd_ctx);
		}
	}
#endif
}
#endif

// add for fb detection start
static int SYNA_fb_state_notify_callback(struct notifier_block *nb, unsigned long val, void *fbdata)
{
	struct fb_event *fb_data = fbdata;
	unsigned int fb_state;

	if (val != FB_EARLY_EVENT_BLANK)
		return 0;
	
	printk("SYNA_fb_state_notify_callback value = %d\n",(int)val);
	
	if (fb_data && fb_data->data) 
	{
		fb_state = *(int *)(fb_data->data);
		switch (fb_state) {
		case FB_BLANK_POWERDOWN:
			screen_status = 0; //set the screen status to off
			printk("SYNA_fb_state_notify_callback FB_BLANK_POWERDOWN - vfsspi_screen_off\n");
			vfsspi_screen_notify();
			break;
		case FB_BLANK_UNBLANK:
			screen_status = 1; //set the screen status to on
			printk("SYNA_fb_state_notify_callback FB_BLANK_UNBLANK - vfsspi_screen_on\n");	
			vfsspi_screen_notify();
			break;
		default:
			printk("SYNA_fb_state_notify_callback: defalut\n");
			break;
		}
	}
	return NOTIFY_OK;
}


static struct notifier_block SYNA_notify_block = {	
	.notifier_call = SYNA_fb_state_notify_callback,
};
// add for fb detection end
/* Return no. of bytes written to device. Negative number for errors */
static inline ssize_t vfsspi_writeSync(struct vfsspi_device_data *vfsspi_device,
					size_t len)
{
	int    status = 0;
	struct spi_message m;

	struct spi_transfer cmd, data;

	DPRINTK("vfsspi_writeSync\n");

	spi_message_init(&m);
	DPRINTK("vfsspi_writeSync: len =%zu [%s]\n", len, vfsspi_device->null_buffer);
	

	if(len > 1024 && len % 1024 != 0)
	{
		printk("vfsspi_writeSync data size > 1024 (%zu)\n", len);
		memset(&cmd, 0, sizeof(cmd));
		memset(&data, 0, sizeof(data));

		//cmd.cs_change = 0;
		cmd.rx_buf    = vfsspi_device->null_buffer;
		cmd.tx_buf    = vfsspi_device->buffer;
		cmd.len       = 1024 * (len/1024);
		cmd.speed_hz  = vfsspi_device->current_spi_speed;

		//data.cs_change = 1;
		data.len       = len % 1024;
		data.rx_buf    = vfsspi_device->null_buffer + cmd.len;
		data.tx_buf    = vfsspi_device->buffer + cmd.len;
		data.speed_hz  = vfsspi_device->current_spi_speed;

		spi_message_add_tail(&cmd, &m);
		spi_message_add_tail(&data, &m);
		printk("vfsspi_writeSync data size > 1024 (%zu = %d + %d)\n", len, cmd.len, data.len);
	}
	else
	{
		memset(&data, 0, sizeof(data));

		data.len       = len;
		data.rx_buf    = vfsspi_device->null_buffer;
		data.tx_buf    = vfsspi_device->buffer;
		data.speed_hz  = vfsspi_device->current_spi_speed;

		spi_message_add_tail(&data, &m);
	}


	status = spi_sync(vfsspi_device->spi, &m);
       if (status == 0){
		status = m.actual_length;
	        DPRINTK("vfsspi_writeSync,length=%d\n", status);
        }
       else{
		DPRINTK("spi_sync fail, status=%d\n", status);
        }

	
	return status;
}

/* Return no. of bytes read > 0. negative integer incase of error. */
static inline ssize_t vfsspi_readSync(struct vfsspi_device_data *vfsspi_device,
					size_t len)
{
	int    status = 0;
	struct spi_message m;

	struct spi_transfer cmd, data;


	DPRINTK("vfsspi_readSync\n");

	spi_message_init(&m);
	memset(vfsspi_device->null_buffer, 0x0, len);

	if(len > 1024 && len % 1024 != 0)
	{
		memset(&cmd, 0, sizeof(cmd));
		memset(&data, 0, sizeof(data));

		//cmd.cs_change = 0;
		cmd.tx_buf    = vfsspi_device->null_buffer;
		cmd.rx_buf    = vfsspi_device->buffer;
		cmd.len       = 1024 * (len/1024);
		cmd.speed_hz  = vfsspi_device->current_spi_speed;

		//data.cs_change = 1;
		data.len       = len % 1024;
		data.tx_buf    = vfsspi_device->null_buffer + cmd.len;
		data.rx_buf    = vfsspi_device->buffer + cmd.len;
		data.speed_hz  = vfsspi_device->current_spi_speed;

		spi_message_add_tail(&cmd, &m);
		spi_message_add_tail(&data, &m);
		printk("vfsspi_readSync data size > 1024 (%zu = %d + %d)\n", len, cmd.len, data.len);
	}
	else
	{
		memset(&data, 0, sizeof(data));

		data.len       = len;
		data.tx_buf    = vfsspi_device->null_buffer;
		data.rx_buf    = vfsspi_device->buffer;
		data.speed_hz  = vfsspi_device->current_spi_speed;

		spi_message_add_tail(&data, &m);
	}


	status = spi_sync(vfsspi_device->spi, &m);
	if (status == 0){
	
		status = m.actual_length;
		

		DPRINTK("vfsspi_readSync,length=%d\n", status);
	 }
	else{
		DPRINTK("spi_sync fail, status=%d\n", status);

	 }

	return status;
}

static ssize_t vfsspi_write(struct file *filp, const char __user *buf,
			size_t count, loff_t *fPos)
{
	struct vfsspi_device_data *vfsspi_device = NULL;
	ssize_t               status = 0;

	
	DPRINTK("vfsspi_write:the count is length :count =%d\n", (int)count);

	if (count > DEFAULT_BUFFER_SIZE || count <= 0)
	{
		DPRINTK("vfsspi_write count incorrect (%zu)\n", count);
		return -EMSGSIZE;
	}

	vfsspi_device = filp->private_data;

	mutex_lock(&vfsspi_device->buffer_mutex);

	if (vfsspi_device->buffer) {
		unsigned long missing = 0;

		missing = copy_from_user(vfsspi_device->buffer, buf, count);

		if (missing == 0)
		{
			status = vfsspi_writeSync(vfsspi_device, count);
		}
                else
		{
			status = -EFAULT;
			DPRINTK("vfsspi_write:copy_from_user fail %ld\n",missing);
		}
	}

	mutex_unlock(&vfsspi_device->buffer_mutex);

	return status;
}



static ssize_t vfsspi_read(struct file *filp, char __user *buf,
			size_t count, loff_t *fPos)
{
	struct vfsspi_device_data *vfsspi_device = NULL;
	ssize_t                status    = 0;

	DPRINTK("vfsspi_read, %d\n", (int)count);

	if (count > DEFAULT_BUFFER_SIZE || count <= 0)
	{
		printk("vfsspi_read count size incorrect (%zu)\n", count);
		return -EMSGSIZE;
	}
	if (buf == NULL)
	{
		printk("vfsspi_read buf is NULL\n");
		return -EFAULT;
	}


	vfsspi_device = filp->private_data;

	mutex_lock(&vfsspi_device->buffer_mutex);

	status  = vfsspi_readSync(vfsspi_device, count);


	if (status > 0) {
		unsigned long missing = 0;
		/* data read. Copy to user buffer.*/
		missing = copy_to_user(buf, vfsspi_device->buffer, status);

		if (missing == status) {
			
			printk(" copy_to_user failed\n");
			/* Nothing was copied to user space buffer. */
			status = -EFAULT;
		} else {
			status = status - missing;
		}
	}

	mutex_unlock(&vfsspi_device->buffer_mutex);

	return status;
}



static int vfsspi_xfer(struct vfsspi_device_data *vfsspi_device,
			struct vfsspi_ioctl_transfer *tr)
{
	int status = 0;
	struct spi_message m;

	struct spi_transfer cmd, data;


	
//	printk("vfsspi_xfer\n");

	if (vfsspi_device == NULL || tr == NULL)
	{
		printk("vfsspi_xfer device or tr is NULL\n");
		return -EFAULT;
         }

	if (tr->len > DEFAULT_BUFFER_SIZE || tr->len <= 0)
	{
		printk("vfsspi_xfer len size incorrect (%d)\n", tr->len);
		return -EMSGSIZE;
         }

	if (tr->tx_buffer != NULL) {
		if (copy_from_user(vfsspi_device->null_buffer,
				tr->tx_buffer, tr->len) != 0)
			return -EFAULT;
	}

	spi_message_init(&m);
	//printk("vfsspi_xSync: len =%d [%s]\n", tr->len, vfsspi_device->null_buffer);


	if(tr->len > 1024 && tr->len % 1024 != 0)
	{
		printk("vfsspi_xfer data size > 1024 (%d)\n", tr->len);
		memset(&cmd, 0, sizeof(cmd));
		memset(&data, 0, sizeof(data));

		//cmd.cs_change = 0;
		cmd.tx_buf    = vfsspi_device->null_buffer;
		cmd.rx_buf    = vfsspi_device->buffer;
		cmd.len       = 1024 * (tr->len/1024);
		cmd.speed_hz  = vfsspi_device->current_spi_speed;

		//data.cs_change = 1;
		data.len       = tr->len % 1024;
		data.tx_buf    = vfsspi_device->null_buffer + cmd.len;
		data.rx_buf    = vfsspi_device->buffer + cmd.len;
		data.speed_hz  = vfsspi_device->current_spi_speed;

		spi_message_add_tail(&cmd, &m);
		spi_message_add_tail(&data, &m);
		printk("vfsspi_xfer data size > 1024 (%d = %d + %d)\n", tr->len, cmd.len, data.len);
	}
	else
	{
		memset(&data, 0, sizeof(data));

		data.len       = tr->len;
		data.tx_buf    = vfsspi_device->null_buffer;
		data.rx_buf    = vfsspi_device->buffer;
		data.speed_hz  = vfsspi_device->current_spi_speed;

		spi_message_add_tail(&data, &m);
	}


	status = spi_sync(vfsspi_device->spi, &m);

	if (status == 0) {
		if (tr->rx_buffer != NULL) {
			unsigned missing = 0;

			missing = copy_to_user(tr->rx_buffer,
					       vfsspi_device->buffer, tr->len);

			if (missing != 0)
				tr->len = tr->len - missing;
		}
	}
	else
		printk("vfsspi_xfer fail, status = %d\n", status);
	
//	printk("vfsspi_xfer,length=%d\n", tr->len);
	return status;

}  /* vfsspi_xfer */


static int vfsspi_rw_spi_message(struct vfsspi_device_data *vfsspi_device,
				 unsigned long arg)
{
	struct vfsspi_ioctl_transfer   *dup  = NULL;
#ifdef VFSSPI_32BIT
    struct vfsspi_compat_ioctl_transfer   dup_compat;
#endif	
	DPRINTK("vfsspi_rw_spi_message arg = %ld\n",arg);
	dup = kmalloc(sizeof(struct vfsspi_ioctl_transfer), GFP_KERNEL);
	if (dup == NULL)
		return -ENOMEM;
#ifdef VFSSPI_32BIT
	if (copy_from_user(&dup_compat, (void __user *)arg,
			   sizeof(struct vfsspi_compat_ioctl_transfer)) != 0)  {
#else
	if (copy_from_user(dup, (void __user *)arg,
			   sizeof(struct vfsspi_ioctl_transfer)) != 0)  {
#endif
		return -EFAULT;
	} else {
		int err;
#ifdef VFSSPI_32BIT		
		dup->rx_buffer = (unsigned char *)(unsigned long)dup_compat.rx_buffer;
		dup->tx_buffer = (unsigned char *)(unsigned long)dup_compat.tx_buffer;
		dup->len = dup_compat.len;
#endif
		err = vfsspi_xfer(vfsspi_device, dup);
		if (err != 0) {
	                DPRINTK("vfsspi_xfer is error\n");
			kfree(dup);
			return err;
		}
	}
#ifdef VFSSPI_32BIT
    dup_compat.len = dup->len;
	if (copy_to_user((void __user *)arg, &dup_compat,
			 sizeof(struct vfsspi_compat_ioctl_transfer)) != 0){
#else
	if (copy_to_user((void __user *)arg, dup,
			 sizeof(struct vfsspi_ioctl_transfer)) != 0){
#endif
		kfree(dup);
		return -EFAULT;
	}
	kfree(dup);
	return 0;
}

static int vfsspi_set_clk(struct vfsspi_device_data *vfsspi_device,
			  unsigned long arg)
{
	unsigned short clock = 0;
	struct spi_device *spidev = NULL;

	DPRINTK("vfsspi_set_clk\n");

	if (copy_from_user(&clock, (void __user *)arg,
			   sizeof(unsigned short)) != 0)
		return -EFAULT;

	spin_lock_irq(&vfsspi_device->vfs_spi_lock);
	spidev = spi_dev_get(vfsspi_device->spi);
	spin_unlock_irq(&vfsspi_device->vfs_spi_lock);
	if (spidev != NULL) {
		switch (clock) {
		case 0:	/* Running baud rate. */
			DPRINTK("Running baud rate.\n");
			spidev->max_speed_hz = MAX_BAUD_RATE;
			vfsspi_device->current_spi_speed = MAX_BAUD_RATE;
			break;
		case 0xFFFF: /* Slow baud rate */
			DPRINTK("slow baud rate.\n");
			spidev->max_speed_hz = SLOW_BAUD_RATE;
			vfsspi_device->current_spi_speed = SLOW_BAUD_RATE;
			break;
		default:
			DPRINTK("baud rate is %d.\n", clock);
			vfsspi_device->current_spi_speed = 
				clock * BAUD_RATE_COEF;
			if (vfsspi_device->current_spi_speed > MAX_BAUD_RATE)
				vfsspi_device->current_spi_speed =
					MAX_BAUD_RATE;
			spidev->max_speed_hz = vfsspi_device->current_spi_speed;
			break;
		}
		spi_dev_put(spidev);
	}
	return 0;
}

static int vfsspi_register_drdy_signal(struct vfsspi_device_data *vfsspi_device,
				       unsigned long arg)
{
	struct vfsspi_ioctl_register_signal usr_signal;

	DPRINTK("vfsspi_register_drdy_signal\n");
	if (copy_from_user(&usr_signal, (void __user *)arg, sizeof(usr_signal)) != 0) {
		DPRINTK("Failed copy from user.\n");
		return -EFAULT;
	} else {
		vfsspi_device->user_pid = usr_signal.user_pid;
		vfsspi_device->signal_id = usr_signal.signal_id;
	}
	return 0;
}

static int vfsspi_register_screen_signal(struct vfsspi_device_data *vfsspi_device,
				       unsigned long arg)
{
	struct vfsspi_ioctl_register_signal usrSignal_screen;
	printk("Enter vfsspi_register_screen_signal\n");
	if (copy_from_user(&usrSignal_screen, (void *)arg, sizeof(usrSignal_screen)) != 0) {
		printk("Failed copy from user.\n");
		return -EFAULT;
	} else {
		vfsspi_device->eUserPID = usrSignal_screen.user_pid;
		vfsspi_device->eSignalID = usrSignal_screen.signal_id;
	}
	printk("Exit vfsspi_register_screen_signal\n");
	return 0;
}



static irqreturn_t vfsspi_irq(int irq, void *desc)
{
	struct vfsspi_device_data *vfsspi_device = NULL;
	vfsspi_device = g_vfsSpiDev;

	/* Linux kernel is designed so that when you disable
	an edge-triggered interrupt, and the edge happens while
	the interrupt is disabled, the system will re-play the
	interrupt at enable time.
	Therefore, we are checking DRDY GPIO pin state to make sure
	if the interrupt handler has been called actually by DRDY
	interrupt and it's not a previous interrupt re-play */

	if (gpio_get_value(vfsspi_device->drdy_pin) == DRDY_ACTIVE_STATUS) {
	    #ifdef WOF
        schedule_work(&vfsspi_device->irq_worker);
		#else
		vfsspi_sendDrdyNotify(vfsspi_device);
		#endif
	}

	return IRQ_HANDLED;
}

static int vfsspi_sendDrdyEventFd(struct vfsspi_device_data *vfsSpiDev)
{
    struct task_struct *t;
    struct file *efd_file = NULL;
    struct eventfd_ctx *efd_ctx = NULL;	int ret = 0;

    DPRINTK("vfsspi_sendDrdyEventFd\n");

    if (vfsSpiDev->user_pid != 0) {
        rcu_read_lock();
        /* find the task_struct associated with userpid */
        DPRINTK("Searching task with PID=%08x\n", vfsSpiDev->user_pid);
        t = pid_task(find_pid_ns(vfsSpiDev->user_pid, &init_pid_ns),
            PIDTYPE_PID);
        if (t == NULL) {
            DPRINTK("No such pid\n");
            rcu_read_unlock();
            return -ENODEV;
        }
        efd_file = fcheck_files(t->files, vfsSpiDev->signal_id);
        rcu_read_unlock();

        if (efd_file == NULL) {
            DPRINTK("No such efd_file\n");
            return -ENODEV;
        }
        
        efd_ctx = eventfd_ctx_fileget(efd_file);
        if (efd_ctx == NULL) {
            DPRINTK("eventfd_ctx_fileget is failed\n");
            return -ENODEV;
        }

        /* notify DRDY eventfd to user process */
        eventfd_signal(efd_ctx, 1);

        /* Release eventfd context */
        eventfd_ctx_put(efd_ctx);
    }

    return ret;
}

static int vfsspi_sendDrdyNotify(struct vfsspi_device_data *vfsSpiDev)
{
    int ret = 0;

    if (vfsSpiDev->drdy_ntf_type == VFSSPI_DRDY_NOTIFY_TYPE_EVENTFD) {
        ret = vfsspi_sendDrdyEventFd(vfsSpiDev);
    } else {
        ret = vfsspi_send_drdy_signal(vfsSpiDev);
    }

    return ret;
}

static int vfsspi_enableIrq(struct vfsspi_device_data *vfsspi_device)
{
	DPRINTK("vfsspi_enableIrq\n");

	if (vfsspi_device->is_drdy_irq_enabled == DRDY_IRQ_ENABLE) {
		DPRINTK("DRDY irq already enabled\n");
		return -EINVAL;
	}

	enable_irq(gpio_irq);
	vfsspi_device->is_drdy_irq_enabled = DRDY_IRQ_ENABLE;
	//#ifdef WOF
	#if 0
    if (enable_irq_wake(gpio_irq)) {
    	pr_err("fail to enable_irq_wake\n");
        return -3;
    }
	#endif
	return 0;
}

static int vfsspi_disableIrq(struct vfsspi_device_data *vfsspi_device)
{
	DPRINTK("vfsspi_disableIrq\n");

	if (vfsspi_device->is_drdy_irq_enabled == DRDY_IRQ_DISABLE) {
		DPRINTK("DRDY irq already disabled\n");
		return -EINVAL;
	}

	disable_irq_nosync(gpio_irq);
	vfsspi_device->is_drdy_irq_enabled = DRDY_IRQ_DISABLE;
	//#ifdef WOF
	#if 0
	if (disable_irq_wake(gpio_irq)) {
		pr_err("fail to disable_irq_wake\n");
		return -3;
	}
	#endif
	return 0;
}
static int vfsspi_set_drdy_int(struct vfsspi_device_data *vfsspi_device,
			       unsigned long arg)
{
	unsigned short drdy_enable_flag;

        DPRINTK("vfsspi_set_drdy_int\n");

	if (copy_from_user(&drdy_enable_flag, (void __user *)arg,
			   sizeof(drdy_enable_flag)) != 0) {
		DPRINTK("Failed copy from user.\n");
		return -EFAULT;
	}
	if (drdy_enable_flag == 0){
	        #ifdef WOF
        	vfsspi_wake_lock_delayed_unlock(vfsspi_device);
			#endif 
			vfsspi_disableIrq(vfsspi_device);
	}else {
			vfsspi_enableIrq(vfsspi_device);
			/* Workaround the issue where the system
			  misses DRDY notification to host when
			  DRDY pin was asserted before enabling
			  device.*/
			if (gpio_get_value(vfsspi_device->drdy_pin) ==
				DRDY_ACTIVE_STATUS) {
				vfsspi_sendDrdyNotify(vfsspi_device);
			}
	}
	return 0;
}

static void vfsspi_hardReset(struct vfsspi_device_data *vfsspi_device)
{
	DPRINTK("vfsspi_hardReset\n");

	if (vfsspi_device != NULL) {
		pin_select(rst_low);
		mdelay(1);
		pin_select(rst_high);
		mdelay(5);
	}
}


static void vfsspi_suspend(struct vfsspi_device_data *vfsspi_device)
{
	DPRINTK("vfsspi_suspend\n");

	if (vfsspi_device != NULL) {
		spin_lock(&vfsspi_device->vfs_spi_lock);
		pin_select(rst_low);
		spin_unlock(&vfsspi_device->vfs_spi_lock);
	}
}

static long vfsspi_ioctl(struct file *filp, unsigned int cmd,
			unsigned long arg)
{
	int ret_val = 0;
	struct vfsspi_device_data *vfsspi_device = NULL;

//	DPRINTK("vfsspi_ioctl\n");

	if (_IOC_TYPE(cmd) != VFSSPI_IOCTL_MAGIC) {
		DPRINTK("invalid magic. cmd=0x%X Received=0x%X Expected=0x%X\n",
			cmd, _IOC_TYPE(cmd), VFSSPI_IOCTL_MAGIC);
		return -ENOTTY;
	}

	vfsspi_device = filp->private_data;
    vfsSpiDevTmp = vfsspi_device;
	mutex_lock(&vfsspi_device->buffer_mutex);
	switch (cmd) {
	case VFSSPI_IOCTL_DEVICE_RESET:
		DPRINTK("VFSSPI_IOCTL_DEVICE_RESET:\n");
		vfsspi_hardReset(vfsspi_device);
		break;
	case VFSSPI_IOCTL_DEVICE_SUSPEND:
	{
		DPRINTK("VFSSPI_IOCTL_DEVICE_SUSPEND:\n");
		vfsspi_suspend(vfsspi_device);
		break;
	}
	case VFSSPI_IOCTL_REGISTER_SCREEN_DETECTION_SIGNAL:
	{
		printk("VFSSPI_IOCTL_REGISTER_SCREEN_DETECTION_SIGNAL\n");
		ret_val = vfsspi_register_screen_signal(vfsspi_device, arg);
		break;
	}
	case VFSSPI_IOCTL_RW_SPI_MESSAGE:
//		DPRINTK("VFSSPI_IOCTL_RW_SPI_MESSAGE:");
		ret_val = vfsspi_rw_spi_message(vfsspi_device, arg);
		break;
	case VFSSPI_IOCTL_SET_CLK:
		DPRINTK("VFSSPI_IOCTL_SET_CLK\n");
		ret_val = vfsspi_set_clk(vfsspi_device, arg);
		break;
	case VFSSPI_IOCTL_REGISTER_DRDY_SIGNAL:
		DPRINTK("VFSSPI_IOCTL_REGISTER_DRDY_SIGNAL\n");
		ret_val = vfsspi_register_drdy_signal(vfsspi_device, arg);
		break;
	case VFSSPI_IOCTL_SET_DRDY_INT:
		DPRINTK("VFSSPI_IOCTL_SET_DRDY_INT\n");
		ret_val = vfsspi_set_drdy_int(vfsspi_device, arg);
		break;
	case VFSSPI_IOCTL_SELECT_DRDY_NTF_TYPE:
        {
            vfsspi_iocSelectDrdyNtfType_t drdyTypes;

            DPRINTK("VFSSPI_IOCTL_SELECT_DRDY_NTF_TYPE\n");

            if (copy_from_user(&drdyTypes, (void __user *)arg,
                sizeof(vfsspi_iocSelectDrdyNtfType_t)) != 0) {
                    DPRINTK("copy from user failed.\n");
                    ret_val = -EFAULT;
            } else {
                if (0 != (drdyTypes.supportedTypes & VFSSPI_DRDY_NOTIFY_TYPE_EVENTFD)) {
                    vfsspi_device->drdy_ntf_type = VFSSPI_DRDY_NOTIFY_TYPE_EVENTFD;
                } else {
                    vfsspi_device->drdy_ntf_type = VFSSPI_DRDY_NOTIFY_TYPE_SIGNAL;
                }
                drdyTypes.selectedType = vfsspi_device->drdy_ntf_type;
                if (copy_to_user((void __user *)arg, &(drdyTypes),
                    sizeof(vfsspi_iocSelectDrdyNtfType_t)) == 0) {
                        ret_val = 0;
                } else {
                    DPRINTK("copy to user failed\n");
                }
            }
            break;
        }
	case VFSSPI_IOCTL_GET_SENSOR_ORIENTATION:
	{
		pr_debug("VFSSPI_IOCTL_GET_SENSOR_ORIENTATION\n");
		if ((void __user *)arg != NULL)
		{
			/* Returning appropriate value based on the current position */
			*((unsigned int *) arg) = VFS_SENSOR_ORIENTATION_ULDR;
			ret_val = 0;
		}
		else
		{
			pr_debug("VFSSPI_IOCTL_GET_SENSOR_ORIENTATION failed\n");
			*((unsigned int *)arg) = 0;
			ret_val = -EFAULT;
		}
		break;	
	}

	default:
		ret_val = -EFAULT;
		break;
	}
	mutex_unlock(&vfsspi_device->buffer_mutex);
	return ret_val;
}

static int vfsspi_open(struct inode *inode, struct file *filp)
{
	struct vfsspi_device_data *vfsspi_device = NULL;
	int status = -ENXIO;

	DPRINTK("vfsspi_open\n");

	mutex_lock(&device_list_mutex);

	list_for_each_entry(vfsspi_device, &device_list, device_entry) {
		if (vfsspi_device->devt == inode->i_rdev) {
			status = 0;
			break;
		}
	}

	if (status == 0) {
		mutex_lock(&vfsspi_device->kernel_lock);
		if (vfsspi_device->is_opened != 0) {
			status = -EBUSY;
			DPRINTK("vfsspi_open: is_opened != 0, -EBUSY");
			goto vfsspi_open_out;
		}
		vfsspi_device->user_pid = 0;
        vfsspi_device->drdy_ntf_type = VFSSPI_DRDY_NOTIFY_TYPE_SIGNAL;
		if (vfsspi_device->buffer != NULL) {
			DPRINTK("vfsspi_open: buffer != NULL");
			goto vfsspi_open_out;
		}
		vfsspi_device->null_buffer =
			kmalloc(DEFAULT_BUFFER_SIZE, GFP_KERNEL);
		if (vfsspi_device->null_buffer == NULL) {
			status = -ENOMEM;
			DPRINTK("vfsspi_open: null_buffer == NULL, -ENOMEM");
			goto vfsspi_open_out;
		}
		vfsspi_device->buffer =
			kmalloc(DEFAULT_BUFFER_SIZE, GFP_KERNEL);
		if (vfsspi_device->buffer == NULL) {
			status = -ENOMEM;
			kfree(vfsspi_device->null_buffer);
			DPRINTK("vfsspi_open: buffer == NULL, -ENOMEM");
			goto vfsspi_open_out;
		}
		vfsspi_device->is_opened = 1;
		filp->private_data = vfsspi_device;
		nonseekable_open(inode, filp);

vfsspi_open_out:
		mutex_unlock(&vfsspi_device->kernel_lock);
	}
	
	mutex_unlock(&device_list_mutex);
	return status;
}


static int vfsspi_release(struct inode *inode, struct file *filp)
{
	struct vfsspi_device_data *vfsspi_device = NULL;
	int                   status     = 0;

	DPRINTK("vfsspi_release\n");

	mutex_lock(&device_list_mutex);
	vfsspi_device = filp->private_data;
	filp->private_data = NULL;
	vfsspi_device->is_opened = 0;
	if (vfsspi_device->buffer != NULL) {
		kfree(vfsspi_device->buffer);
		vfsspi_device->buffer = NULL;
	}

	if (vfsspi_device->null_buffer != NULL) {
		kfree(vfsspi_device->null_buffer);
		vfsspi_device->null_buffer = NULL;
	}

	mutex_unlock(&device_list_mutex);
	return status;
}

#ifndef IN_TZ
void PerformReset(void)
{
	pin_select(rst_low);
	mdelay(1);
	pin_select(rst_high);
	mdelay(5);
}

static void vfsspi_test(struct spi_device *spi)
{
	char tx_buf[64] = {1};
	char rx_buf[64] = {0};
	struct spi_transfer t;
	struct spi_message m;
	int i = 0;
	
	DPRINTK("ValiditySensor: Inside spi_probe\n");
	
	tx_buf[0]=1; //EP0 Read
	tx_buf[1]=0;
	/*
	spi->bits_per_word = 8;
	spi->max_speed_hz= 9600000;
	spi->mode = SPI_MODE_0;
	spi_setup(spi);	
	*/
	PerformReset();
	mdelay(5);
	memset(&t, 0, sizeof(t));
	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	t.len = 6;
	//spi_setup(spi);
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	
	DPRINTK("Sensor: spi_sync returned %d \n", spi_sync(spi, &m));
	
	for(i=0;i<6;i++)
	printk("vfsspi Sensor:%0x\n",rx_buf[i]);
		
	mdelay(10);
	
	tx_buf[0]=1; //EP0 read
	tx_buf[1]=0;
	
	memset(&t, 0, sizeof(t));
	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	t.len = 6;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	
	DPRINTK("ValiditySensor: spi_sync returned %d \n", spi_sync(spi, &m));
	
	for(i=0;i<6;i++)
	printk("vfsspi Sensor:%0x\n",rx_buf[i]);
		
	mdelay(5);
	
	tx_buf[0]=2; //Getver command on EP1OUT
	tx_buf[1]=1;
	memset(&t, 0, sizeof(t));
	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	t.len = 2;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
  	printk("vfsspi Sensor: spi_sync returned %d \n", spi_sync(spi, &m)); 
	
	mdelay(5);
	tx_buf[0]=3; //Read Gerver command reply
	tx_buf[1]=0;
	
	memset(&t, 0, sizeof(t));
	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	t.len = 40;
	spi_message_init(&m);
	spi_message_add_tail(&t, &m);
	
	printk("vfsspi sensor: spi_sync returned %d \n", spi_sync(spi, &m));
	
	for(i=0;i<40;i++) {
		printk("vfsspi Sensor:%0x\n",rx_buf[i]);
		if ((rx_buf[i] !=0) && (rx_buf[i] !=0xff))
			vfs_finger_detected = 1;
	}
}
#endif

/* file operations associated with device */
static const struct file_operations vfsspi_fops = {
	.owner   = THIS_MODULE,
	.write   = vfsspi_write,
	.read    = vfsspi_read,
	.unlocked_ioctl   = vfsspi_ioctl,
	.open    = vfsspi_open,
	.release = vfsspi_release,
};

ssize_t debug_vfs_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
	//vfsspi_hardReset(g_vfsSpiDev);
	printk("irq: %d\n" + g_vfsSpiDev->is_drdy_irq_enabled);
	return 0;
}

static long vfs_ioctl(struct file *filp, unsigned int cmd,
			unsigned long arg)
{
	int __user *argp = (int __user *)arg;
	int val = 0;
	switch(cmd) {
		case VFS_IOCTL_DEVICE_GETINT:
			if(g_vfsSpiDev != NULL)
			{
				val = gpio_get_value(g_vfsSpiDev->drdy_pin);
				put_user(val, argp);
			}
			break;
		default:
			break;
	}
	return 0;
}

struct file_operations file_vfs = {
	.owner = THIS_MODULE,
	.read = debug_vfs_read,
	.unlocked_ioctl   = vfs_ioctl,
};

static struct miscdevice vfs_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vfs",
	.fops = &file_vfs,
};

static ssize_t fp_proc_read_val(struct file *file,
		char __user *buffer, size_t count, loff_t *offset)
{
	ssize_t len = 0; 
	char data[80];

	len += sprintf(data + len, "IC:synaptics.\n");

	return simple_read_from_buffer(buffer, count, offset, data, len);
}

static ssize_t fp_proc_write_val(struct file *filp,
		const char *buff, size_t len, 
		loff_t * off) 
{
	return len; 
}

static struct file_operations fp_proc_ops = {
	.read = fp_proc_read_val,
	.write = fp_proc_write_val,
};

static void create_fp_proc_entry(void)
{
	struct proc_dir_entry *fp_proc_entry;
	fp_proc_entry = proc_create("driver/fingerprint_id", 0644, NULL, &fp_proc_ops);
	if (fp_proc_entry) {
		printk(KERN_INFO "create vfsspi proc file sucess!\n");
	} else 
		printk(KERN_INFO "create vfsspi proc file failed!\n");
}

static int vfsspi_probe(struct spi_device *spi)
{
	int status = 0;
	struct vfsspi_device_data *vfsspi_device;
	struct device *dev;

	DPRINTK("vfsspi_probe\n");

    pin_select(pwr_on);

	vfsspi_device = kzalloc(sizeof(*vfsspi_device), GFP_KERNEL);

	if (vfsspi_device == NULL)
		return -ENOMEM;

	/* Initialize driver data. */
	vfsspi_device->current_spi_speed = SLOW_BAUD_RATE;
	vfsspi_device->spi = spi;

	vfsspi_device->spi->controller_data = (void*)&spi_conf_mt65xx;

	g_vfsSpiDev = vfsspi_device;

	spin_lock_init(&vfsspi_device->vfs_spi_lock);
	mutex_init(&vfsspi_device->buffer_mutex);
	mutex_init(&vfsspi_device->kernel_lock);

	INIT_LIST_HEAD(&vfsspi_device->device_entry);

	if (vfsspi_device == NULL) {
		status = -EFAULT;
		goto vfsspi_probe_drdy_failed;
	}

	pin_select(rst_high);
	pin_select(irq_en);
#ifdef WOF	
	//vfsspi_device->sleep_pin = VFSSPI_SLEEP_PIN;
    vfsspi_device->wake_lock_acquired = 0;
    wake_lock_init(&vfsspi_device->wake_lock, WAKE_LOCK_SUSPEND, "fingerprint_wakelock");

    /* init work queue for interrupt */
    INIT_WORK(&vfsspi_device->irq_worker, vfsspi_irq_worker);
    init_timer(&vfsspi_device->wake_unlock_timer);
    vfsspi_device->wake_unlock_timer.expires = jiffies - 1;
    vfsspi_device->wake_unlock_timer.function = vfsspi_wake_unlock_timer_handler;
    vfsspi_device->wake_unlock_timer.data = (unsigned long)vfsspi_device;
    add_timer(&vfsspi_device->wake_unlock_timer);
#endif

	vfsspi_device->is_drdy_irq_enabled = DRDY_IRQ_ENABLE;
	spi->bits_per_word = BITS_PER_WORD;
	spi->max_speed_hz = MAX_BAUD_RATE;
	spi->mode = SPI_MODE_0;

	status = spi_setup(spi);
	if (status != 0){
		printk("vfsspi_probe: spi_setup fail\n");
		goto vfsspi_probe_setup_failed;
	}

	vfsspi_test(spi);
	if (!vfs_finger_detected) {
		DPRINTK("vfsspi test failed\n");
		status = -ENODEV;
		goto vfsspi_probe_setup_failed;
	}
	
	vfsspi_device->drdy_pin = get_gpio(gpio);
	gpio_irq = get_gpio(num);
	if (gpio_irq <= 0) {
		pr_err("gpio_to_drdy DRDY failed\n");
		status = -EBUSY;
		goto vfsspi_probe_setup_failed;
	}

	if (request_irq(gpio_irq, vfsspi_irq, IRQF_TRIGGER_RISING,
			"vfsspi_irq", vfsspi_device) < 0) {
		DPRINTK("request_irq failed\n");
		status = -EBUSY;
		goto vfsspi_probe_irq_failed;
	}

	mutex_lock(&device_list_mutex);
	/* Create device node */
	/* register major number for character device */
	status = alloc_chrdev_region(&(vfsspi_device->devt),
				     0, 1, VALIDITY_PART_NAME);
	if (status < 0) {
		DPRINTK("alloc_chrdev_region failed\n");
		goto vfsspi_probe_alloc_chardev_failed;
	}

	cdev_init(&(vfsspi_device->cdev), &vfsspi_fops);
	vfsspi_device->cdev.owner = THIS_MODULE;
	status = cdev_add(&(vfsspi_device->cdev), vfsspi_device->devt, 1);
	if (status < 0) {
		DPRINTK("cdev_add failed\n");
		unregister_chrdev_region(vfsspi_device->devt, 1);
		goto vfsspi_probe_cdev_add_failed;
	}

	vfsspi_device_class = class_create(THIS_MODULE, "validity_fingerprint");

	if (IS_ERR(vfsspi_device_class)) {
		DPRINTK("vfsspi_init: class_create() is failed - unregister chrdev.\n");
		cdev_del(&(vfsspi_device->cdev));
		unregister_chrdev_region(vfsspi_device->devt, 1);
		status = PTR_ERR(vfsspi_device_class);
		goto vfsspi_probe_class_create_failed;
	}

    // register receiver for FB event
	vfsspi_device->fb_notifier = SYNA_notify_block;
	fb_register_client(&vfsspi_device->fb_notifier);
	// register receiver for FB event
	dev = device_create(vfsspi_device_class, &spi->dev,
			    vfsspi_device->devt, vfsspi_device, "vfsspi");
	status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	if (status == 0)
		list_add(&vfsspi_device->device_entry, &device_list);
	device_create_file(dev, &dev_attr_screenState);
		mutex_unlock(&device_list_mutex);

	if (status != 0){
        printk("%s: vfsspi create status: %d\n", __func__, (int)status);
		goto vfsspi_probe_device_failed;
	}

#ifndef IN_TZ
	spi_set_drvdata(spi, vfsspi_device);
	misc_register(&vfs_misc);
#else
    platform_set_drvdata(spi, vfsspi_device);
#endif
	create_fp_proc_entry();
	DPRINTK("vfsspi_probe successful");

	return 0;

vfsspi_probe_device_failed:
	class_destroy(vfsspi_device_class);
vfsspi_probe_class_create_failed:
	cdev_del(&(vfsspi_device->cdev));
vfsspi_probe_cdev_add_failed:
	unregister_chrdev_region(vfsspi_device->devt, 1);
vfsspi_probe_alloc_chardev_failed:
	free_irq(gpio_irq, vfsspi_device);
vfsspi_probe_irq_failed:
vfsspi_probe_setup_failed:
#ifdef WOF
	wake_lock_destroy(&vfsspi_device->wake_lock);
    del_timer(&vfsspi_device->wake_unlock_timer);
#endif
#if DO_CHIP_SELECT
vfsspi_probe_cs_failed:
#endif
vfsspi_probe_drdy_failed:
	mutex_destroy(&vfsspi_device->buffer_mutex);
	mutex_destroy(&vfsspi_device->kernel_lock);
	kfree(vfsspi_device);
	DPRINTK("vfsspi_probe failed!!\n");
	return status;
}

static int vfsspi_remove(struct spi_device *spi)
{
	int status = 0;

	struct vfsspi_device_data *vfsspi_device = NULL;

	DPRINTK("vfsspi_remove\n");

	vfsspi_device = spi_get_drvdata(spi);

	if (vfsspi_device != NULL) {
		spin_lock_irq(&vfsspi_device->vfs_spi_lock);
		vfsspi_device->spi = NULL;
		spi_set_drvdata(spi, NULL);
		spin_unlock_irq(&vfsspi_device->vfs_spi_lock);

		mutex_lock(&device_list_mutex);

		/* Remove device entry. */
		list_del(&vfsspi_device->device_entry);
		device_destroy(vfsspi_device_class, vfsspi_device->devt);
		class_destroy(vfsspi_device_class);
		cdev_del(&(vfsspi_device->cdev));
		unregister_chrdev_region(vfsspi_device->devt, 1);

		mutex_destroy(&vfsspi_device->buffer_mutex);
		mutex_destroy(&vfsspi_device->kernel_lock);
#ifdef WOF
		wake_lock_destroy(&vfsspi_device->wake_lock);
    	del_timer(&vfsspi_device->wake_unlock_timer);
#endif
		
		kfree(vfsspi_device);
		mutex_unlock(&device_list_mutex);
		misc_deregister(&vfs_misc);
	}

	return status;
}

struct spi_driver vfsspi_spi = {
	.driver = {
		//.name  = VALIDITY_PART_NAME,
		.name  = "fingerprint_spi",
		.owner = THIS_MODULE,
		//.of_match_table = validity_metallica_table,
	},
		.probe  = vfsspi_probe,
		.remove = vfsspi_remove,
};

static int __init vfsspi_init(void)
{
	int status = 0;

	DPRINTK("vfsspi_init\n");
	
	status = spi_register_driver(&vfsspi_spi);
	if (status < 0) {
		DPRINTK("vfsspi_init: spi_register_driver() is failed - unregister chrdev.\n");
	}
	if (!vfs_finger_detected){
		spi_unregister_driver(&vfsspi_spi);
		DPRINTK("init is failed\n");
	} else
		DPRINTK("init is successful status = %d\n", status);

	return status;
}

static void __exit vfsspi_exit(void)
{
	DPRINTK("vfsspi_exit\n");
	spi_unregister_driver(&vfsspi_spi);
}

module_init(vfsspi_init);
module_exit(vfsspi_exit);

MODULE_DESCRIPTION("Validity FPS sensor");
MODULE_LICENSE("GPL");
