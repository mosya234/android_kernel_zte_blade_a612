#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/ioctl.h>
#include <linux/input.h>   
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/poll.h>
#include <linux/sort.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/spi/spidev.h>
#include <linux/spi/spi.h>
#include <linux/usb.h>
#include <linux/usb/ulpi.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/power_supply.h>
#include <asm/uaccess.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/of_irq.h>
#include <linux/completion.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <asm/io.h>
#include <linux/platform_data/spi-mt65xx.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/notifier.h>
#include <linux/fb.h>
#include <linux/proc_fs.h>

#include "gf-common.h"
#include "gf-milanf.h"
#include "gf-regs.h"
#include "fingerprint_core.h"

#if CONFIG_HAS_EARLYSUSPEND
static int suspend_flag = 0;
#endif

/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */
static struct class *gf_spi_class;

/*************************data stream***********************
 *	FRAME NO  | RING_H  | RING_L  |  DATA STREAM | CHECKSUM |
 *     1B      |   1B    |  1B     |    2048B     |  2B      |
 ************************************************************/
static unsigned bufsiz = 14260;
static unsigned char g_frame_buf[14260]={0};

static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static gf_dev_t gf;
static unsigned char g_sendorID[16] = {0};
static unsigned short g_fdt_delta = 0;
static unsigned short g_tcode_img = 0;
static int g_irq_enabled = 1;

module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");
int gf_write_configs(gf_dev_t *gf_dev,struct gf_configs config[],int len);
unsigned int fingerprint_irq = 0;

#if defined(CONFIG_FINGERPRINT_GSL6163E_A36)
extern int gsl6163e_detected;
#endif
int gf3208_detected = 0;

#if defined(CONFIG_OF)
struct platform_device *fingerprint_device = NULL;
#endif

struct gf_key_map key_map[] =
{
      {  "POWER",  KEY_POWER  },
      {  "HOME" ,  KEY_HOME   },
      {  "MENU" ,  KEY_MENU   },
      {  "BACK" ,  KEY_BACK   },
      {  "UP"   ,  KEY_UP     },
      {  "DOWN" ,  KEY_DOWN   },
      {  "LEFT" ,  KEY_LEFT   },
      {  "RIGHT",  KEY_RIGHT  },
      {  "FORCE",  KEY_F9     },
      {  "CLICK",  KEY_F19    },
};

/************* Configs Definition **********/
struct gf_configs normal_cfg[]=        
{
	{GF_FDT_AREA_NUM,0x0000},
	{GF_MODE_CTRL0,0x4010},/*enable osr and wake up timer*/
	{GF_MODE_CTRL2,0x0080},
	{GF_PIXEL_CTRL3,0x2808},
	{GF_PIXEL_CTRL4,0x0148},
	{GF_CMP_NUM_CTRL,0x0000},
	{GF_FDT_DELTA,0x0C7F},
	{GF_FDT_THR0,0x817F},
	{GF_FDT_THR1,0x8C7F},
	{GF_FDT_THR2,0x977F},
	{GF_FDT_THR3,0xB07F},
	{GF_FDT_THR4,0x867F},
	{GF_FDT_THR5,0x8C7F},
	{GF_FDT_THR6,0xA07F},
	{GF_FDT_THR7,0xB37F},
	{GF_FDT_THR8,0x847F},
	{GF_FDT_THR9,0x887F},
	{GF_FDT_THR10,0xA07F},
	{GF_FDT_THR11,0xB87F},
	{GF_ENCRYPT_EN,0x0001}, /*default do not encrypt*/
};

struct gf_configs fdt_down_cfg[]=
{
	{GF_IRQ_CTRL0, 0x0482}, /*enable fdt INT*/
	{GF_MODE_CTRL0,0x4010},
	{GF_MODE_CTRL1,0x2001}, 
	{GF_MODE_CTRL2,0x0014}, 
	{GF_PIXEL_CTRL6,0x0100},
	{GF_FDT_DELTA,0x157F},
	{GF_FDT_AREA_NUM,0x0007},
	{GF_FDT,0x0401},        /*fdt enabel, detect down*/

};

struct gf_configs nav_fdt_down_cfg[]=
{
	{GF_IRQ_CTRL0, 0x0482}, /*enable fdt INT*/
	{GF_MODE_CTRL0,0x4010},
	{GF_MODE_CTRL1,0x2001}, 
	{GF_MODE_CTRL2,0x0014}, 
	{GF_PIXEL_CTRL6,0x0100},
	{GF_FDT_DELTA,0x157F},
	{GF_FDT_AREA_NUM,0x0000},
	{GF_FDT,0x0001},        /*fdt enabel, detect down*/
};

struct gf_configs fdt_up_cfg[]=
{
	{GF_IRQ_CTRL0, 0x0482}, /*enable fdt INT*/
	{GF_MODE_CTRL0,0x4010},
	{GF_MODE_CTRL1,0x2001}, 
	{GF_MODE_CTRL2,0x0014}, 
	{GF_PIXEL_CTRL6,0x0100}, 
	{GF_FDT_DELTA,0x137F},
	{GF_FDT,0x0003},        /*fdt enabel, detect up*/
};

struct gf_configs ff_cfg[]=
{
	{GF_IRQ_CTRL0, 0x0402}, /*enable fdt INT*/
	{GF_MODE_CTRL0,0x4010},	
	{GF_MODE_CTRL1,0x2001}, 
	{GF_MODE_CTRL2,0x0032}, 
	{GF_PIXEL_CTRL6,0x0100}, 
	{GF_FDT_DELTA,0x157F},
	{GF_FDT,0x0001},        /*fdt enabel, detect down*/	
};

struct gf_configs nav_cfg[]=
{
	{GF_IRQ_CTRL0, 0x0408}, /*enable data_int INT*/
	{GF_MODE_CTRL1,0x0810}, /*manual mode*/
	{GF_PIXEL_CTRL6,0x0100},
	{GF_FDT,0x0001},        /*fdt enabel, detect down*/
};

struct gf_configs nav_img_cfg[]=
{
	{GF_PIXEL_CTRL6,0x0080},       /*set rate 128*/
	{GF_IRQ_CTRL0,0x0408},         /*enable data_int INT*/
	{GF_PIXEL_CTRL1,0x0008},       /*set one_frame mode*/
	{GF_PIXEL_CTRL0,0x0501},	
};

struct gf_configs img_cfg[]=
{
	{GF_PIXEL_CTRL6,0x0100},       
	{GF_IRQ_CTRL0,0x0408},         /*enable data_int INT*/
	{GF_PIXEL_CTRL1,0x0008},       /*set one_frame mode*/
	{GF_PIXEL_CTRL0,0x0501},	
};

struct gf_configs nav_img_manual_cfg[]=
{
	{GF_PIXEL_CTRL6,0x0080},
	{GF_IRQ_CTRL0,0x0408},
	{GF_PIXEL_CTRL1,0x0004}, /*Manual Mode Enable*/
	{GF_PIXEL_CTRL0,0x0501},
	{GF_PIXEL_CTRL2,0x0100}, /*Shadow Mode Select*/ 
};

static void gf_enable_irq(gf_dev_t* gf_dev)
{
	if(g_irq_enabled) {
		gf_debug(DEFAULT_DEBUG,"%s irq has been enabled.",__func__);
	} else {
		gf_debug(DEFAULT_DEBUG,"%s called.",__func__);
		enable_irq(fingerprint_irq);
		g_irq_enabled = 1;
	}
}

static void gf_disable_irq(gf_dev_t* gf_dev)
{
	if(g_irq_enabled) {
		g_irq_enabled = 0;
		gf_debug(DEFAULT_DEBUG,"%s called.",__func__);
		disable_irq(fingerprint_irq);
	} else {
		gf_debug(DEFAULT_DEBUG,"%s irq has been disabled.",__func__);
	}
}

void readOTPByte(gf_dev_t* gf_dev,unsigned short usBank,
					unsigned short usAddr,unsigned char* pValue)
{
	unsigned short data = 0;
	gf_spi_write_word(gf_dev,GF_OTP_ADDR1,0x0020);
	gf_spi_write_word(gf_dev,GF_OTP_ADDR2,usBank);
	gf_spi_write_word(gf_dev,GF_OTP_ADDR1,0x0021);
	gf_spi_write_word(gf_dev,GF_OTP_ADDR1,0x0020);

	gf_spi_write_word(gf_dev,GF_OTP_ADDR2,((usAddr<<8) & (0xFFFF)));
	gf_spi_write_word(gf_dev,GF_OTP_ADDR1,0x0022);
	gf_spi_write_word(gf_dev,GF_OTP_ADDR1,0x0020);
	gf_spi_read_word(gf_dev,GF_OTP_ADDR3,&data);

	*pValue = data & 0xFF;
}

void gf_parse_otp(gf_dev_t* gf_dev,unsigned char* pSensorID)
{
	unsigned short i = 0;
	unsigned char ucOTP[32] = {0};
	unsigned char Tcode,Diff;
	unsigned short Tcode_use,Diff_use,Diff_256;
	
	for(i=0;i<32;i++)
	{
		readOTPByte(gf_dev,(i>>2),(i & 0x03),ucOTP+i);
		gf_debug(DEFAULT_DEBUG,"OTP_BYTE[%d] = 0x%x ",i,ucOTP[i]);
	}
			
	memcpy(pSensorID,ucOTP,16);
	Tcode = (ucOTP[22] & 0xF0) >> 4;
	Diff = ucOTP[22] & 0x0F;

	if((Tcode != 0) && (Diff != 0) && ((ucOTP[22] & ucOTP[23]) == 0)) 
	{
		Tcode_use = (Tcode + 1)*16;
		Diff_use = (Diff + 2)*100;

		Diff_256 = (Diff_use * 256) / Tcode_use;
		gf_debug(DEFAULT_DEBUG,"%s Tcode:0x%x(%d),Diff:0x%x(%d)",__func__,Tcode,Tcode,Diff,Diff);
		gf_debug(DEFAULT_DEBUG,"%s Tcode_use:0x%x(%d),Diff_use:0x%x(%d),Diff_256:0x%x(%d)",
			        __func__,Tcode_use,Tcode_use,Diff_use,Diff_use,Diff_256,Diff_256);
		g_fdt_delta = (((Diff_256/3)>>4)<<8)|0x7F;
		g_tcode_img = Tcode_use;		
	}
	gf_debug(DEFAULT_DEBUG,"%s g_tcode_img:0x%x,g_fdt_delta:0x%x",__func__,g_tcode_img,g_fdt_delta);
} 

static int gf_reg_key_kernel(gf_dev_t *gf_dev)
{
	int i;
	
	set_bit(EV_KEY, gf_dev->input->evbit); //tell the kernel is key event
	for(i = 0; i< ARRAY_SIZE(key_map); i++) {
		set_bit(key_map[i].val, gf_dev->input->keybit);
	}

	gf_dev->input->name = GF_INPUT_NAME;
	if (input_register_device(gf_dev->input)){
		gf_error("Failed to register GF as input device.");
		return -1;
	}
	return 0;
}

void print_16hex(u8 *config, u8 len)
{
	u8 i, j = 0;
	
	gf_debug(DEFAULT_DEBUG,"dump hex ");
	for(i = 0 ; i< len ; i++) {
		gf_debug(DEFAULT_DEBUG,"0x%x " , config[i]);
		if(j++ == 15) {
			j = 0;
		}
	} 
}

/********************************************************************
*CPU output low level in RST pin to reset GF. This is the MUST action for GF.
*Take care of this function. IO Pin driver strength / glitch and so on.
********************************************************************/
inline static void gf_hw_reset(gf_dev_t *gf_dev, int ms)
{	
	gf_debug(DEFAULT_DEBUG, "gf_hw_reset");
	pin_select(rst_low);
	mdelay(3);
	pin_select(rst_high);
}

/*********************************************************
**Power control
**function: hwPowerOn   hwPowerDown
*********************************************************/
int gf_power_on(gf_dev_t *gf_dev, bool onoff)
{
	gf_debug(DEFAULT_DEBUG,"%s onoff = %d", __func__, onoff);
	if(onoff) {
		if (0 == gf_dev->poweron){
			/*INT GPIO Pull-down before power on*/
			//pinctrl_select_state(pinctrl1, eint_pulldown);

			/*Reset GPIO Output-low before poweron*/
			pin_select(rst_low);
			msleep(5);

			pin_select(pwr_on);
			gf_dev->poweron = 1;
			
			msleep(20);

			/*INT GPIO set floating after poweron and controlled by GF*/
			//pinctrl_select_state(pinctrl1, eint_pulldisable);
			msleep(5);
			
			/*Reset GPIO Output-high, GF works*/
			pin_select(rst_high);
			msleep(60);
		}
	} else {
		if (1 == gf_dev->poweron){
			//pinctrl_select_state(pinctrl1, eint_pulldown);
			pin_select(rst_low);

			msleep(10);
			pin_select(pwr_off);
			gf_dev->poweron = 0;
			msleep(50);
		}
	}
	return 0;
}

static ssize_t gf_debug_show(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	return (sprintf(buf, "%d\n", g_debug_level));
}

static ssize_t gf_debug_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int g_debug = 0;
	sscanf(buf, "%d", &g_debug);
	return count;
}

static DEVICE_ATTR(debug, S_IRUGO|S_IWUSR, gf_debug_show, gf_debug_store);

static ssize_t gf_chipid_show(struct device *dev,   
		    struct device_attribute *attr, char *buf)
{   
	return sprintf(buf, "%d\n", gf3208_detected); 
}

static DEVICE_ATTR(chipid, S_IRUGO, gf_chipid_show, NULL);

static struct attribute *gf_debug_attrs[] = {
	&dev_attr_debug.attr,
	&dev_attr_chipid.attr,
	NULL
};

static const struct attribute_group gf_debug_attr_group = {
	.attrs = gf_debug_attrs,
	.name = "debug"
};

int gf_write_configs(gf_dev_t *gf_dev,struct gf_configs config[],int len)
{
	int cnt;
	int length = len;
	int ret = 0;
	for(cnt=0;cnt<length;cnt++)
	{
		gf_debug(DEFAULT_DEBUG,"addr = 0x%x, value = 0x%x",config[cnt].addr,config[cnt].value);
		ret = gf_spi_write_word(gf_dev,config[cnt].addr,config[cnt].value);
		if(ret < 0){
			gf_error("%s failed. ",__func__);
			return ret;
		}
	}

	return 0;
}

static ssize_t gf_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
 	gf_dev_t *gf_dev = filp->private_data;
	int status = 0;
	int len = 0;

	if(buf == NULL || count > bufsiz)
	{
		return -EMSGSIZE;
	}

	len = gf_spi_read_data(gf_dev,0xAAAA,count,g_frame_buf);
	status = copy_to_user(buf, g_frame_buf, count);
	if(status != 0) {
		gf_error("%s copy_to_user failed. status = %d ",__func__,status);
		return -EFAULT;
	}
    return 0;
}

static ssize_t gf_write(struct file *filp, const char __user *buf,
	size_t count, loff_t *f_pos)
{
	return 0;
}

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	gf_dev_t *gf_dev = (gf_dev_t *)filp->private_data;
	struct gf_ioc_transfer *ioc = NULL;
	struct gf_key gf_key={0};
	u8* tmpbuf = NULL; 
	int ret = 0;
	int retval = 0;
	int err = 0;
	unsigned char command = 0;
	unsigned char config_type = 0;
	int i;

	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
		return -ENOTTY;

    /* Check access direction once here; don't repeat below.
     * IOC_DIR is from the user perspective, while access_ok is
     * from the kernel perspective; so they look reversed.
     */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;
	
	FUNC_ENTRY();
	switch(cmd) {
	case GF_IOC_RW:
		ioc = kzalloc(sizeof(*ioc), GFP_KERNEL);
		if (ioc == NULL) {
			gf_error("kzalloc ioc failed.");
			retval = -ENOMEM;
			break;
		}
		/*copy command data from user to kernel.*/
		if(copy_from_user(ioc, (struct gf_ioc_transfer*)arg, sizeof(*ioc))){
			gf_error("Failed to copy command from user to kernel.");
			retval = -EFAULT;
			break;
		}
		tmpbuf = kzalloc(ioc->len, GFP_KERNEL);
		if (tmpbuf == NULL) {
			gf_error("kzalloc tmpbuf failed.");
			retval = -ENOMEM;
			break;			
		}
		if((ioc->len > bufsiz)||(ioc->len == 0)) {
			gf_error("The request length[%d] is longer than supported maximum buffer length[%d].", 
					ioc->len, bufsiz);
			retval = -EMSGSIZE;
			break;
		}

		if(ioc->cmd == GF_R) {
			/*if want to read data from hardware.*/
			mutex_lock(&gf_dev->frame_lock);
			gf_spi_read_data(gf_dev, ioc->addr, ioc->len, tmpbuf);
			mutex_unlock(&gf_dev->frame_lock);

			mutex_lock(&gf_dev->buf_lock);
#if PROCESSOR_64_BIT
			ret = copy_to_user((void __user*)((unsigned long)ioc->buf), tmpbuf, ioc->len);
#else	
			ret = copy_to_user(ioc->buf, gf_dev->buffer + GF_RDATA_OFFSET, ioc->len);
#endif
			mutex_unlock(&gf_dev->buf_lock);

			if(ret) {
				gf_error("Failed to copy data from kernel to user.");
				retval = -EFAULT;
				break;
			}
		} else if (ioc->cmd == GF_W) {
			/*if want to read data from hardware.*/
			gf_debug(FLOW_DEBUG,"gf_ioctl Write data to 0x%x, len = 0x%x", ioc->addr, ioc->len);
#if PROCESSOR_64_BIT			
			ret = copy_from_user(tmpbuf, (void __user*)((unsigned long) ioc->buf), ioc->len);
#else			
			ret = copy_from_user(gf_dev->buffer + GF_WDATA_OFFSET, ioc->buf, ioc->len);
#endif
			if(ret){
				gf_error("Failed to copy data from user to kernel.");
				retval = -EFAULT;
				break;
			}
			
			mutex_lock(&gf_dev->frame_lock);
			gf_spi_write_data(gf_dev, ioc->addr, ioc->len, tmpbuf);
			mutex_unlock(&gf_dev->frame_lock);
		}
		else {
			gf_error("Error command for gf_ioctl.");
			retval = -EFAULT;
		}
	break;
	case GF_IOC_CMD:
		retval = __get_user(command ,(u32 __user*)arg);
		mutex_lock(&gf_dev->buf_lock);
		gf_spi_send_cmd(gf_dev,&command,1);
		mdelay(2);
		mutex_unlock(&gf_dev->buf_lock);
	break;
	case GF_IOC_CONFIG:
		retval = __get_user(config_type, (u32 __user*)arg);
		gf_debug(DEFAULT_DEBUG, "config type %d", config_type);
		if(config_type == CONFIG_FDT_DOWN){
			gf_write_configs(gf_dev,fdt_down_cfg,sizeof(fdt_down_cfg)/sizeof(struct gf_configs));
			break;
		}
		else if(config_type == CONFIG_FDT_UP){
			gf_write_configs(gf_dev,fdt_up_cfg,sizeof(fdt_up_cfg)/sizeof(struct gf_configs));
			break;
		}
		else if(config_type == CONFIG_FF){
			gf_write_configs(gf_dev,ff_cfg,sizeof(ff_cfg)/sizeof(struct gf_configs));
			break;
		}
		else if(config_type == CONFIG_NAV){
			gf_write_configs(gf_dev,nav_cfg,sizeof(nav_cfg)/sizeof(struct gf_configs));
			break;
		}
		else if(config_type == CONFIG_IMG){
			gf_write_configs(gf_dev,img_cfg,sizeof(img_cfg)/sizeof(struct gf_configs));
			break;
		}else if(config_type == CONFIG_NAV_IMG){			
			gf_write_configs(gf_dev,nav_img_cfg,sizeof(nav_img_cfg)/sizeof(struct gf_configs));
			break;
		}else if(config_type == CONFIG_NAV_IMG_MAN) {
			gf_write_configs(gf_dev,nav_img_manual_cfg,sizeof(nav_img_manual_cfg)/sizeof(struct gf_configs));
			break;
		}else if(config_type == CONFIG_NAV_FDT_DOWN) {
			gf_write_configs(gf_dev,nav_fdt_down_cfg,sizeof(nav_fdt_down_cfg)/sizeof(struct gf_configs));			
			break;
		}
		else{
			gf_debug(DEFAULT_DEBUG, "%s unknow config_type is %d ",__func__,config_type);
			break;
		}
	case GF_IOC_RESET:
		gf_hw_reset(gf_dev,0);
		gf_debug(DEFAULT_DEBUG, "%s GF_IOC_RESET ",__func__);
	break;
	case GF_IOC_ENABLE_IRQ:
		gf_debug(DEFAULT_DEBUG, "%s ++++++++++++ GF_IOC_ENABLE_IRQ ",__func__);
		gf_enable_irq(gf_dev);
	break;
	case GF_IOC_DISABLE_IRQ:
		gf_debug(DEFAULT_DEBUG, "%s ------------ GF_IOC_DISABLE_IRQ ",__func__);
		gf_disable_irq(gf_dev);
	break;
	case GF_IOC_SENDKEY:
		gf_debug(DEFAULT_DEBUG, "%s GF_IOC_SENDKEY.",__func__);
		if(copy_from_user(&gf_key,(struct gf_key*)arg, sizeof(struct gf_key))) {
			gf_debug(DEFAULT_DEBUG, "%s GF_IOC_SENDKEY failed to copy data from user.",__func__);
			retval = -EFAULT;
			break;
		}
		for(i = 0; i< ARRAY_SIZE(key_map); i++) {
			if(key_map[i].val == gf_key.key){
				input_report_key(gf_dev->input, gf_key.key, gf_key.value);
				input_sync(gf_dev->input);
				gf_debug(DEFAULT_DEBUG, "report key %s", key_map[i].name);
				break;
			}
		}

		if(i == ARRAY_SIZE(key_map)) {
			gf_error("key %d not support yet ", gf_key.key);
			retval = -EFAULT;
		}
	break;
	default:
		gf_error("%s gf doesn't support this command.",__func__);
		gf_error("%s CMD = 0x%x,_IOC_DIR:0x%x,_IOC_TYPE:0x%x,IOC_NR:0x%x,IOC_SIZE:0x%x",
					__func__,cmd,_IOC_DIR(cmd),_IOC_TYPE(cmd),_IOC_NR(cmd),_IOC_SIZE(cmd));
		retval = -EFAULT;
	break;
	}
	
	FUNC_EXIT();
	if(tmpbuf != NULL){
		kfree(tmpbuf);
		tmpbuf = NULL;
	}
	if(ioc != NULL) {
		kfree(ioc);
		ioc = NULL;
	}
    return retval;
}

static unsigned int gf_poll(struct file *filp, struct poll_table_struct *wait)
{
	return 0;
}

#if CONFIG_HAS_EARLYSUSPEND
static void gf_early_suspend(struct early_suspend *h)
{    
	gf_dev_t *gf_dev = container_of(h, gf_dev_t, early_fp);
	gf_debug(DEFAULT_DEBUG,"gf  suspend.");
	suspend_flag = 1;
}

static void gf_late_resume(struct early_suspend *h)
{
	gf_dev_t *gf_dev = container_of(h, gf_dev_t, early_fp);	
	gf_debug(DEFAULT_DEBUG,"gf  resume");

	suspend_flag = 0;
}
#endif

/*******************************************
**Interrupter
**
*******************************************/
static irqreturn_t fingerprint_eint_interrupt_handler(unsigned irq, struct irq_desc *desc)
{
	gf_dev_t *gf_dev = &gf;
#if GF_FASYNC
	if(gf_dev->async) {
		gf_debug(DEFAULT_DEBUG,"async ");
		kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
	}
#endif
	return IRQ_HANDLED;
}

#if GF_FASYNC
static int gf_fasync(int fd, struct file *filp, int mode)
{
	gf_dev_t *gf_dev = filp->private_data;
	int ret;
	FUNC_ENTRY();

	ret = fasync_helper(fd, filp, mode, &gf_dev->async);
	FUNC_EXIT();
	return ret;
}
#endif

static int gf_open(struct inode *inode, struct file *filp)
{
	gf_dev_t *gf_dev;
	int status = -ENXIO;
	int cnt = 0;
	unsigned short reg = 0;
	FUNC_ENTRY();
	
	mutex_lock(&device_list_lock);

	list_for_each_entry(gf_dev, &device_list, device_entry) {
		if(gf_dev->devt == inode->i_rdev) {
			gf_debug(DEFAULT_DEBUG, "Found");
			status = 0;
			break;
		}
	}

	if(status == 0){
		mutex_lock(&gf_dev->buf_lock);
		if( gf_dev->buffer == NULL) {
			gf_dev->buffer = kzalloc(bufsiz + GF_RDATA_OFFSET, GFP_KERNEL);
			if(gf_dev->buffer == NULL) {
				gf_error("open/ENOMEM");
				status = -ENOMEM;
			}
		}
		mutex_unlock(&gf_dev->buf_lock);

		if(status == 0) {
			gf_dev->users++;
			filp->private_data = gf_dev;

			/* 1. disable irq */
			gf_disable_irq(gf_dev);
			/* 2. set chip to init state */
			gf_hw_reset(gf_dev,0); 
			while(cnt < 10){
				gf_spi_read_word(gf_dev,GF_IRQ_CTRL3,&reg);
				gf_debug(DEFAULT_DEBUG,"%s IRQ status : 0x%x ,cnt = %d",__func__,reg,cnt);
				if(reg == 0x0100) {
					gf_spi_write_word(gf_dev,GF_IRQ_CTRL2,0x0100);
					break;
				}
				cnt++;
			}
			/* 3. parse otp */
			gf_parse_otp(gf_dev,g_sendorID);

			if(g_tcode_img!=0 && g_fdt_delta!=0)
			{
				fdt_down_cfg[5].value = g_fdt_delta;
				ff_cfg[5].value = g_fdt_delta;
				nav_fdt_down_cfg[5].value = g_fdt_delta;
				fdt_up_cfg[5].value = (((g_fdt_delta>>8)-2)<<8)|0x7F;
				img_cfg[0].value = g_tcode_img;
				gf_debug(DEFAULT_DEBUG,"%s use img_tcode:0x%x fdt_delta:0x%x fdt_up_delta:0x%x from OTP",
				      __func__,g_tcode_img,g_fdt_delta,((((g_fdt_delta>>8)-2)<<8)|0x7F));
			}
			/* 4. open device node and open irq*/
			nonseekable_open(inode, filp);
			gf_debug(DEFAULT_DEBUG, "Succeed to open device. irq = %d", gf_dev->spi->irq);
			gf_enable_irq(gf_dev);	
		}
	} else {
		gf_error("No device for minor %d", iminor(inode));
	}
	mutex_unlock(&device_list_lock);
	FUNC_EXIT();
	return status;
}

static int gf_release(struct inode *inode, struct file *filp)
{
	gf_dev_t *gf_dev = filp->private_data;
	int    status = 0;
	FUNC_ENTRY();
	
	mutex_lock(&device_list_lock);
	filp->private_data = NULL;

	/*last close??*/
	gf_dev->users --;
	if(!gf_dev->users) {
		gf_debug(DEFAULT_DEBUG, "disable_irq. irq = %d", gf_dev->spi->irq);
		gf_disable_irq(gf_dev);
	}
	mutex_unlock(&device_list_lock);
	FUNC_EXIT();
	return status;
}

static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return gf_ioctl(filp, cmd, (unsigned long)(arg));
}

static const struct file_operations gf_fops = {
	.owner =	THIS_MODULE,
	/* REVISIT switch to aio primitives, so that userspace
	* gets more complete API coverage.  It'll simplify things
	* too, except for the locking.
	*/
	.write = gf_write,
	.read = gf_read,
	.unlocked_ioctl = gf_ioctl,
	.compat_ioctl	= gf_compat_ioctl,
	.open = gf_open,
	.release = gf_release,
	.poll = gf_poll,
#if GF_FASYNC
	.fasync = gf_fasync,
#endif
};

static ssize_t fp_proc_read_val(struct file *file,
		char __user *buffer, size_t count, loff_t *offset)
{
	ssize_t len = 0; 
	char data[800];

	len += sprintf(data + len, "IC:goodix gf3208.\n");
	return simple_read_from_buffer(buffer, count, offset, data, len);
}

static ssize_t fp_proc_write_val(struct file *filp,
		const char *buff, size_t len, loff_t * off) 
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
		printk(KERN_INFO "create gf3208 proc file sucess!\n");
	} else 
		printk(KERN_INFO "create gf3208 proc file failed!\n");
}

static int  gf_probe(struct spi_device *spi)
{	
	gf_dev_t *gf_dev = &gf;
	int status;
	int ret;
	unsigned long minor;
	int tt = 0;

#if defined(CONFIG_FINGERPRINT_GSL6163E_A36)
	if(gsl6163e_detected == 1)
		return 0;
#endif

	printk("gf3208:driver version is %s.\n",GF_DRIVER_VERSION);

	/* Initialize the driver data */
	gf_dev->spi = spi;
	spi_set_drvdata(spi, gf_dev);
		
	/* Allocate driver data */
	gf_dev->buffer = kzalloc(bufsiz + GF_RDATA_OFFSET, GFP_KERNEL);
	if(gf_dev->buffer == NULL) {
		status = -ENOMEM;
		goto err_alloc_buffer;
	}

	spin_lock_init(&gf_dev->spi_lock);
	mutex_init(&gf_dev->buf_lock);
	mutex_init(&gf_dev->frame_lock);
	INIT_LIST_HEAD(&gf_dev->device_entry);

	/*power on*/
	gf_power_on(gf_dev, 1);//if this power is alwals on please delete this function add by car

	/*setup gf configurations.*/
	gf_debug(DEFAULT_DEBUG, "Setting gf device configuration.");
	
	/*SPI parameters.*/	
	gf_spi_setup(gf_dev, 8*1000*1000);
	gf_spi_set_mode(gf_dev->spi, SPEED_8MHZ, 0);

	gf_spi_write_word(gf_dev,0x0124,0x0100); 
	gf_spi_write_word(gf_dev,0x0204,0x0000);

	for(tt=0;tt<3;tt++) {
		unsigned short chip_id_1 = 0;
		unsigned short chip_id_2 = 0;

		gf_spi_read_word(gf_dev,0x0000,&chip_id_1);
		gf_spi_read_word(gf_dev,0x0002,&chip_id_2);
		printk("%s gf3208:chip id is 0x%04x 0x%04x \n",__func__,chip_id_2,chip_id_1);

		if(chip_id_2 == 0x0022 && chip_id_1 == 0x02A0){
			gf3208_detected = 1;
			break;
		} else {
			gf_error("Failed to detect chip id");
			status = -ENODEV;
			goto err_check_9p;
		}
	}
	
	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */

	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */
	BUILD_BUG_ON(N_SPI_MINORS > 256);
	status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops);
	if (status < 0){
		gf_error("Failed to register char device!");
		goto err_register_char;
	}
	
	gf_spi_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(gf_spi_class)) {
		gf_error("Failed to create class.");
		FUNC_EXIT();
		status = PTR_ERR(gf_spi_class);
		goto err_creat_class;
	}

	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;
		status = sysfs_create_group(&spi->dev.kobj,&gf_debug_attr_group);
		if(status){
		    gf_error("Failed to create sysfs file.");
		    goto err_creat_group;
		}
		gf_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(gf_spi_class, &spi->dev, gf_dev->devt, gf_dev, DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		gf_error( "no minor number available!");
		status = -ENODEV;
	}
	
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&gf_dev->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);
	
	/*register device within input system.*/
	gf_dev->input = input_allocate_device();
	if(gf_dev->input == NULL) {
		gf_error("Failed to allocate input device.");
		status = -ENOMEM;
		goto err_alloc_input;
	}
	
	status = gf_reg_key_kernel(gf_dev);
	if (status){
		gf_error("Failed to register input device.");
		goto err_free_input;
	}

	/*irq config and interrupt init*/
	fingerprint_irq = get_gpio(num);
	if (fingerprint_irq <= 0) {
		pr_err("gpio_to_drdy DRDY failed\n");
		status = -EBUSY;
		goto err_free_input;
	}

	gf_dev->spi->irq = fingerprint_irq;
	ret = request_irq(fingerprint_irq, (irq_handler_t) fingerprint_eint_interrupt_handler, IRQF_TRIGGER_RISING,"FINGERPRINT-eint", NULL);
	gf_disable_irq(gf_dev);

#if CONFIG_HAS_EARLYSUSPEND		
	gf_dev->early_fp.suspend = gf_early_suspend,		
	gf_dev->early_fp.resume = gf_late_resume,    	
	register_early_suspend(&gf_dev->early_fp);
#endif

	create_fp_proc_entry();
	gf_debug(DEFAULT_DEBUG,"GF installed.");

	return status;
err_free_input:
	if (gf_dev->input != NULL)
		input_free_device(gf_dev->input);
err_alloc_input:
err_creat_group:
	class_destroy(gf_spi_class);
err_creat_class:	
	unregister_chrdev(SPIDEV_MAJOR, SPI_DEV_NAME);
err_register_char:
err_check_9p:
	if (gf_dev->buffer !=NULL)
		kfree(gf_dev->buffer);
err_alloc_buffer:	
	FUNC_EXIT();
	return status;
}

static int  gf_remove(struct spi_device *spi)
{
	gf_dev_t *gf_dev = spi_get_drvdata(spi);
	FUNC_ENTRY();

	gf_dev->spi = NULL;
	spi_set_drvdata(spi, NULL);

	/* prevent new opens */
	mutex_lock(&device_list_lock);
	sysfs_remove_group(&spi->dev.kobj, &gf_debug_attr_group);
	list_del(&gf_dev->device_entry);
	device_destroy(gf_spi_class, gf_dev->devt);
	clear_bit(MINOR(gf_dev->devt), minors);
	
	if (gf_dev->users == 0) {
		if(gf_dev->input != NULL)
			input_unregister_device(gf_dev->input);
			if(gf_dev->buffer != NULL)
				kfree(gf_dev->buffer);
	}
	
	mutex_unlock(&device_list_lock);
	class_destroy(gf_spi_class);
	unregister_chrdev(SPIDEV_MAJOR, SPI_DEV_NAME);
	FUNC_EXIT();
	return 0;
}

static struct spi_driver gf_spi_driver = {
	.driver = {
		.name = "fingerprint_spi",
		.owner =	THIS_MODULE,
	},
	.probe =	gf_probe,
	.remove = gf_remove,
};

static int __init gf_init(void)
{
	int status = 0;
	gf_debug(DEFAULT_DEBUG, "gf SPI driver.");

	status = spi_register_driver(&gf_spi_driver);
	if (status < 0) {
		gf_error("Failed to register SPI driver.");
	}

	return status;
}
module_init(gf_init);

static void __exit gf_exit(void)
{
	spi_unregister_driver(&gf_spi_driver);
}
module_exit(gf_exit);

MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");
