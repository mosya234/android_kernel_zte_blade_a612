//add by lifengxiang@wind-mobi.com 20160725 begin
/*
 * Driver for CAM_CAL
 */
#include <linux/dma-mapping.h>
#include <linux/module.h> 
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "hi843b_otp.h"
//#include <asm/system.h>  // for SMP
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif

#define PFX "hi843b_OTP_FMT"
//#define CAM_CALGETDLT_DEBUG
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#define CAM_CALINF(fmt, arg...)    pr_err("[%s] " fmt, __FUNCTION__, ##arg)
#define CAM_CALDB(fmt, arg...)     pr_err("[%s] " fmt, __FUNCTION__, ##arg)
#define CAM_CALERR(fmt, arg...)    pr_err("[%s] " fmt, __FUNCTION__, ##arg)
#else
#define CAM_CALINF(x,...)
#define CAM_CALDB(x,...)
#define CAM_CALERR(fmt, arg...)    pr_err("[%s] " fmt, __FUNCTION__, ##arg)
#endif

#define CAM_CAL_ICS_REVISION 1 //seanlin111208
#define CAM_CAL_DRVNAME "HI843BCAM_CAL_DRV"
#define CAM_CAL_I2C_GROUP_ID 2
#define MAX_LSC_SIZE 1024
#define MAX_OTP_SIZE 1100

static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP
static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;

static int hi843b_otp_read = 0;

typedef struct {
    u8     MoudleID;
	u8     Version;//0x01
	u8     AwbAfInfo;//0xF
	u8     UnitAwbR_Gr_H;
	u8     UnitAwbR_Gr_L;
	u8     UnitAwbB_Gb_H;
	u8     UnitAwbB_Gb_L;
	u8     UnitAwbGb_Gr_H;
	u8     UnitAwbGb_Gr_L;
	u8     GoldenAwbR_Gr_H;
	u8     GoldenAwbR_Gr_L;
	u8     GoldenAwbB_Gb_H;
	u8     GoldenAwbB_Gb_L;
	u8     GoldenAwbGb_Gr_H;
	u8     GoldenAwbGb_Gr_L;
	u16    AfInfinite;
	u16    AfMacro;
	u16    LscSize;//0x03A4
	u8     Lsc[MAX_LSC_SIZE];
}OTP_MTK_TYPE;

OTP_MTK_TYPE    Hi843b_otp_data = {0};

static void write_cmos_sensor_byte(kal_uint32 addr, kal_uint32 para)
{
    char pusendcmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,(char)(para & 0xFF)};
	kdSetI2CSpeed(400); // Add this func to set i2c speed by each sensor
    iWriteRegI2C(pusendcmd, 3, 0xc0);
}

void Hi843bOTPSetting(void)
{	
	write_cmos_sensor_byte(0x0a02, 0x01); //fast sleep On
	write_cmos_sensor_byte(0x0a00,0x00);  //stand by on
	mdelay(5);//liuying 20150825
	write_cmos_sensor_byte(0x0F02, 0x00); //pll disable
	write_cmos_sensor_byte(0x071A, 0x01); //CP TRI_H
	write_cmos_sensor_byte(0x071B, 0x09); //IPGM TRIM_H
	write_cmos_sensor_byte(0x0D04, 0x01); //Fsync Output enable
	write_cmos_sensor_byte(0x0D00, 0x07); //Fsync Output Drivability
	//write_cmos_sensor_byte(0x004C, 0x01); //TG MCU enable
	write_cmos_sensor_byte(0x003e, 0x10); //OTP R/W
	write_cmos_sensor_byte(0x0a00, 0x01); //sleep off
	//mdelay(10);
	//Hi545_bytewrite_cmos_sensor(0x004C, 0x00); //TG MCU disable
	//Hi545_bytewrite_cmos_sensor(0x004C, 0x01); //TG MCU enable
	mdelay(2);//liuying 20150825
	CAM_CALERR("Hi843b OTPSetting exit :\n ");
}

void HI843bOTPExit(void)
{
	write_cmos_sensor_byte(0x0a00,0x00);  //stand by on
	mdelay(10);
	write_cmos_sensor_byte(0x003e,0x00); //display mode
	write_cmos_sensor_byte(0x0a00,0x01);  //stand by off
	CAM_CALERR("Hi843b OTPclose exit :\n ");
}

static u16 otp_signal_read(u16 otp_addr)
{
    u16 data;
    write_cmos_sensor_byte(0x070a, (otp_addr & 0xff00)>> 8); //start address H        
    write_cmos_sensor_byte(0x070b, otp_addr& 0x00ff); //start address L
    write_cmos_sensor_byte(0x0702, 0x01); //single read
    mdelay(2);
    data = read_cmos_sensor(0x0708); //OTP data read  
    return data;
}

int read_hi843b_otp_mtk_fmt(void)
{
	u16 i=0, flag=0, addr=0;

	CAM_CALERR("---E: readed =%d \n",hi843b_otp_read);

	if(1 == hi843b_otp_read ) {
		CAM_CALERR("---HYNIX_CAM_CAL OTP has readed ! skip\n");
        return 0;
    }
	
	flag = otp_signal_read(0x0c5f);
	CAM_CALERR("zyk:HI843B : flag = 0x%x\n",flag);
	switch (flag)
	{
		case 0x01 : addr=0x0c60; break;
		case 0x13 : addr=0x0c7e; break;
		case 0x37 : addr=0x0c9c; break;
		default : return -1;
	}
	
	Hi843b_otp_data.MoudleID = 0x43;
	Hi843b_otp_data.Version = 0x01;//0x01
	Hi843b_otp_data.AwbAfInfo = 0x03;
	
	write_cmos_sensor_byte(0x070a, (addr & 0xff00)>> 8); //start address H        
	write_cmos_sensor_byte(0x070b, addr & 0x00ff); //start address L
	write_cmos_sensor_byte(0x0702, 0x01); //continue read
	
	Hi843b_otp_data.UnitAwbR_Gr_H = read_cmos_sensor(0x0708);
    CAM_CALDB("hi843b_otp_data = 0x%x\n", Hi843b_otp_data.UnitAwbR_Gr_H);
    Hi843b_otp_data.UnitAwbR_Gr_L = read_cmos_sensor(0x0708);
	CAM_CALDB("hi843b_otp_data = 0x%x\n", Hi843b_otp_data.UnitAwbR_Gr_L);
    Hi843b_otp_data.UnitAwbB_Gb_H = read_cmos_sensor(0x0708);
	CAM_CALDB("hi843b_otp_data = 0x%x\n", Hi843b_otp_data.UnitAwbB_Gb_H);
    Hi843b_otp_data.UnitAwbB_Gb_L = read_cmos_sensor(0x0708);
	CAM_CALDB("hi843b_otp_data = 0x%x\n", Hi843b_otp_data.UnitAwbB_Gb_L);
    Hi843b_otp_data.UnitAwbGb_Gr_H = read_cmos_sensor(0x0708);
	CAM_CALDB("hi843b_otp_data = 0x%x\n", Hi843b_otp_data.UnitAwbGb_Gr_H);
    Hi843b_otp_data.UnitAwbGb_Gr_L = read_cmos_sensor(0x0708);
	CAM_CALDB("hi843b_otp_data = 0x%x\n", Hi843b_otp_data.UnitAwbGb_Gr_L);
    Hi843b_otp_data.GoldenAwbR_Gr_H = read_cmos_sensor(0x0708);
	CAM_CALDB("hi843b_otp_data = 0x%x\n", Hi843b_otp_data.GoldenAwbR_Gr_H);
    Hi843b_otp_data.GoldenAwbR_Gr_L = read_cmos_sensor(0x0708);
	CAM_CALDB("hi843b_otp_data = 0x%x\n", Hi843b_otp_data.GoldenAwbR_Gr_L);
    Hi843b_otp_data.GoldenAwbB_Gb_H = read_cmos_sensor(0x0708);
	CAM_CALDB("hi843b_otp_data = 0x%x\n", Hi843b_otp_data.GoldenAwbB_Gb_H);
    Hi843b_otp_data.GoldenAwbB_Gb_L = read_cmos_sensor(0x0708);
	CAM_CALDB("hi843b_otp_data = 0x%x\n", Hi843b_otp_data.GoldenAwbB_Gb_L);
    Hi843b_otp_data.GoldenAwbGb_Gr_H = read_cmos_sensor(0x0708);
	CAM_CALDB("hi843b_otp_data = 0x%x\n", Hi843b_otp_data.GoldenAwbGb_Gr_H);
    Hi843b_otp_data.GoldenAwbGb_Gr_L = read_cmos_sensor(0x0708);
	CAM_CALDB("hi843b_otp_data = 0x%x\n", Hi843b_otp_data.GoldenAwbGb_Gr_L);
	
	flag = otp_signal_read(0x0235);
	CAM_CALERR("zyk:HI843B : flag = 0x%x\n",flag);
	switch (flag)
	{
		case 0x01 : addr=0x0236; break;
		case 0x13 : addr=0x0599; break;
		case 0x37 : addr=0x08fc; break;
		default : return -1;
	}
	
	Hi843b_otp_data.LscSize = 858;
	
	write_cmos_sensor_byte(0x070a, (addr & 0xff00)>> 8); //start address H        
	write_cmos_sensor_byte(0x070b, addr & 0x00ff); //start address L
	write_cmos_sensor_byte(0x0702, 0x01); //continue read
	
	for (i = 0; i < Hi843b_otp_data.LscSize; i++)
	{
		Hi843b_otp_data.Lsc[i] = read_cmos_sensor(0x0708);
		CAM_CALDB("hi843b_otp_data = 0x%x\n", Hi843b_otp_data.Lsc[i]);
	}
	
	addr = 0x0cbd;
	write_cmos_sensor_byte(0x070a, (addr & 0xff00)>> 8); //start address H        
	write_cmos_sensor_byte(0x070b, addr & 0x00ff); //start address L
	write_cmos_sensor_byte(0x0702, 0x01); //continue read
	Hi843b_otp_data.AfInfinite = ((u16)read_cmos_sensor(0x0708) << 8)|((u16)read_cmos_sensor(0x0708) & 0x00ff);
	CAM_CALDB("Hi843b_otp_data.AfInfinite = 0x%x\n", Hi843b_otp_data.AfInfinite);
	Hi843b_otp_data.AfMacro = ((u16)read_cmos_sensor(0x0708) << 8)|((u16)read_cmos_sensor(0x0708) & 0x00ff);
	CAM_CALDB("Hi843b_otp_data.AfMacro = 0x%x\n", Hi843b_otp_data.AfMacro);
	return 0;
}

#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data->u4Offset);
    err |= put_user(i, &data32->u4Offset);
    err |= get_user(i, &data->u4Length);
    err |= put_user(i, &data32->u4Length);
    /* Assume pointer is not change */
#if 1
    err |= get_user(p, &data32->pu1Params);
    err |= put_user(compat_ptr(p), &data->pu1Params);
#endif
    return err;
}
static int compat_get_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data32->u4Offset);
    err |= put_user(i, &data->u4Offset);
    err |= get_user(i, &data32->u4Length);
    err |= put_user(i, &data->u4Length);
    err |= get_user(p, &data32->pu1Params);
    err |= put_user(compat_ptr(p), &data->pu1Params);

    return err;
}

static long hi843botp_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret;
    COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
    stCAM_CAL_INFO_STRUCT __user *data;
    int err;
	CAM_CALDB("[CAMERA SENSOR] hi843b_OTP_DEVICE_ID,%p %p %x ioc size %d\n",filp->f_op ,filp->f_op->unlocked_ioctl,cmd,_IOC_SIZE(cmd) );

    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
        return -ENOTTY;

    switch (cmd) {

    case COMPAT_CAM_CALIOC_G_READ:
    {
        data32 = compat_ptr(arg);
        data = compat_alloc_user_space(sizeof(*data));
        if (data == NULL)
            return -EFAULT;

        err = compat_get_cal_info_struct(data32, data);
        if (err)
            return err;

        ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ,(unsigned long)data);
        err = compat_put_cal_info_struct(data32, data);


        if(err != 0)
            CAM_CALERR("[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
        return ret;
    }
    default:
        return -ENOIOCTLCMD;
    }
}
#endif

static int selective_read_region(u32 offset, unsigned char* data,u16 i2c_id,u32 size)
{
    memcpy((void *)data, (void *)&Hi843b_otp_data + offset, size);
	CAM_CALDB("selective_read_region offset =%x size %d data read = %d\n", offset,size, *data);
    return size;
}

/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else
static long CAM_CAL_Ioctl(
    struct file *file,
    unsigned int a_u4Command,
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pu1Params = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;
#ifdef CAM_CALGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif

    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            CAM_CALERR(" ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                CAM_CALERR("ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pu1Params = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL);
    if(NULL == pu1Params)
    {
        kfree(pBuff);
        CAM_CALERR("ioctl allocate mem failed\n");
        return -ENOMEM;
    }


    if(copy_from_user((u8*)pu1Params ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pu1Params);
        CAM_CALERR(" ioctl copy from user failed\n");
        return -EFAULT;
    }

    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:
            CAM_CALDB("Write CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            i4RetValue = 0;//iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            CAM_CALDB("Write data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif
            break;
        case CAM_CALIOC_G_READ:
            CAM_CALDB("[CAM_CAL] Read CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif
            i4RetValue = selective_read_region(ptempbuf->u4Offset, pu1Params, HI843B_OTP_DEVICE_ID,ptempbuf->u4Length);

#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            CAM_CALDB("Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif

            break;
        default :
      	     CAM_CALINF("[CAM_CAL] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pu1Params , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pu1Params);
            CAM_CALERR("[CAM_CAL] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pu1Params);
    return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    CAM_CALDB("CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
		CAM_CALERR("Opened, return -EBUSY\n");
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);
    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_CALLock);

    g_u4Opened = 0;

    atomic_set(&g_CAM_CALatomic,0);

    spin_unlock(&g_CAM_CALLock);

    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
    //.ioctl = CAM_CAL_Ioctl
#ifdef CONFIG_COMPAT
    .compat_ioctl = hi843botp_Ioctl_Compat,
#endif
    .unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
//#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1

inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;
    CAM_CALDB("RegisterCAM_CALCharDrv\n");
#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
        CAM_CALERR(" Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        CAM_CALERR(" Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        CAM_CALERR(" Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALERR(" Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv_HI843b");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALERR("Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

    return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
    //Release char driver
    cdev_del(g_pCAM_CAL_CharDrv);

    unregister_chrdev_region(g_CAM_CALdevno, 1);

    device_destroy(CAM_CAL_class, g_CAM_CALdevno);
    class_destroy(CAM_CAL_class);
}

#if 0
//////////////////////////////////////////////////////////////////////
#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#elif 0
static int CAM_CAL_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#else
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int CAM_CAL_i2c_remove(struct i2c_client *);

static const struct i2c_device_id CAM_CAL_i2c_id[] = {{CAM_CAL_DRVNAME,0},{}};



static struct i2c_driver CAM_CAL_i2c_driver = {
    .probe = CAM_CAL_i2c_probe,
    .remove = CAM_CAL_i2c_remove,
//   .detect = CAM_CAL_i2c_detect,
    .driver.name = CAM_CAL_DRVNAME,
    .id_table = CAM_CAL_i2c_id,
};

#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {
    strcpy(info->type, CAM_CAL_DRVNAME);
    return 0;
}
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {
//int i4RetValue = 0;
    CAM_CALDB("Attach I2C \n");
//    spin_lock_init(&g_CAM_CALLock);

    //get sensor i2c client
    spin_lock(&g_CAM_CALLock); //for SMP
    g_pstI2Cclient = client;
    g_pstI2Cclient->addr = hi843b_OTP_DEVICE_ID>>1;
    spin_unlock(&g_CAM_CALLock); // for SMP

    CAM_CALDB("g_pstI2Cclient->addr = 0x%x \n",g_pstI2Cclient->addr);
    return 0;
}

static int CAM_CAL_i2c_remove(struct i2c_client *client)
{
    return 0;
}
#endif
static int CAM_CAL_probe(struct platform_device *pdev)
{

    return 0;//i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    //i2c_del_driver(&CAM_CAL_i2c_driver);
    return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
    .probe		= CAM_CAL_probe,
    .remove	= CAM_CAL_remove,
    .driver		= {
        .name	= CAM_CAL_DRVNAME,
        .owner	= THIS_MODULE,
    }
};


static struct platform_device g_stCAM_CAL_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init CAM_CAL_init(void)
{
    int i4RetValue = 0;
    CAM_CALDB("CAM_CAL_i2C_init\n");
   //Register char driver
	i4RetValue = RegisterCAM_CALCharDrv();
    if(i4RetValue){
 	   CAM_CALDB(" register char device failed!\n");
	   return i4RetValue;
	}
	CAM_CALDB(" Attached!! \n");

  //  i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);
    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        CAM_CALERR("failed to register hi843botp driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        CAM_CALERR("failed to register hi843botp driver, 2nd time\n");
        return -ENODEV;
    }

    return 0;
}

static void __exit CAM_CAL_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_init);
module_exit(CAM_CAL_exit);

MODULE_DESCRIPTION("CAM_CAL driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");
//add by lifengxiang@wind-mobi.com 20160725 end

