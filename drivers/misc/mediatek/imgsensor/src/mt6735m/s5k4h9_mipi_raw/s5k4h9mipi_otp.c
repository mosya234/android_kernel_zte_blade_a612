/*************************************************************************************************
  4h8_otp.c
  ---------------------------------------------------------
  OTP Application file From Truly for s5k4h8
  2015.08.19
  ---------------------------------------------------------
NOTE:
The modification is appended to initialization of image sensor. 
After sensor initialization, use the function , and get the id value.
bool otp_wb_update(BYTE zone)
and
bool otp_lenc_update(BYTE zone), 
then the calibration of AWB and LSC will be applied. 
After finishing the OTP written, we will provide you the golden_rg and golden_bg settings.
 **************************************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>  
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/slab.h>

//#include <linux/xlog.h>


#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k4h9mipi_otp.h"
//#include "s5k4h5ymipiraw_Camera_Sensor_para.h"
//#include "s5k4h5ymipiraw_CameraCustomized.h"

//#include "4h5_otp.h"

#define S5K4H9_DEBUG
#ifdef S5K4H9_DEBUG
//#define LOG_TAG (__FUNCTION__)
#define PFX "s5k4h9mipi_otp"
#define LOG_INF(format, args...)	printk(PFX "%s: " format, __FUNCTION__ , ##args) 

//#define SENSORDB(fmt,arg...) pr_debug("s5k4h8_otp" "%s: " fmt, __FUNCTION__ , ##arg)  	//printk(LOG_TAG "%s: " fmt "\n", __FUNCTION__ ,##arg)
#else
#define SENSORDB(fmt,arg...)  
#endif

#define s5k4h9MIPI_WRITE_ID   0x20
//extern int s5k4h8_version;
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);//add by hhl
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);//add by hhl
#define s5k4h9_write_cmos_sensor(addr, para)  iWriteReg((u16) addr , (u32) para , 1, s5k4h9MIPI_WRITE_ID)//add by hhl


#define USHORT             unsigned short
#define BYTE               unsigned char
#define Sleep(ms) mdelay(ms)

#define TRULY_ID           0x02
#define LARGAN_LENS        0x01
#define DONGWOON           0x01
#define TDK_VCM			   0x01
#define VALID_OTP          0x01
#define WB_OFFSET          0x1C

#define GAIN_DEFAULT       512

#define Golden_RG_4h9   647 //0x206  //0x209  // modify by gionee wangxiaoyu 2015-09-01
#define Golden_BG_4h9   587 //0x24B  //0x259  // modify by gionee wangxiaoyu 2015-09-01

USHORT Current_RG_4h9;
USHORT Current_BG_4h9;


static kal_uint32 r_ratio_4h9;
static kal_uint32 b_ratio_4h9;

kal_uint16 s5k4h9_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	iReadReg((u16) addr ,(u8*)&get_byte,s5k4h9MIPI_WRITE_ID);
	return get_byte;
}
/*************************************************************************************************
 * Function    :  start_read_otp
 * Description :  before read otp , set the reading block setting  
 * Parameters  :  void
 * Return      :  void
 **************************************************************************************************/
void start_read_otp_4h9(void)
{

	s5k4h9_write_cmos_sensor(0x0100, 0x01);
	s5k4h9_write_cmos_sensor(0x0A02, 0x0F);   //Select the page to write by writing to 0xD0000A02 0x01~0x0C
	s5k4h9_write_cmos_sensor(0x0A00, 0x01);   //Enter read mode by writing 01h to 0xD0000A00
	Sleep(20);
}

/*************************************************************************************************
 * Function    :  stop_read_otp
 * Description :  after read otp , stop and reset otp block setting  
 **************************************************************************************************/
void stop_read_otp_4h9(void)
{
	s5k4h9_write_cmos_sensor(0x0A00, 0x00);   //Reset the NVM interface by writing 00h to 0xD0000A00
}


/*************************************************************************************************
 * Function    :  get_otp_flag
 * Description :  get otp WRITTEN_FLAG  
 * Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0B
 * Return      :  [BYTE], if 1 , this type has valid otp data, otherwise, invalid otp data
 **************************************************************************************************/
BYTE get_otp_flag_4h9(BYTE zone)
{
	BYTE flag = 0;
	start_read_otp_4h9();
	if(zone==1)
	{
		flag = s5k4h9_read_cmos_sensor(0x0A04);

	}
	if(zone==2)
	{
		flag = s5k4h9_read_cmos_sensor(0x0A1A);

	}
	LOG_INF("get_otp_flag: flag=%d\n", zone);
	stop_read_otp_4h9();
	if((flag&0xFF)==0x00)//empty
	{
		return 0;
	}
	else if((flag&0xFF)==0x01)//valid
	{
		return 1;
	}
	else if((flag&0xFF)==0x03)//invalid
	{
		return 2;
	}
	else 
	{
		return 0;
	}
}

/*************************************************************************************************
 * Function    :  get_otp_date
 * Description :  get otp date value    
 * Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0B    
 **************************************************************************************************/
#if 0
bool get_otp_date(BYTE zone) 
{

	BYTE year  = 0;
	BYTE month = 0;
	BYTE day   = 0;
	start_read_otp();
	year  = s5k4h8_read_cmos_sensor(0x0A06+(zone*WB_OFFSET));
	month = s5k4h8_read_cmos_sensor(0x0A07+(zone*WB_OFFSET));
	day   = s5k4h8_read_cmos_sensor(0x0A08+(zone*WB_OFFSET));
	stop_read_otp();

	LOG_INF("OTP date=%02d.%02d.%02d", year,month,day);

	return 1;

}
#endif


/*************************************************************************************************
 * Function    :  get_otp_module_id
 * Description :  get otp MID value 
 * Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0B
 * Return      :  [BYTE] 0 : OTP data fail 
 other value : module ID data , TRULY ID is 0x0002            
 **************************************************************************************************/
#if 0
BYTE get_otp_module_id(BYTE zone)
{

	BYTE module_id = 0;
	start_read_otp();

	module_id  = s5k4h8_read_cmos_sensor(0x0A05+(zone*WB_OFFSET));

	stop_read_otp();

	LOG_INF("OTP_Module ID: 0x%02x.\n",module_id);

	return module_id;

}
#endif


/*************************************************************************************************
 * Function    :  get_otp_lens_id
 * Description :  get otp LENS_ID value 
 * Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0B
 * Return      :  [BYTE] 0 : OTP data fail 
 other value : LENS ID data             
 **************************************************************************************************/
#if 0
BYTE get_otp_lens_id(BYTE zone)
{

	BYTE lens_id = 0;

	start_read_otp();

	lens_id  = s5k4h8_read_cmos_sensor(0x0A09+(zone*WB_OFFSET));

	stop_read_otp();

	LOG_INF("OTP_Lens ID: 0x%02x.\n",lens_id);

	return lens_id;

}
#endif


/*************************************************************************************************
 * Function    :  get_otp_vcm_id
 * Description :  get otp VCM_ID value 
 * Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0B
 * Return      :  [BYTE] 0 : OTP data fail 
 other value : VCM ID data             
 **************************************************************************************************/
#if 0
BYTE get_otp_vcm_id(BYTE zone)
{

	BYTE vcm_id = 0;

	start_read_otp();

	vcm_id = s5k4h8_read_cmos_sensor(0x0A0A+(zone*WB_OFFSET));

	stop_read_otp();

	LOG_INF("OTP_VCM ID: 0x%02x.\n",vcm_id);

	return vcm_id;	

}
#endif


/*************************************************************************************************
 * Function    :  get_otp_driver_id
 * Description :  get otp driver id value 
 * Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0B
 * Return      :  [BYTE] 0 : OTP data fail 
 other value : driver ID data             
 **************************************************************************************************/
#if 0
BYTE get_otp_driver_id(BYTE zone)
{

	BYTE driver_id = 0;

	start_read_otp();

	driver_id = s5k4h8_read_cmos_sensor(0x0A0B+(zone*WB_OFFSET));

	stop_read_otp();

	LOG_INF("OTP_Driver ID: 0x%02x.\n",driver_id);

	return driver_id;

}
#endif

/*************************************************************************************************
 * Function    :  get_light_id
 * Description :  get otp environment light temperature value 
 * Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0B
 * Return      :  [BYTE] 0 : OTP data fail 
 other value : driver ID data     
BIT0:D65(6500K) EN
BIT1:D50(5100K) EN
BIT2:CWF(4000K) EN
BIT3:A Light(2800K) EN
 **************************************************************************************************/
 #if 0
BYTE get_light_id(BYTE zone)
{

	BYTE light_id = 0;

	start_read_otp();
	light_id = s5k4h8_read_cmos_sensor(0x0A0D);

	stop_read_otp();

	LOG_INF("OTP_Light ID: 0x%02x.\n",light_id);

	return light_id;

}
 #endif

#if 1

/*************************************************************************************************
 * Function    :  otp_lenc_update
 * Description :  Update lens correction 
 * Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0B
 * Return      :  [bool] 0 : OTP data fail 
1 : otp_lenc update success            
 **************************************************************************************************/
bool otp_lenc_update_4h9(USHORT zone)
{
	LOG_INF("otp_lenc_update_4h9: flag=%d\n",zone);
	s5k4h9_write_cmos_sensor(0x0B00, 0x0180);
	s5k4h9_write_cmos_sensor(0x6028, 0x2000);
	s5k4h9_write_cmos_sensor(0x602A, 0x0c40);
	s5k4h9_write_cmos_sensor(0x6F12, 0x0140);
	Sleep(100);

	s5k4h9_write_cmos_sensor(0x0100, 0x0100);

	return 1;

}
#endif



/*************************************************************************************************
 * Function    :  wb_gain_set
 * Description :  Set WB ratio to register gain setting  512x
 * Parameters  :  [int] r_ratio : R ratio data compared with golden module R
b_ratio : B ratio data compared with golden module B
 * Return      :  [bool] 0 : set wb fail 
1 : WB set success            
 **************************************************************************************************/

bool wb_gain_set_4h9(void)
{
	USHORT R_GAIN;
	USHORT B_GAIN;
	USHORT Gr_GAIN;
	USHORT Gb_GAIN;
	USHORT G_GAIN;

    kal_uint16 R_GainH, B_GainH, G_GainH;
	kal_uint16 R_GainL, B_GainL, G_GainL;

	if(!r_ratio_4h9 || !b_ratio_4h9)
	{
		return 0;
	}
	
	if(r_ratio_4h9 >= 512 )
	{
		if(b_ratio_4h9>=512) 
		{
			R_GAIN = (USHORT)(GAIN_DEFAULT * r_ratio_4h9 / 512);						
			G_GAIN = GAIN_DEFAULT;
			B_GAIN = (USHORT)(GAIN_DEFAULT * b_ratio_4h9 / 512);
		}

		else
		{
			R_GAIN =  (USHORT)(GAIN_DEFAULT*r_ratio_4h9 / b_ratio_4h9 );
			G_GAIN = (USHORT)(GAIN_DEFAULT*512 / b_ratio_4h9 );
			B_GAIN = GAIN_DEFAULT;    
		}
	}
	else
	{
		if(b_ratio_4h9 >= 512)
		{
			R_GAIN = GAIN_DEFAULT;
			G_GAIN = (USHORT)(GAIN_DEFAULT*512 /r_ratio_4h9);		
			B_GAIN =  (USHORT)(GAIN_DEFAULT*b_ratio_4h9 / r_ratio_4h9 );
		} 
		else
		{

			Gr_GAIN = (USHORT)(GAIN_DEFAULT*512/ r_ratio_4h9 );						
			Gb_GAIN = (USHORT)(GAIN_DEFAULT*512/b_ratio_4h9 );						
			if(Gr_GAIN >= Gb_GAIN)						
			{						
				R_GAIN = GAIN_DEFAULT;						
				G_GAIN = (USHORT)(GAIN_DEFAULT *512/ r_ratio_4h9 );						
				B_GAIN =  (USHORT)(GAIN_DEFAULT*b_ratio_4h9 / r_ratio_4h9 );						
			} 

			else
			{						
				R_GAIN =  (USHORT)(GAIN_DEFAULT*r_ratio_4h9  / b_ratio_4h9);						
				G_GAIN = (USHORT)(GAIN_DEFAULT*512 / b_ratio_4h9 );						
				B_GAIN = GAIN_DEFAULT;	
			}
		}        
	}

	R_GainH = (R_GAIN & 0xff00)>>8;
	R_GainL = (R_GAIN & 0x00ff);

	B_GainH = (B_GAIN & 0xff00)>>8;
	B_GainL = (B_GAIN & 0x00ff);

	G_GainH = (G_GAIN & 0xff00)>>8;
	G_GainL = (G_GAIN & 0x00ff);


	LOG_INF("QYC_OTP_golden_rg=%d,golden_bg=%d\n",Golden_RG_4h9,Golden_BG_4h9);
	LOG_INF("QYC_OTP_current_rg=%d,current_bg=%d\n",Current_RG_4h9,Current_BG_4h9);
	LOG_INF("QYC_OTP_r_ratio=%d,b_ratio=%d \n",r_ratio_4h9,b_ratio_4h9);
#if 1
	s5k4h9_write_cmos_sensor(0x6028, 0x4000);
	s5k4h9_write_cmos_sensor(0x602A, 0x3058);
	s5k4h9_write_cmos_sensor(0x6F12, 0x01);
		
	s5k4h9_write_cmos_sensor(0x020e, G_GainH);
	s5k4h9_write_cmos_sensor(0x020f, G_GainL);
	s5k4h9_write_cmos_sensor(0x0210, R_GainH);
	s5k4h9_write_cmos_sensor(0x0211, R_GainL);
	s5k4h9_write_cmos_sensor(0x0212, B_GainH);
	s5k4h9_write_cmos_sensor(0x0213, B_GainL);
	s5k4h9_write_cmos_sensor(0x0214, G_GainH);
	s5k4h9_write_cmos_sensor(0x0215, G_GainL);
#endif
	LOG_INF("QYC_OTP WB Update Finished! \n");
	return 1;

}

/*************************************************************************************************
 * Function    :  get_otp_wb
 * Description :  Get WB data    
 * Parameters  :  [BYTE] zone : OTP PAGE index , 0x00~0x0f      
 **************************************************************************************************/
bool get_otp_wb_4h9(int zone)
{

	BYTE temph = 0;
	BYTE templ = 0;
	LOG_INF("get_otp_wb: zone=%d\n",zone);
	
	start_read_otp_4h9();
	if(zone == 1){
		templ = s5k4h9_read_cmos_sensor(0x0A0C);// + (zone*WB_OFFSET));
		temph= s5k4h9_read_cmos_sensor(0x0A0D);// + (zone*WB_OFFSET));

		Current_RG_4h9 = (USHORT)((temph<<8)| templ);

		templ = s5k4h9_read_cmos_sensor(0x0A0E);//+ (zone*WB_OFFSET));
		temph= s5k4h9_read_cmos_sensor(0x0A0F);//+ (zone*WB_OFFSET));

		Current_BG_4h9 = (USHORT)((temph<<8)| templ);
	}
	if(zone == 2){
		templ = s5k4h9_read_cmos_sensor(0x0A22);// + (zone*WB_OFFSET));
		temph = s5k4h9_read_cmos_sensor(0x0A23);// + (zone*WB_OFFSET));
	
		Current_RG_4h9 = (USHORT)((temph<<8)| templ);
	
		templ = s5k4h9_read_cmos_sensor(0x0A24);//+ (zone*WB_OFFSET));
		temph = s5k4h9_read_cmos_sensor(0x0A25);//+ (zone*WB_OFFSET));
	
		Current_BG_4h9 = (USHORT)((temph<<8)| templ);
	}


	LOG_INF("Current_RG=0x%x, Current_BG=0x%x\n", Current_RG_4h9, Current_BG_4h9);

	stop_read_otp_4h9();

	return 1;
}


/*************************************************************************************************
 * Function    :  otp_wb_update
 * Description :  Update WB correction 
 * Return      :  [bool] 0 : OTP data fail 
1 : otp_WB update success            
 **************************************************************************************************/
bool otp_wb_update_4h9(BYTE zone)
{
	//USHORT golden_g, current_g;


	if(!get_otp_wb_4h9(zone))  // get wb data from otp
		return 0;

	r_ratio_4h9 = 512 * Golden_RG_4h9 / Current_RG_4h9;
	b_ratio_4h9 = 512 * Golden_BG_4h9 / Current_BG_4h9;

	wb_gain_set_4h9();

	LOG_INF("otp_wb_update_4h9 finished! \n");

	return 1;
}

/*************************************************************************************************
 * Function    :  otp_update_s5k4h8()
 * Description :  update otp data from otp , it otp data is valid, 
 it include get ID and WB update function  
 * Return      :  [bool] 0 : update fail
1 : update success
 **************************************************************************************************/
bool otp_update_s5k4h9(void)
{
	BYTE zone = 0x00;
	BYTE FLG = 0x00;
	int i;
	LOG_INF("otp_update_s5k4h9 begain! \n");

	for(i=0;i<3;i++)
	{
		FLG = get_otp_flag_4h9(zone);
		if(FLG == VALID_OTP)
			break;
		else
			zone++;
	}
	if(i==3)
	{
		LOG_INF("wgs_Warning: No OTP Data or OTP data is invalid!!");
		return 0;
	}

	//MID = get_otp_module_id(zone);
	//LENS_ID =	get_otp_lens_id(zone);
	//VCM_ID =	get_otp_vcm_id(zone);
	//get_otp_date(zone);
	//get_otp_driver_id(zone);
	//	get_light_id(zone);
	//if(MID != TRULY_ID)
	//{
		//	return 0;
	//}
	otp_wb_update_4h9(zone);	
	otp_lenc_update_4h9(zone);
	
	LOG_INF("otp_update_s5k4h9 end! \n");
	
	return 1;

}
