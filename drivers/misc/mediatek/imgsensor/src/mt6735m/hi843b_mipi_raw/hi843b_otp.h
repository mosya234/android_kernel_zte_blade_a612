//add by lifengxiang@wind-mobi.com 20160725 begin
/*****************************************************************************
 *
 * Filename:
 * ---------
 *   catc24c16.h
 *
 * Project:
 * --------
 *   ALPS
 *
 * Description:
 * ------------
 *   Header file of CAM_CAL driver
 *
 *
 * Author:
 * -------
 *   John Wei (MTK07407)
 *
 *============================================================================*/
#ifndef __CAM_CAL_H
#define __CAM_CAL_H

#define CAM_CAL_DEV_MAJOR_NUMBER 226

/* CAM_CAL READ/WRITE ID */
#define HI843B_OTP_DEVICE_ID							0xc1//slave id of imx135
//#define I2C_UNIT_SIZE                                  1 //in byte
//#define OTP_START_ADDR                            0x0A04
//#define OTP_SIZE                                      24

extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern void kdSetI2CSpeed(u32 i2cSpeed); 
extern int iBurstWriteReg(u8 *pData, u32 bytes, u16 i2cId);
extern kal_uint16 read_cmos_sensor(kal_uint32 addr);
#endif /* __CAM_CAL_H */
//add by lifengxiang@wind-mobi.com 20160725 end

