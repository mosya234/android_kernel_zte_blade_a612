#ifdef BUILD_LK
#include <string.h>
#include <mt_gpio.h>
#include <platform/mt_pmic.h>
#include <cust_gpio_usage.h>
#include <platform/gpio_const.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <linux/string.h>
#include <mt-plat/mt_gpio.h>
#include <mach/gpio_const.h>
#include <mach/upmu_hw.h>
#endif
#include "lcm_drv.h"


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE									0
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)
#define LCM_ID_ILI9881                                                              (0x9881)     //modify yudengwu

#define REGFLAG_DELAY             								0xFC
#define REGFLAG_END_OF_TABLE      							0xFD   // END OF REGISTERS MARKER

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif
//extern unsigned char which_lcd_module_triple();
//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------
int lcm_status =1;

static LCM_UTIL_FUNCS lcm_util;

#define __SAME_IC_COMPATIBLE__

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting1[] = {
	//CCMON
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...
 
	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/
	{0xFF,3,{0x98,0x81,0x03}},
	//GIP_1                   
		{0x01,1,{0x00}},
		{0x02,1,{0x00}},
		{0x03,1,{0x53}},
		{0x04,1,{0x13}},
		{0x05,1,{0x13}},
		{0x06,1,{0x06}},
		{0x07,1,{0x00}},
		{0x08,1,{0x04}},
		{0x09,1,{0x00}},
		{0x0a,1,{0x00}},
		{0x0b,1,{0x00}},
		{0x0c,1,{0x00}},
		{0x0d,1,{0x00}},
		{0x0e,1,{0x00}},
		{0x0f,1,{0x00}},
		{0x10,1,{0x00}}, 
		{0x11,1,{0x00}},
		{0x12,1,{0x00}},
		{0x13,1,{0x00}},
		{0x14,1,{0x00}},
		{0x15,1,{0x10}},   //00  GPM
		{0x16,1,{0x00}}, 
		{0x17,1,{0x01}},   //00  GPM
		{0x18,1,{0x00}},
		{0x19,1,{0x00}},
		{0x1a,1,{0x00}},
		{0x1b,1,{0x00}},
		{0x1c,1,{0x00}},
		{0x1d,1,{0x00}},
		{0x1e,1,{0xC0}},
		{0x1f,1,{0x80}},
		{0x20,1,{0x04}},
		{0x21,1,{0x0B}},
		{0x22,1,{0x00}},
		{0x23,1,{0x00}},
		{0x24,1,{0x00}},
		{0x25,1,{0x00}},
		{0x26,1,{0x00}},
		{0x27,1,{0x00}},
		{0x28,1,{0x55}},
		{0x29,1,{0x03}},
		{0x2a,1,{0x00}},
		{0x2b,1,{0x00}},
		{0x2c,1,{0x00}},
		{0x2d,1,{0x00}},
		{0x2e,1,{0x00}},
		{0x2f,1,{0x00}},
		{0x30,1,{0x00}},
		{0x31,1,{0x00}},
		{0x32,1,{0x00}},
		{0x33,1,{0x00}},
		{0x34,1,{0x04}},
		{0x35,1,{0x05}},
		{0x36,1,{0x05}},
		{0x37,1,{0x00}},
		{0x38,1,{0x3C}},
		{0x39,1,{0x00}},
		{0x3a,1,{0x40}}, 
		{0x3b,1,{0x40}},
		{0x3c,1,{0x00}},
		{0x3d,1,{0x00}},
		{0x3e,1,{0x00}},
		{0x3f,1,{0x00}},
		{0x40,1,{0x00}},
		{0x41,1,{0x00}},
		{0x42,1,{0x00}},
		{0x43,1,{0x00}},
		{0x44,1,{0x00}},
		                
		//GIP_2      
		{0x50,1,{0x01}},
		{0x51,1,{0x23}},
		{0x52,1,{0x45}},
		{0x53,1,{0x67}},
		{0x54,1,{0x89}},
		{0x55,1,{0xab}},
		{0x56,1,{0x01}},
		{0x57,1,{0x23}},
		{0x58,1,{0x45}},
		{0x59,1,{0x67}},
		{0x5a,1,{0x89}},
		{0x5b,1,{0xab}},
		{0x5c,1,{0xcd}},
		{0x5d,1,{0xef}},
		             
		//GIP_3      
		{0x5e,1,{0x01}},
		{0x5f,1,{0x14}},
		{0x60,1,{0x15}},
		{0x61,1,{0x0C}},
		{0x62,1,{0x0D}},
		{0x63,1,{0x0E}},
		{0x64,1,{0x0F}},
		{0x65,1,{0x10}},
		{0x66,1,{0x11}},
		{0x67,1,{0x08}},
		{0x68,1,{0x02}},
		{0x69,1,{0x0A}},
		{0x6a,1,{0x02}},
		{0x6b,1,{0x02}},
		{0x6c,1,{0x02}},
		{0x6d,1,{0x02}},
		{0x6e,1,{0x02}},
		{0x6f,1,{0x02}},
		{0x70,1,{0x02}},
		{0x71,1,{0x02}},
		{0x72,1,{0x06}},
		{0x73,1,{0x02}},
		{0x74,1,{0x02}},
		{0x75,1,{0x14}},
		{0x76,1,{0x15}},
		{0x77,1,{0x11}},
		{0x78,1,{0x10}},
		{0x79,1,{0x0F}},
		{0x7a,1,{0x0E}},
		{0x7b,1,{0x0D}},
		{0x7c,1,{0x0C}},
		{0x7d,1,{0x06}},
		{0x7e,1,{0x02}},
		{0x7f,1,{0x0A}},
		{0x80,1,{0x02}},
		{0x81,1,{0x02}},
		{0x82,1,{0x02}},
		{0x83,1,{0x02}},
		{0x84,1,{0x02}},
		{0x85,1,{0x02}},
		{0x86,1,{0x02}},
		{0x87,1,{0x02}},
		{0x88,1,{0x08}},
		{0x89,1,{0x02}},
		{0x8A,1,{0x02}},
		             
		            
		//CMD_Page 4
		{0xFF,3,{0x98,0x81,0x04}},
		{0x6C,1,{0x15}},                //Set VCORE voltage =1.5V
		{0x6E,1,{0x3B}},                //di_pwr_reg=0 for power mode 2A //VGH clamp 18V
		{0x6F,1,{0x53}},                // reg vcl + pumping ratio VGH=4x VGL=-2.5x
		{0x3A,1,{0x24}},                //POWER SAVING
		{0x35,1,{0x1F}},
		             
		{0x8D,1,{0x15}},               //VGL clamp -10V
		{0x87,1,{0xBA}},             //2A
		             
		{0x26,1,{0x76}},
		{0xB2,1,{0xD1}},
		{0x88,1,{0x0B}},
	
		//CMD_Page 1 
		{0xFF,3,{0x98,0x81,0x01}},
		{0x22,1,{0x0A}},               //BGR, SS
		{0x31,1,{0x00}},               //column inversion
		{0x53,1,{0x95}},               //VCOM1
		{0x55,1,{0x99}},               //VCOM2
		{0x50,1,{0xA7}},               // VREG1OUT=4.7V
		{0x51,1,{0xa3}},               // VREG2OUT=-4.7V
		{0x60,1,{0x14}},               //SDT
		             
		{0x63,1,{0x00}},
		             
		{0xA0,1,{0x0A}},               //VP255 Gamma P
		{0xA1,1,{0x1E}},               //VP251
		{0xA2,1,{0x2E}},               //VP247
		{0xA3,1,{0x12}},               //VP243
		{0xA4,1,{0x16}},               //VP239
		{0xA5,1,{0x2A}},               //VP231
		{0xA6,1,{0x1E}},               //VP219
		{0xA7,1,{0x1F}},               //VP203
		{0xA8,1,{0x8B}},               //VP175
		{0xA9,1,{0x1D}},               //VP144
		{0xAA,1,{0x27}},               //VP111
		{0xAB,1,{0x73}},               //VP80
		{0xAC,1,{0x1A}},               //VP52
		{0xAD,1,{0x1A}},               //VP36
		{0xAE,1,{0x4D}},               //VP24
		{0xAF,1,{0x21}},               //VP16
		{0xB0,1,{0x28}},               //VP12
		{0xB1,1,{0x4C}},               //VP8
		{0xB2,1,{0x5A}},               //VP4
		{0xB3,1,{0x3F}},               //VP0
		             
		{0xC0,1,{0x00}},               //VN255 GAMMA N
		{0xC1,1,{0x1E}},               //VN251
		{0xC2,1,{0x2C}},               //VN247
		{0xC3,1,{0x12}},               //VN243
		{0xC4,1,{0x16}},               //VN239
		{0xC5,1,{0x29}},              //VN231
		{0xC6,1,{0x1E}},               //VN219
		{0xC7,1,{0x1F}},               //VN203
		{0xC8,1,{0x8A}},               //VN175
		{0xC9,1,{0x1B}},               //VN144
		{0xCA,1,{0x28}},               //VN111
		{0xCB,1,{0x74}},               //VN80
		{0xCC,1,{0x19}},               //VN52
		{0xCD,1,{0x17}},               //VN36
		{0xCE,1,{0x4C}},               //VN24
		{0xCF,1,{0x21}},               //VN16
		{0xD0,1,{0x27}},               //VN12
		{0xD1,1,{0x49}},               //VN8
		{0xD2,1,{0x59}},               //VN4
		{0xD3,1,{0x3F}},               //VN0
	
		//CMD_Page 0        
		{0xFF,3,{0x98,0x81,0x00}},
		{0x35,1,{0x00}},	
		{0x11, 0,{0x00}},
		{REGFLAG_DELAY, 120, {}},
		{0x29, 0,{0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 40, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
	//zhangaifeng@wind-mobi.com begin
    {0xFF,	3,		{0x98,0x81,0x01}},
    {0x58, 1, {0x01}},
    {REGFLAG_DELAY, 20, {}},
		//zhangaifeng@wind-mobi.com end
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;

            default:
                dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{

		memset(params, 0, sizeof(LCM_PARAMS));
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		params->dbi.te_mode				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		//LCM_DBI_TE_MODE_DISABLED;
		//LCM_DBI_TE_MODE_VSYNC_ONLY;  
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING; 
		/////////////////////   
		//if(params->dsi.lcm_int_te_monitor)  
		//params->dsi.vertical_frontporch *=2;  
		//params->dsi.lcm_ext_te_monitor= 0;//TRUE; 
	//	params->dsi.noncont_clock= TRUE;//FALSE;   
	//	params->dsi.noncont_clock_period=2;
		params->dsi.cont_clock=1;
		////////////////////          
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;  
		// DSI    /* Command mode setting */  
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;      
		//The following defined the fomat for data coming from LCD engine.  
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;   
		params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST; 
		params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;    
		params->dsi.data_format.format	  = LCM_DSI_FORMAT_RGB888;       
		// Video mode setting		   
		params->dsi.intermediat_buffer_num = 2;  
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;  
		params->dsi.packet_size=256;    
		// params->dsi.word_count=480*3;	
		//DSI CMD mode need set these two bellow params, different to 6577   
		// params->dsi.vertical_active_line=800;   
		params->dsi.vertical_sync_active				= 6; //4   
		params->dsi.vertical_backporch				       = 20;  //14  
		params->dsi.vertical_frontporch				       = 16;  //16  
		params->dsi.vertical_active_line				       = FRAME_HEIGHT;     
		params->dsi.horizontal_sync_active				= 10;   //4
		params->dsi.horizontal_backporch				= 80;  //60  
		params->dsi.horizontal_frontporch				= 80;    //60
		params->dsi.horizontal_blanking_pixel				= 60;   
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;  
		
	//	params->dsi.pll_div1=1;		   
	//	params->dsi.pll_div2=1;		   
	//	params->dsi.fbk_div =28;//28	
//zhounengwen@wind-mobi.com 20150327 beign
// To fix rf
		params->dsi.PLL_CLOCK = 212;	   // 245;

		params->dsi.ssc_disable = 1;

		params->dsi.CLK_TRAIL = 10;
			
//zhounengwen@wind-mobi.com 20140820 end
		params->dsi.esd_check_enable = 1;
                params->dsi.customization_esd_check_enable = 1;
                params->dsi.lcm_esd_check_table[0].cmd                  = 0x0a;
                params->dsi.lcm_esd_check_table[0].count                = 1;
                params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
}

static void lcm_init(void)
{



	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	
	      push_table(lcm_initialization_setting1, sizeof(lcm_initialization_setting1) / sizeof(struct LCM_setting_table), 1); 

}

#define GPIO_DISP_BL_EN (GPIO119 | 0x80000000)

static void lcm_suspend(void) 
{



	// when phone sleep , config output low, disable backlight drv chip  
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	
    MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(1);
	MDELAY(10);

	mt_set_gpio_mode(GPIO_DISP_BL_EN, GPIO_MODE_00);
   	mt_set_gpio_dir(GPIO_DISP_BL_EN, GPIO_DIR_OUT);
   	mt_set_gpio_out(GPIO_DISP_BL_EN, GPIO_OUT_ZERO);

}

static void lcm_resume(void)
{

	mt_set_gpio_mode(GPIO_DISP_BL_EN, GPIO_MODE_00);
   	mt_set_gpio_dir(GPIO_DISP_BL_EN, GPIO_DIR_OUT);
   	mt_set_gpio_out(GPIO_DISP_BL_EN, GPIO_OUT_ONE);
	MDELAY(20);


	lcm_init();

}

static unsigned int lcm_compare_id(void)
{
	return 1;
	
}




LCM_DRIVER ili9881c_hd720_dsi_vdo_cmi_lcm_drv =
{
    .name           	= "ili9881c_dsi_vdo_yashi_cmi",
    .set_util_funcs 	= lcm_set_util_funcs,
    .get_params     	= lcm_get_params,
    .init           	= lcm_init,
    .suspend        	= lcm_suspend,
    .resume         	= lcm_resume,
    .compare_id     	= lcm_compare_id,


};

