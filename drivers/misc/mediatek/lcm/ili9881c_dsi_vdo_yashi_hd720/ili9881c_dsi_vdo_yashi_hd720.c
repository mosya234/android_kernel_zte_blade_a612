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

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             							0XFC
#define REGFLAG_END_OF_TABLE      							0xFD   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))

#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

#define GPIO_LCD_DRV_EN_PIN  (GPIO64 | 0x80000000)
#define GPIO_PCD_ID0  (GPIO124 | 0x80000000)
#define GPIO_PCD_ID1  (GPIO125 | 0x80000000)
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)      

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_sleep_out_setting[] = 
{
{0xFF,3,{0x98,0x81,0x03}},
{0x01,1,{0x00}},
{0x02,1,{0x00}},
{0x03,1,{0x73}},
{0x04,1,{0x00}},
{0x05,1,{0x00}},
{0x06,1,{0x0C}},
{0x07,1,{0x00}},
{0x08,1,{0x00}},
{0x09,1,{0x01}},
{0x0A,1,{0x01}},
{0x0B,1,{0x01}},
{0x0C,1,{0x01}},
{0x0D,1,{0x01}},
{0x0E,1,{0x01}},
{0x0F,1,{0x00}},
{0x10,1,{0x00}},
{0x11,1,{0x00}},
{0x12,1,{0x00}},
{0x13,1,{0x00}},
{0x14,1,{0x00}},
{0x15,1,{0x10}},
{0x16,1,{0x10}},
{0x17,1,{0x00}},
{0x18,1,{0x00}},
{0x19,1,{0x00}},
{0x1A,1,{0x00}},
{0x1B,1,{0x00}},
{0x1C,1,{0x00}},
{0x1D,1,{0x00}},
{0x1E,1,{0x44}},
{0x1F,1,{0xC0}},
{0x20,1,{0x0A}},
{0x21,1,{0x03}},
{0x22,1,{0x0A}},
{0x23,1,{0x00}},
{0x24,1,{0x8C}},
{0x25,1,{0x8C}},
{0x26,1,{0x00}},
{0x27,1,{0x00}},
{0x28,1,{0x3B}},
{0x29,1,{0x03}},
{0x2A,1,{0x00}},
{0x2B,1,{0x00}},
{0x2C,1,{0x00}},
{0x2D,1,{0x00}},
{0x2E,1,{0x00}},
{0x2F,1,{0x00}},
{0x30,1,{0x00}},
{0x31,1,{0x00}},
{0x32,1,{0x00}},
{0x33,1,{0x00}},
{0x34,1,{0x00}},
{0x35,1,{0x00}},
{0x36,1,{0x00}},
{0x37,1,{0x00}},
{0x38,1,{0x00}},
{0x39,1,{0x00}},
{0x3A,1,{0x00}},
{0x3B,1,{0x00}},
{0x3C,1,{0x00}},
{0x3D,1,{0x00}},
{0x3E,1,{0x00}},
{0x3F,1,{0x00}},
{0x40,1,{0x00}},
{0x41,1,{0x00}},
{0x42,1,{0x00}},
{0x43,1,{0x00}},
{0x44,1,{0x00}},
{0x50,1,{0x01}},
{0x51,1,{0x23}},
{0x52,1,{0x45}},
{0x53,1,{0x67}},
{0x54,1,{0x89}},
{0x55,1,{0xAB}},
{0x56,1,{0x01}},
{0x57,1,{0x23}},
{0x58,1,{0x45}},
{0x59,1,{0x67}},
{0x5A,1,{0x89}},
{0x5B,1,{0xAB}},
{0x5C,1,{0xCD}},
{0x5D,1,{0xEF}},
{0x5E,1,{0x11}},
{0x5F,1,{0x0C}},
{0x60,1,{0x0D}},
{0x61,1,{0x0E}},
{0x62,1,{0x0F}},
{0x63,1,{0x06}},
{0x64,1,{0x07}},
{0x65,1,{0x02}},
{0x66,1,{0x02}},
{0x67,1,{0x02}},
{0x68,1,{0x02}},
{0x69,1,{0x02}},
{0x6A,1,{0x02}},
{0x6B,1,{0x02}},
{0x6C,1,{0x02}},
{0x6D,1,{0x02}},
{0x6E,1,{0x02}},
{0x6F,1,{0x02}},
{0x70,1,{0x02}},
{0x71,1,{0x02}},
{0x72,1,{0x02}},
{0x73,1,{0x01}},
{0x74,1,{0x00}},
{0x75,1,{0x0C}},
{0x76,1,{0x0D}},
{0x77,1,{0x0E}},
{0x78,1,{0x0F}},
{0x79,1,{0x06}},
{0x7A,1,{0x07}},
{0x7B,1,{0x02}},
{0x7C,1,{0x02}},
{0x7D,1,{0x02}},
{0x7E,1,{0x02}},
{0x7F,1,{0x02}},
{0x80,1,{0x02}},
{0x81,1,{0x02}},
{0x82,1,{0x02}},
{0x83,1,{0x02}},
{0x84,1,{0x02}},
{0x85,1,{0x02}},
{0x86,1,{0x02}},
{0x87,1,{0x02}},
{0x88,1,{0x02}},
{0x89,1,{0x01}},
{0x8A,1,{0x00}},
{0xFF,3,{0x98,0x81,0x04}},
{0x6C,1,{0x15}},
{0x6E,1,{0x1a}},			
{0x6F,1,{0x25}},			
{0x3B,1,{0xC0}},
{0x3a,1,{0x24}},
{0x35,1,{0x1f}},
{0x8D,1,{0x20}},			
{0x87,1,{0xBA}},			
{0x26,1,{0x76}},
{0xB2,1,{0xD1}},
{0x33,1,{0x00}},
{0xFF,3,{0x98,0x81,0x01}},
{0x22,1,{0x0A}},			
{0x31,1,{0x00}},
{0x50,1,{0x95}},			
{0x51,1,{0x95}},			
{0x53,1,{0x59}},			
{0x55,1,{0x59}},		
{0x60,1,{0x09}},
{0xA0,1,{0x08}},			
{0xA1,1,{0x11}},			
{0xA2,1,{0x1A}},			
{0xA3,1,{0x0b}},			
{0xA4,1,{0x0d}},			
{0xA5,1,{0x1f}},			
{0xA6,1,{0x15}},			
{0xA7,1,{0x19}},			
{0xA8,1,{0x5f}},			
{0xA9,1,{0x1C}},			
{0xAA,1,{0x28}},			
{0xAB,1,{0x53}},			
{0xAC,1,{0x18}},			
{0xAD,1,{0x13}},			
{0xAE,1,{0x4D}},			
{0xAF,1,{0x22}},			
{0xB0,1,{0x29}},			
{0xB1,1,{0x4d}},			
{0xB2,1,{0x69}},			
{0xB3,1,{0x39}},			
{0xC0,1,{0x08}},			
{0xC1,1,{0x10}},			
{0xC2,1,{0x19}},			
{0xC3,1,{0x0b}},			
{0xC4,1,{0x0c}},			
{0xC5,1,{0x1f}},			
{0xC6,1,{0x15}},			
{0xC7,1,{0x19}},			
{0xC8,1,{0x5e}},			
{0xC9,1,{0x1D}},			
{0xCA,1,{0x28}},			
{0xCB,1,{0x53}},			
{0xCC,1,{0x19}},			
{0xCD,1,{0x13}},			
{0xCE,1,{0x4C}},			
{0xCF,1,{0x22}},			
{0xD0,1,{0x28}},			
{0xD1,1,{0x4f}},			
{0xD2,1,{0x68}},			
{0xD3,1,{0x39}},
{0xFF,3,{0x98,0x81,0x02}},//pwm=31.2K
{0x06,1,{0x20}},
{0x07,1,{0x00}},
{0xFF,3,{0x98,0x81,0x00}},
{0x36,1,{0x00}},
{0x35,1,{0x00}},
{0x51,2,{0x00,0x00}},//PWM ON 
{0x53,1,{0x2C}},
{0x55,1,{0x01}},
{0x11,1,{0x00}},
{REGFLAG_DELAY, 120, {}},		
{0x29,1,{0x00}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_in_setting[] = {
{0xFF, 3, {0x98, 0x81, 0x00}},
 // Display off sequence
 {0x28, 0, {0x00}},
 {REGFLAG_DELAY, 40, {}},
 // Sleep Mode On
 {0x10, 0, {0x00}},
 {REGFLAG_DELAY, 150, {}},
 {REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_backlight_level_setting[] = {
{0xFF, 3, {0x98, 0x81, 0x00}},	
{0x51, 2, {0x0F,0xFF}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void lcm_init_power(void)
{
	#ifndef CONFIG_FPGA_EARLY_PORTING
		#ifdef BUILD_LK
			//mt6328_upmu_set_rg_vgp1_en(1);  //liuyi
		#else
			printk("%s, begin\n", __func__);
			//hwPowerOn(MT6325_POWER_LDO_VGP1, VOL_DEFAULT, "LCM_DRV");	//liuyi
			printk("%s, end\n", __func__);
		#endif
	#endif
}

static void lcm_suspend_power(void)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef BUILD_LK
	//mt6325_upmu_set_rg_vgp1_en(0);//liuyi
#else
	printk("%s, begin\n", __func__);
	//hwPowerDown(MT6325_POWER_LDO_VGP1, "LCM_DRV");	//liuyi
	printk("%s, end\n", __func__);
#endif
#endif
}

static void lcm_resume_power(void)
{
#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef BUILD_LK
	//mt6325_upmu_set_rg_vgp1_en(1);  //liuyi
#else
	printk("%s, begin\n", __func__);
	//hwPowerOn(MT6325_POWER_LDO_VGP1, VOL_DEFAULT, "LCM_DRV");	 //liuyi
	printk("%s, end\n", __func__);
#endif
#endif
}

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
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
		params->physical_width = 62;
		params->physical_height =110;

#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = SYNC_EVENT_VDO_MODE; //BURST_VDO_MODE;
#endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;
            //The following defined the fomat for data coming from LCD engine.
            params->dsi.data_format.color_order     = LCM_COLOR_ORDER_RGB;
            params->dsi.data_format.trans_seq       = LCM_DSI_TRANS_SEQ_MSB_FIRST;
            params->dsi.data_format.padding         = LCM_DSI_PADDING_ON_LSB;
            params->dsi.data_format.format              = LCM_DSI_FORMAT_RGB888;

            // Highly depends on LCD driver capability.
                 params->dsi.packet_size=256;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
             
             params->dsi.vertical_sync_active    = 2;
             params->dsi.vertical_backporch      = 16;
             params->dsi.vertical_frontporch     = 9;
             params->dsi.vertical_active_line    = FRAME_HEIGHT; 
             
             params->dsi.horizontal_sync_active    = 40;
             params->dsi.horizontal_backporch      = 40;
             params->dsi.horizontal_frontporch     = 40;
             params->dsi.horizontal_active_pixel   = FRAME_WIDTH; 
             params->dsi.HS_TRAIL = 12;
             
             params->dsi.PLL_CLOCK = 212;
///////////////////////////////////////////////////////ESD			
		params->dsi.esd_check_enable = 0;
		params->dsi.customization_esd_check_enable = 1;
		params->dsi.lcm_esd_check_table[0].cmd 			= 0x0a;
		params->dsi.lcm_esd_check_table[0].count 		= 1;
		params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
		
		params->dsi.lcm_esd_check_table[1].cmd 			= 0x54;
		params->dsi.lcm_esd_check_table[1].count 		= 1;
		params->dsi.lcm_esd_check_table[1].para_list[0] = 0x2c;
		
		params->dsi.noncont_clock = 1;
		params->dsi.noncont_clock_period = 1;
///////////////////////////////////////////////////////			
		params->dsi.vertical_vfp_lp = 100;
}

static unsigned int lcm_compare_id(void)
{
	int pin_lcd_id0=-1,pin_lcd_id1=-1;	

	mt_set_gpio_mode(GPIO_PCD_ID0, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_PCD_ID0, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_PCD_ID0, GPIO_PULL_ENABLE);
	mt_set_gpio_mode(GPIO_PCD_ID1, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_PCD_ID1, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_PCD_ID1, GPIO_PULL_ENABLE);	
	pin_lcd_id0= mt_get_gpio_in(GPIO_PCD_ID0);
	pin_lcd_id1= mt_get_gpio_in(GPIO_PCD_ID1);
#ifdef BUILD_LK	
	dprintf(0,"%s, F10 yashi ili9881c boe , pin_lcd_id0= %d  pin_lcd_id1 is 0x%x \n", __func__, pin_lcd_id0,pin_lcd_id1);
#else
	printk("%s, F10 yashi ili9881c boe , pin_lcd_id0= %d  pin_lcd_id1 is 0x%x \n", __func__, pin_lcd_id0,pin_lcd_id1);
#endif
	if ( 0== pin_lcd_id0 && 1== pin_lcd_id1)
	{
		return 1;
	}
	else 
		return 0;
    
}

extern unsigned short pmic_set_register_value(PMU_FLAGS_LIST_ENUM flagname, unsigned int val);
static void lcm_init(void)
{
#ifdef BUILD_LK
	dprintf(0,"%s,lk lcm_init:yashi ili9881c\n", __func__);
#else
	printk("lcm_init:yashi ili9881c \n");
#endif
	pmic_set_register_value(PMIC_RG_VGP1_EN,1);

	mt_set_gpio_pull_enable(GPIO_LCD_DRV_EN_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_LCD_DRV_EN_PIN, GPIO_PULL_UP);
	mt_set_gpio_mode(GPIO_LCD_DRV_EN_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_DRV_EN_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_DRV_EN_PIN, 1);	
	MDELAY(10);
	
	SET_RESET_PIN(1);
	MDELAY(10);//5
	SET_RESET_PIN(0);
	MDELAY(10);//50
	SET_RESET_PIN(1);
	MDELAY(120);//100

	//init_lcm_registers();
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(5);//100
}
static void lcm_suspend(void)
{	
    push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
    MDELAY(5);
    SET_RESET_PIN(0);
    #ifdef BUILD_LK
	dprintf(0,"%s,lk :yashi ili9881c 10ms\n", __func__);
    #else
    	printk("lcm_suspend:yashi ili9881c 10ms \n");
    #endif	
    MDELAY(10);
    // gpio_set_value(GPIO_DISP_BL_EN, 0);
	
}



static void lcm_resume(void)
{

	lcm_init();
//	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);	
}
#endif
static void lcm_setbacklight(unsigned int level)
{
        unsigned char level_high,level_low;

	// Refresh value of backlight level.

	level = 4095*level /255;	
	level_high = (level >> 8) & 0x0F;
	level_low = level & 0x0FF;        
	lcm_backlight_level_setting[1].para_list[0] = level_high;
	lcm_backlight_level_setting[1].para_list[1] = level_low;
	
#ifdef BUILD_LK
	dprintf(0,"%s,zhangjian lk ili9881c_dsi_vdo_yashi_hd720: level = %d\n", __func__, level);
#else
	printk("%s,  ili9881c_dsi_vdo_yashi_hd720 backlight: new level = 0x%x evel_high, level_low 0x%x,0x%x\n", __func__, level,level_high, level_low);
#endif       
        push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);

}

LCM_DRIVER ili9881c_dsi_vdo_yashi_hd720_lcm_drv = 
{
	.name			= "ili9881c_dsi_vdo_yashi_hd720",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
	.init_power		= lcm_init_power,
    .resume_power = lcm_resume_power,
    .suspend_power = lcm_suspend_power,
	.set_backlight	= lcm_setbacklight,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};

