#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#else
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

//#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/clk.h>
//#endif
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK

#define LCM_ID_PIN GPIO80

static int lcm_get_gpio_input(unsigned int GPIO)
{
    mt_set_gpio_mode(GPIO, GPIO_MODE_00);
	mt_set_gpio_pull_enable(GPIO, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO, GPIO_DIR_IN);
	//MDELAY(10);
	printf("ili9881c lk gpio=%d\n", mt_get_gpio_in(GPIO));
	return mt_get_gpio_in(GPIO);
}
#else
static unsigned int GPIO_LCD_ID;

static void lcm_get_gpio_infor(void)
{
	static struct device_node *node;
	node = of_find_compatible_node(NULL, NULL, "mediatek,lcm");
	GPIO_LCD_ID = of_get_named_gpio(node, "lcm_id_gpio", 0);
}

static int lcm_get_gpio_input(unsigned int GPIO)
{
	gpio_direction_input(GPIO);
	return gpio_get_value(GPIO);
}

static int lcm_probe(struct device *dev)
{
	int lcm_id=1;
	lcm_get_gpio_infor();
    	lcm_id=lcm_get_gpio_input(GPIO_LCD_ID);
        printk("ili9881c lcm id=%d, gpio=%d\n", lcm_id, GPIO_LCD_ID);
	return 0;
}

static const struct of_device_id lcm_of_ids[] = {
	{.compatible = "mediatek,lcm",},
	{}
};

static struct platform_driver lcm_driver = {
	.driver = {
		   .name = "mtk_lcm",
		   .owner = THIS_MODULE,
		   .probe = lcm_probe,
		   .of_match_table = lcm_of_ids,
	},
};

static int __init lcm_init_drv(void)
{
	pr_notice("LCM: Register lcm driver\n");
	if (platform_driver_register(&lcm_driver)) {
		pr_err("LCM: failed to register disp driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_exit_drv(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_notice("LCM: Unregister lcm driver done\n");
}

module_init(lcm_init_drv);
module_exit(lcm_exit_drv);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (720)
#define FRAME_HEIGHT (1280)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))
#define REGFLAG_DELAY             							0XFD
#define REGFLAG_END_OF_TABLE      							0xFE   // END OF REGISTERS MARKER


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
//#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

//<2015-12-10-Lever.ni, [A110][DRV]modify mipi timing
#define NS_TO_CYCLE(n, c)	((n) / (c))
//>2015-12-10-Lever.ni

       

//#define LCM_DSI_CMD_MODE

struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

#if 1 

static struct LCM_setting_table lcm_initialization_setting[] = {
	{0xFF,3,{0x98,0x81,0x03}},
	{0x01,1,{0x00}},
	{0x02,1,{0x00}},
	{0x03,1,{0x72}},
	{0x04,1,{0x00}},
	{0x05,1,{0x00}},
	{0x06,1,{0x09}},
	{0x07,1,{0x00}},
	{0x08,1,{0x00}},
	{0x09,1,{0x01}},
	{0x0a,1,{0x00}},
	{0x0b,1,{0x00}},
	{0x0c,1,{0x01}},
	{0x0d,1,{0x00}},
	{0x0e,1,{0x00}},
	{0x0f,1,{0x00}},
	{0x10,1,{0x00}},
	{0x11,1,{0x00}},
	{0x12,1,{0x00}},
	{0x13,1,{0x00}},
	{0x14,1,{0x00}},
	{0x15,1,{0x00}},  /////
	{0x16,1,{0x00}},  /////
	{0x17,1,{0x00}},  /////
	{0x18,1,{0x00}},
	{0x19,1,{0x00}},
	{0x1a,1,{0x00}},
	{0x1b,1,{0x00}},
	{0x1c,1,{0x00}},
	{0x1d,1,{0x00}},
	{0x1e,1,{0x40}},
	{0x1f,1,{0x80}},
	{0x20,1,{0x05}},
	{0x21,1,{0x02}},
	{0x22,1,{0x00}},
	{0x23,1,{0x00}},
	{0x24,1,{0x00}},
	{0x25,1,{0x00}},
	{0x26,1,{0x00}},
	{0x27,1,{0x00}},
	{0x28,1,{0x33}},
	{0x29,1,{0x02}},
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
	{0x35,1,{0x00}},
	{0x36,1,{0x00}},
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
	{0x5e,1,{0x11}},
	{0x5f,1,{0x01}},
	{0x60,1,{0x00}},
	{0x61,1,{0x15}},
	{0x62,1,{0x14}},
	{0x63,1,{0x0E}},
	{0x64,1,{0x0F}},
	{0x65,1,{0x0C}},
	{0x66,1,{0x0D}},
	{0x67,1,{0x06}},
	{0x68,1,{0x02}},
	{0x69,1,{0x02}},
	{0x6a,1,{0x02}},
	{0x6b,1,{0x02}},
	{0x6c,1,{0x02}},
	{0x6d,1,{0x02}},
	{0x6e,1,{0x07}},
	{0x6f,1,{0x02}},
	{0x70,1,{0x02}},
	{0x71,1,{0x02}},
	{0x72,1,{0x02}},
	{0x73,1,{0x02}},
	{0x74,1,{0x02}},
	{0x75,1,{0x01}},
	{0x76,1,{0x00}},
	{0x77,1,{0x14}},
	{0x78,1,{0x15}},
	{0x79,1,{0x0E}},
	{0x7a,1,{0x0F}},
	{0x7b,1,{0x0C}},
	{0x7c,1,{0x0D}},
	{0x7D,1,{0x06}},
	{0x7E,1,{0x02}},
	{0x7F,1,{0x02}},
	{0x80,1,{0x02}},
	{0x81,1,{0x02}},
	{0x82,1,{0x02}},
	{0x83,1,{0x02}},
	{0x84,1,{0x07}},
	{0x85,1,{0x02}},
	{0x86,1,{0x02}},
	{0x87,1,{0x02}},
	{0x88,1,{0x02}},
	{0x89,1,{0x02}},
	{0x8A,1,{0x02}},
	{0xFF,3,{0x98,0x81,0x04}},
	{0x6C,1,{0x15}},
	{0x6E,1,{0x2A}},   //
	{0x6F,1,{0x33}},
	{0x3A,1,{0x24}},
	{0x8D,1,{0x16}},
	{0x87,1,{0xBA}},
	{0x26,1,{0x76}},
	{0xB2,1,{0xD1}},
	{0xB5,1,{0x06}},
	//{0x31,1,{0x75}},  /////
	{0x35,1,{0x1F}},
	{0xFF,3,{0x98,0x81,0x01}},
	{0x22,1,{0x0A}},
	{0x31,1,{0x00}},
	{0x53,1,{0x83}},  ////
	{0x55,1,{0x9E}},
	{0x50,1,{0x95}},
	{0x51,1,{0x90}},
	{0x60,1,{0x28}},
	//{0x63,1,{0x00}},
	{0xA0,1,{0x08}},
	{0xA1,1,{0x15}},
	{0xA2,1,{0x20}},
	{0xA3,1,{0x10}},
	{0xA4,1,{0x12}},
	{0xA5,1,{0x24}},
	{0xA6,1,{0x18}},
	{0xA7,1,{0x1B}},
	{0xA8,1,{0x69}},
	{0xA9,1,{0x1A}},
	{0xAA,1,{0x25}},
	{0xAB,1,{0x5F}},
	{0xAC,1,{0x1A}},
	{0xAD,1,{0x19}},
	{0xAE,1,{0x4D}},
	{0xAF,1,{0x21}},
	{0xB0,1,{0x28}},
	{0xB1,1,{0x54}},
	{0xB2,1,{0x68}},
	{0xB3,1,{0x39}},
	{0xC0,1,{0x08}},
	{0xC1,1,{0x15}},
	{0xC2,1,{0x20}},
	{0xC3,1,{0x10}},
	{0xC4,1,{0x12}},
	{0xC5,1,{0x24}},
	{0xC6,1,{0x18}},
	{0xC7,1,{0x1A}},
	{0xC8,1,{0x69}},
	{0xC9,1,{0x1A}},
	{0xCA,1,{0x25}},
	{0xCB,1,{0x5F}},
	{0xCC,1,{0x1A}},
	{0xCD,1,{0x19}},
	{0xCE,1,{0x4D}},
	{0xCF,1,{0x21}},
	{0xD0,1,{0x28}},
	{0xD1,1,{0x54}},
	{0xD2,1,{0x68}},
	{0xD3,1,{0x39}},
	{0xFF,3,{0x98,0x81,0x00}},
	{0x11,1,{0x00 }}, 
	{REGFLAG_DELAY, 120, {}}, 
	{0x29,1,{0x00}}, 
	{REGFLAG_DELAY, 20, {}},     
	{0x35,1,{0x00}},
};
#endif 
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
    //<2015-12-10-Lever.ni, [A110][DRV]modify mipi timing 
        unsigned int cycle_time;
        unsigned int ui;
    //>2015-12-10-Lever.ni

	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// enable tearing-free
	params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if defined(LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
	params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif
	
	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573

	//params->dsi.DSI_WMEM_CONTI=0x3C; 
	//params->dsi.DSI_RMEM_CONTI=0x3E; 

		
	params->dsi.packet_size=256;

	// Video mode setting		
	params->dsi.intermediat_buffer_num = 2;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

        params->dsi.vertical_sync_active                                = 4;   //3// 
        params->dsi.vertical_backporch                                        = 18; 
        params->dsi.vertical_frontporch                                        = 10;   //2// 
        params->dsi.vertical_active_line                                = FRAME_HEIGHT; 

        params->dsi.horizontal_sync_active                                = 10; 
        params->dsi.horizontal_backporch                                = 85; 
        params->dsi.horizontal_frontporch                                = 65; 
        params->dsi.horizontal_active_pixel                                = FRAME_WIDTH; 

	// Bit rate calculation
	params->dsi.PLL_CLOCK				= 208;
	params->dsi.ssc_disable = 1;
	//<2015-12-10-Lever.ni, [A110][DRV]modify mipi timing 
    	ui = 1000/(params->dsi.PLL_CLOCK*2)+0x01;  //3.5
	cycle_time = 8000/(params->dsi.PLL_CLOCK*2)+0x01;   //20
    	params->dsi.LPX = NS_TO_CYCLE(0x50, cycle_time);  //0x50=80ns
        params->dsi.HS_TRAIL = NS_TO_CYCLE(0x82, cycle_time);   //>= 69.79
        params->dsi.HS_PRPR = NS_TO_CYCLE(0x60, cycle_time); //49.79 -- 99.69
        //printf("ili9806e lk params->dsi.HS_TRAIL=%d, HS_PRPR=%d\n\r", params->dsi.HS_TRAIL, params->dsi.HS_PRPR);
    	//>2015-12-10-Lever.ni
	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 1;
	params->dsi.HS_TRAIL = 15;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
}



static unsigned int lcm_compare_id(void)
{
	int   array[4];
	char  buffer[3];
	int   id;
	
	char  page1[]={0x98,0x81,0x01};
	char  page0[]={0x98,0x81,0x00};
	
	SET_RESET_PIN(0);
	MDELAY(200);
	SET_RESET_PIN(1);
	MDELAY(200);

	dsi_set_cmdq_V2(0xFF, 3, page1, 1);
	
	array[0] = 0x00033700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0x00,buffer, 1); 
	
	array[0] = 0x00033700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x01,buffer+1, 1);

	id = (buffer[0]<<8) | buffer[1];
	#ifdef BUILD_LK
	printf("%s, id = 0x%08x\n", __func__, id);//should be 0x00
    	#else
	printk("%s, id = 0x%08x\n", __func__, id);//should be 0x00
	#endif
	//return (id == 0x9881? 1 : 0);
	if(id == 0x9881)
	{
	#ifdef BUILD_LK
    		if(lcm_get_gpio_input(LCM_ID_PIN)) {
    	#else
    		if(lcm_get_gpio_input(GPIO_LCD_ID)) {
    	#endif
			return 0;
		}
		else
		{
			dsi_set_cmdq_V2(0xFF, 3, page0, 1);
		        array[0] = 0x00033700;// read id return two byte,version and id
        		dsi_set_cmdq(array, 1, 1);
        		read_reg_v2(0xDA,buffer, 1);
			return (buffer[0] == 0x01 ? 1 : 0);
		}

	}
	else
		return 0;
}


static void lcm_init(void)
{
	  SET_RESET_PIN(1);
	  MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_init_resume(void)
{
	SET_RESET_PIN(1);
	MDELAY(10);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(20);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	unsigned int data_array[2];

	data_array[0] = 0x00280500; // Display Off
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(10); 
	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(100);

#ifdef BUILD_LK
	printf("uboot %s\n", __func__);
#else
	printk("kernel %s\n", __func__);
#endif
}


static void lcm_resume(void)
{
#ifdef BUILD_LK
	printf("uboot %s\n", __func__);
#else
	printk("kernel %s\n", __func__);
#endif
//	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
	lcm_init_resume();
}

#if 0
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
	
#ifdef BUILD_LK
	printf("uboot %s\n", __func__);
#else
	printk("kernel %s\n", __func__);	
#endif

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(data_array, 7, 0);

}
#endif


LCM_DRIVER ili9881c_hd720_dsi_vdo_hsd_lcm_drv = 
{
    .name			= "ili9881c_hd720_dsi_vdo_hsd",
	.set_util_funcs = lcm_set_util_funcs,
	.compare_id     = lcm_compare_id,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
#if defined(LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    };
