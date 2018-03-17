#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/errno.h>
#include <linux/spi/spi.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include "fingerprint_core.h"


static struct spi_board_info spi_boardinfo[] = {
        {
            .modalias  = "fingerprint_spi",
            .max_speed_hz  = 48000000,
            .bus_num  = 0,
            .chip_select  = 0,
            .mode = SPI_MODE_0,
        },
};

struct of_device_id fp_of_match[] = {	
	{ .compatible = "fp,fingerprint_ree", },	
	{},
};
struct fingerprint fp;

static int fp_probe(struct platform_device *pdev)
{		
	struct pinctrl_state *spi_state;
	struct device_node *node = NULL;
	u32 ints[2] = { 0, 0 };

	fp.pc = devm_pinctrl_get(&pdev->dev);    
	if (IS_ERR(fp.pc)) {        
		pr_err("fp:get pinctrl config failed");        
		return PTR_ERR(fp.pc);    
		}	
	fp.ps_pwr_on = pinctrl_lookup_state(fp.pc, "fp_state_pwr_on");	
	fp.ps_pwr_off = pinctrl_lookup_state(fp.pc, "fp_state_pwr_off");	
	fp.ps_rst_low = pinctrl_lookup_state(fp.pc, "fp_state_rst_output0");	
	fp.ps_rst_high = pinctrl_lookup_state(fp.pc, "fp_state_rst_output1");	
	fp.ps_irq_init = pinctrl_lookup_state(fp.pc, "fp_state_eint_pull_down");	
	fp.ps_irq_en = pinctrl_lookup_state(fp.pc, "fp_state_eint_as_int");	
	fp.ps_id_up = pinctrl_lookup_state(fp.pc, "fp_state_id_up");	
	fp.ps_id_down = pinctrl_lookup_state(fp.pc, "fp_state_id_down");	
	if (IS_ERR(fp.ps_pwr_on) || IS_ERR(fp.ps_pwr_off)		 
		|| IS_ERR(fp.ps_rst_low) || IS_ERR(fp.ps_rst_high) || IS_ERR(fp.ps_irq_init)		 
		|| IS_ERR(fp.ps_irq_en) || IS_ERR(fp.ps_id_up) || IS_ERR(fp.ps_id_down)) {		
		pr_err("fp:lookup pinctrl state failed");		
		return -1;	
	}	
	spi_state = pinctrl_lookup_state(fp.pc, "fp_default");
	pinctrl_select_state(fp.pc, spi_state);
	
	node = of_find_matching_node(node, fp_of_match);
	if(node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));	
		gpio_request(ints[0], "fp_irq");
		gpio_set_debounce(ints[0], ints[1]);
		fp.irq_gpio = ints[0];
		fp.irq_num =  irq_of_parse_and_map(node, 0);
		printk(KERN_ERR "fp: irq number %d", fp.irq_num);		
		if (!fp.irq_num) {
			pr_err("Failed to get irq number.");
			return -2;
		}
    }
	pr_info("fp:platform_driver probe success");	
	
	spi_register_board_info(spi_boardinfo, ARRAY_SIZE(spi_boardinfo));
	return 0;
}

static int fp_remove(struct platform_device *pdev)
{
	gpio_free(fp.irq_gpio);
	return 0;
}

static struct platform_driver fp_driver = {     
	.remove = fp_remove,   
	.shutdown = NULL,    
	.probe = fp_probe,    
	.driver = {             
		.name = "fp_config",            
		.owner = THIS_MODULE,            
		.of_match_table = fp_of_match,    
	},  
};

static int __init fingerprint_init(void)
{
	int status = 0;
	status = platform_driver_register(&fp_driver);
	if (status < 0){
		printk(KERN_ERR "fp: Failed to register SPI platform driver.\n");
	}
	return status;
}

static void __exit fingerprint_exit(void)
{
	platform_driver_unregister(&fp_driver);
}

module_init(fingerprint_init);
module_exit(fingerprint_exit);