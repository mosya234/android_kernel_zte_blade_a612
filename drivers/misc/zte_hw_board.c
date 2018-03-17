/*
 *  kernel/drivers/misc/zte_board.c
 *
 *  Copyright (C) 2012 ZTE
 *
 * Modified for MSM8916 zte hardware board ID control.
 */
 
 #include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/err.h>
#include <linux/kobject.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>

/*zte-modify by zhengcz 20161114,begin*/
extern int sec_schip_enabled(void);
/*zte-modify by zhengcz 20161114,end*/

#define ZTE_BOARD_VERSION_REGISTER_MISC_DEV

#define GPIO_LABLE_NAME_LEN 30
#define BOARD_VERSION_NAME_LEN 30

struct hwversion_gpio {
	int gpio;
	char name[GPIO_LABLE_NAME_LEN];
};

struct board_info_type {
	u32 value;
	char version[BOARD_VERSION_NAME_LEN];
};

struct hwversion_platform_data
{
	int num_gpios;
	int num_versions;
	struct hwversion_gpio *gpios;
	struct board_info_type *board_info;
};

static struct hwversion_platform_data hwversion_info;

static struct kobject *zte_board_id_kobj;

static ssize_t zte_board_id_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	int i, gpio_val;
	u32 value = 0;
	ssize_t ret = 0;

	ret = sprintf(buf, "%s\n", hwversion_info.board_info[0].version);

	for(i = hwversion_info.num_gpios - 1; i >= 0; i--)
		gpio_direction_input(hwversion_info.gpios[i].gpio);

	for(i = hwversion_info.num_gpios - 1; i >= 0; i--) {
		value <<= 1;
		if(gpio_get_value(hwversion_info.gpios[i].gpio))
			value |= 0x1;
		gpio_val=gpio_get_value(hwversion_info.gpios[i].gpio);
		pr_info("zte board id  gpio value=%d\n",gpio_val);
	}	

	for(i = 0; i < hwversion_info.num_versions; i++) {
		if(value == hwversion_info.board_info[i].value){
			pr_info("zte board info_value =%d\n",hwversion_info.board_info[i].value);
			return sprintf(buf, "%s\n", hwversion_info.board_info[i].version);
		}	
	}
	return ret;
}

static struct kobj_attribute zte_board_id_attribute =
	__ATTR(board_id, 0444, zte_board_id_show, NULL);

/*zte-modify by zhengcz 20161114,begin*/
static ssize_t zte_sec_state_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	int ret = 0;
	ret = sec_schip_enabled();
	pr_info("zte_sec_state_show(%d)\n",ret);
	return sprintf(buf, "%d\n", ret);
}

static struct kobj_attribute zte_sec_state_attribute =
	__ATTR(secstate, 0444, zte_sec_state_show, NULL);
/*zte-modify by zhengcz 20161114,end*/

static struct attribute *attrs[] = {
	&zte_board_id_attribute.attr,
	&zte_sec_state_attribute.attr,    //add by zhengcz
	NULL,	/* need to NULL terminate the list of attributes */
};

static struct attribute_group attr_group = {
	.attrs = attrs,
};

#ifdef ZTE_BOARD_VERSION_REGISTER_MISC_DEV
static ssize_t board_show_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i, gpio_val;
	u32 value = 0;

	for(i = hwversion_info.num_gpios - 1; i >= 0; i--)
		gpio_direction_input(hwversion_info.gpios[i].gpio);

	for(i = hwversion_info.num_gpios - 1; i >= 0; i--) {
		value <<= 1;
		if(gpio_get_value(hwversion_info.gpios[i].gpio))
			value |= 0x1;
		gpio_val=gpio_get_value(hwversion_info.gpios[i].gpio);
		pr_info("zte board id  gpio value=%d\n",gpio_val);
	}	

	return sprintf(buf, "%d\n", value + 1);
}

static struct miscdevice board_version_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "zte_boardid",
	.fops = NULL,
};

static DEVICE_ATTR(id,      0664, board_show_id,   NULL);

static int zte_ddrsize_GB = 0;
static int __init ddrsize_setup(char *str)
{
	zte_ddrsize_GB = simple_strtol(str, NULL, 0);
	return 1;
}
__setup("ddrsize_GB=", ddrsize_setup);

static ssize_t zte_ddrsize_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	return sprintf(buf, "%d\n", zte_ddrsize_GB);
}
static DEVICE_ATTR(ddrsize, 0444, zte_ddrsize_show, NULL);

static struct device_attribute *misc_dev_attr_list[] = {
	&dev_attr_id,
	&dev_attr_ddrsize,
};
#endif


static int hw_version_probe(struct platform_device *pdev)
{
	struct pinctrl *pinctrl;
	int ret = 0;
	u32 temp;
	int len;
	int i;
	struct property *prop;

	pinctrl = pinctrl_get_select(&pdev->dev, "zte_board_hw_version0");
	if (IS_ERR(pinctrl)) {
		dev_err(&pdev->dev, "gpio versio0 is not configured from the driver\n");
		return -EINVAL;		
	}
	pinctrl = pinctrl_get_select(&pdev->dev, "zte_board_hw_version1");
	if (IS_ERR(pinctrl)) {
		dev_err(&pdev->dev, "gpio version1 is not configured from the driver\n");
		//return -EINVAL;		
	}
	pinctrl = pinctrl_get_select(&pdev->dev, "zte_board_hw_version2");
	if (IS_ERR(pinctrl)) {
		dev_err(&pdev->dev, "gpio version2 is not configured from the driver\n");
		//return -EINVAL;		
	}

	if(pdev->dev.of_node){
		if (!of_property_read_u32(pdev->dev.of_node, "num-gpios", &temp)) {
			hwversion_info.num_gpios= temp;
			
		} else{
			dev_err(&pdev->dev, "num-gpios is not configured from the driver\n");
			return -EINVAL;
		}

		if(hwversion_info.num_gpios) {
			hwversion_info.gpios = kzalloc(sizeof(struct hwversion_gpio) * hwversion_info.num_gpios, GFP_KERNEL);
			if(!(hwversion_info.gpios)) {
				dev_err(&pdev->dev, "out of memory, hwversion_gpio allocate failed.\n");
				return -ENOMEM;
			}
			
			for(i = 0; i < hwversion_info.num_gpios; i++) {
				snprintf(hwversion_info.gpios[i].name, GPIO_LABLE_NAME_LEN, "board-hwversion%d", i);
				ret = of_get_named_gpio(pdev->dev.of_node, "board-hwversions-gpios", i);
				if(gpio_is_valid(ret))
					hwversion_info.gpios[i].gpio = ret;
				else {
					ret = -EINVAL;
					goto gpio_get_err;
				}
				
				ret = gpio_request(hwversion_info.gpios[i].gpio, hwversion_info.gpios[i].name);
				if(ret) {
					dev_err(&pdev->dev, "gpio request failed");
					goto gpio_get_err;
				}
				gpio_direction_input(hwversion_info.gpios[i].gpio);
			}
		}

		//checkout board version name
		prop = of_find_property(pdev->dev.of_node, "board-info-value", &len);
		if (!prop) {
			ret = -EINVAL;
			goto board_info_value_get_err;
		}
		
		hwversion_info.num_versions = len/sizeof(u32);

		hwversion_info.board_info = kzalloc(sizeof(struct board_info_type) * hwversion_info.num_versions, GFP_KERNEL);
		if(!(hwversion_info.board_info)) {
				dev_err(&pdev->dev, "out of memory, board_info allocate failed.\n");
				ret = -ENOMEM;
				goto board_info_value_get_err;
		}
			
		for (i = 0; i < hwversion_info.num_versions; i++) {
			of_property_read_u32_index(pdev->dev.of_node, "board-info-value", i, &(hwversion_info.board_info[i].value));
		}
		
		len = of_property_count_strings(pdev->dev.of_node, "board-info-version");
		if (len != hwversion_info.num_versions) {
			pr_err("Invalid number of version name used.\n");
			ret = -EINVAL;
			goto board_version_get_error;
		}

		for (i = 0; i < len; i++) {
			const char *name = NULL;
			of_property_read_string_index(pdev->dev.of_node, "board-info-version", i, &name);
			strncpy(hwversion_info.board_info[i].version, name, BOARD_VERSION_NAME_LEN - 1);
			hwversion_info.board_info[i].version[BOARD_VERSION_NAME_LEN - 1] = '\0';
		}
	}

	zte_board_id_kobj = kobject_create_and_add("zte_board_id", NULL);
	if (!zte_board_id_kobj) {
		ret = -EINVAL;
		goto board_version_get_error;
	}
	/* Create the files associated with this kobject */
	ret = sysfs_create_group(zte_board_id_kobj, &attr_group);
	if (ret) {
		kobject_put(zte_board_id_kobj);
		goto board_version_get_error;
	}

	/*zte-modify by zhengcz 20161114,end*/
	ret = sec_schip_enabled();
	pr_info("sec_schip_enabled(%d)\n",ret);
	/*zte-modify by zhengcz 20161114,end*/

	#ifdef ZTE_BOARD_VERSION_REGISTER_MISC_DEV
	ret = misc_register(&board_version_dev);
	if (!ret) {
		device_create_file(board_version_dev.this_device, misc_dev_attr_list[0]);
		device_create_file(board_version_dev.this_device, misc_dev_attr_list[1]);
	}
	#endif


	return ret;
	
	board_version_get_error:
		kfree(hwversion_info.board_info);

	board_info_value_get_err:
		for(i = 0; i < hwversion_info.num_gpios; i++) {
			gpio_free(hwversion_info.gpios[i].gpio);
		}

	gpio_get_err:
		kfree(hwversion_info.gpios);
	return ret;
	
}
static int hw_version_remove(struct platform_device *pdev)
{
	int i;

	kobject_put(zte_board_id_kobj);
	
	for(i = 0; i < hwversion_info.num_gpios; i++) {
		gpio_free(hwversion_info.gpios[i].gpio);
	}

	kfree(hwversion_info.gpios);
	kfree(hwversion_info.board_info);

	return 0;
}


static struct of_device_id hwversion_match_table[] = {
	{ .compatible = "zte,board-hwversion",},
	{ },
};

static struct platform_driver hw_id_driver = {
	.probe = hw_version_probe,
	.remove = hw_version_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "zte-board-hwversion",
		.of_match_table = hwversion_match_table,
	},
};

static int __init zte_board_hwversion_init(void)
{
 	return platform_driver_register(&hw_id_driver);
}

static void __exit zte_board_hwversion_exit(void)
{
	platform_driver_unregister(&hw_id_driver);
}

module_init(zte_board_hwversion_init);
module_exit(zte_board_hwversion_exit);
