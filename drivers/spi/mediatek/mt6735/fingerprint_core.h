#ifndef FINGERPRINT_CORE_H
#define FINGERPRINT_CORE_H

struct fingerprint{
	struct pinctrl *pc;	
	struct pinctrl_state *ps_pwr_on;	
	struct pinctrl_state *ps_pwr_off;	
	struct pinctrl_state *ps_rst_low;	
	struct pinctrl_state *ps_rst_high;	
	struct pinctrl_state *ps_irq_init;		
	struct pinctrl_state *ps_irq_en;	
	struct pinctrl_state *ps_id_up;	
	struct pinctrl_state *ps_id_down;
	int irq_gpio;
	int irq_num;
};

extern struct fingerprint fp;

#define get_gpio(name) fp.irq_##name

#define pin_select(name) (\
{\
	if((fp.pc != NULL) && (fp.ps_##name != NULL)){\
		pinctrl_select_state(fp.pc, fp.ps_##name);\
	}\
}\
)

#endif