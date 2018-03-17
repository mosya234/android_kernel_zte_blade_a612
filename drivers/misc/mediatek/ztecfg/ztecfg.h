#ifndef __ZTECFG_H__
#define __ZTECFG_H__

enum
{
	TYPE_PS,
	TYPE_ACC,
};

struct ztecfg_data
{
	int ps_cal_crosstalk;
	int ps_high_threshold;
	int ps_low_threshold;
	int acc_offset_x;
	int acc_offset_y;
	int acc_offset_z;
};
extern int read_params_from_ztecfg(struct ztecfg_data *data_read);
extern int write_params_to_ztecfg(int type, int para1, int para2, int para3);

#endif
