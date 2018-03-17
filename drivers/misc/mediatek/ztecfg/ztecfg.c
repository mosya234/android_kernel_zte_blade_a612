/*
 The function is not so perfect now.
 The fomat of data here is as follows:
 ps_cal_crosstalk, ps_high_threshold, ps_low_threshold, acc_offset_x, acc_offset_y, acc_offset_z;
 So it must be read in a particular order.
*/

#include <linux/fs.h>
#include <linux/slab.h>

//#include <asm/uaccess.h>
#include "ztecfg.h"

#define ProdFlagPart    "/dev/ztecfg"//"/dev/block/platform/mtk-msdc.0/11230000.msdc0/by-name/ztecfg"
#define ProdFlagBuff   524288//512*1024

int read_params_from_ztecfg(struct ztecfg_data *data_read)
{
	char *data = NULL;
	struct file *fp = NULL;
	int len = 0, size = 0, datalen = 3*sizeof(int);

	fp = filp_open(ProdFlagPart, O_RDWR, 0);
	if(IS_ERR(fp))
       {
       	printk("%s: open failed!err=%ld \n", __FUNCTION__, PTR_ERR(fp));
    	       goto meta_ReadProdFlag_op_fail;
      }

	len = fp->f_op->llseek(fp, 0, SEEK_END);
	if (len > ProdFlagBuff)
	{
       	printk("%s: fp->f_op->llseek failed! len=%d \n", __FUNCTION__, len);
    	       goto meta_ReadProdFlag_op_fail;
	}
	
      data = (char*) kmalloc(datalen, GFP_KERNEL);
      if(NULL == data)
      {
	      	printk("%s: not enough memory! \n", __FUNCTION__);
		filp_close(fp, NULL);
	      	goto meta_ReadProdFlag_op_fail;
      }
	memset((void *)data, -2, datalen);

	fp->f_op->llseek(fp, 0, SEEK_SET);
	
	size = fp->f_op->read(fp, data, sizeof(struct ztecfg_data), &fp->f_pos);
	if (size != sizeof(struct ztecfg_data))
       {
       	//printk("%s: read fail! size=%d result=%d\n", __FUNCTION__, sizeof(data_read), size);
		filp_close(fp, NULL);
       	goto meta_ReadProdFlag_op_fail;
       }

	filp_close(fp, NULL);

	memcpy((char *)data_read, data, sizeof(struct ztecfg_data));

	printk("%s ztecfg data read: %d %d %d %d %d %d\n", __func__, 
			data_read->ps_cal_crosstalk, data_read->ps_high_threshold, data_read->ps_low_threshold,
			data_read->acc_offset_x, data_read->acc_offset_y, data_read->acc_offset_z);
	
	return 0;

meta_ReadProdFlag_op_fail:
	if(NULL != data)
	{
		kfree(data);
		data = NULL;
	}
	
	return -1;
}

int write_params_to_ztecfg(int type, int para1, int para2, int para3)
{
	char *data = NULL;
	int *tmp = NULL;
	struct file *fp = NULL;
	int len = 0, datalen = 3*sizeof(int), size;
	struct ztecfg_data tmp2 = {0};

	printk("%s in: type=%d para1=%d para2=%d para3=%d\n", __func__, type, para1, para2, para3);

	fp = filp_open(ProdFlagPart, O_RDWR, 0);
	if(IS_ERR(fp))
       {
       	printk("%s: open failed!err=%ld \n", __FUNCTION__, PTR_ERR(fp));
    	       goto meta_SetProdFlag_op_fail;
      }

	len = fp->f_op->llseek(fp, 0, SEEK_END);
	if (len > ProdFlagBuff)
	{
       	printk("%s: fp->f_op->llseek failed! len=%d \n", __FUNCTION__, len);
    	       goto meta_SetProdFlag_op_fail;
	}

      data = (char*) kmalloc(datalen, GFP_KERNEL);
      if(NULL == data)
      {
	      	printk("%s: not enough memory! \n", __FUNCTION__);
		filp_close(fp, NULL);
	      	goto meta_SetProdFlag_op_fail;
      }
	memset((void *)data, -2, datalen);

	tmp = (int *)data;
	*tmp++ = para1;
	*tmp++ = para2;
	*tmp = para3;

	if(type == TYPE_PS)
		fp->f_op->llseek(fp, 0, SEEK_SET);
	else
		fp->f_op->llseek(fp, datalen, SEEK_SET);

	size = fp->f_op->write(fp, data, datalen, &fp->f_pos);
	if(size != datalen)
	{
	  	printk("%s: write fail!size=%d result=%d\n", __FUNCTION__, datalen, size);
	  	filp_close(fp, NULL);
	  	goto meta_SetProdFlag_op_fail;
	}

	filp_close(fp, NULL);

	read_params_from_ztecfg(&tmp2);

	if(NULL != data)
	{
		kfree(data);
		data = NULL;
	}
	
	return 0;

meta_SetProdFlag_op_fail:
	if(NULL != data)
	{
		kfree(data);
		data = NULL;
	}
	
	return -1;
}

