// SPDX-License-Identifier: GPL-2.0+
/*
 * Nuvoton NCT6694 I2C adapter driver based on USB interface.
 *
 * Copyright (C) 2024 Nuvoton Technology Corp.
 */

#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mfd/nct6694.h>

#define DEFAULT_BR		2

/* Host interface */
#define REQUEST_I2C_MOD		0x03

/* Message Channel*/
/* Command 00h */
#define REQUEST_I2C_OFFSET	0x0000	/* OFFSET = SEL|CMD */
#define REQUEST_I2C_LEN		0x90
#define I2C_PORT_IDX		0x00
#define I2C_BR_IDX		0x01
#define I2C_ADDR_IDX		0x02
#define I2C_W_CNT_IDX		0x03
#define I2C_R_CNT_IDX		0x04

#define I2C_RD_IDX		0x50
#define I2C_WR_IDX		0x10

#define DRVNAME "nct6694-i2c"

#define SET_SPEED_FROM_PARAM(port, param) {	\
	if (param >= 0 && param <= 6)	\
		nct6694_i2c_adapter[port].priv.br = param;	\
	else	\
		nct6694_i2c_adapter[port].priv.br = DEFAULT_BR;  /* Default: 100K */ \
}

static int sp0 = 2;
module_param(sp0, int, 0444);
MODULE_PARM_DESC(sp0, "The I2C clock speed of port 0. Default:2(100K). The value can range\n"
		      "from 0 to 6, corresponding to 25K/50K/100K/200K/400K/800K/1M\n"
		      "respectively.\n");
static int sp1 = 2;
module_param(sp1, int, 0444);
MODULE_PARM_DESC(sp1, "The I2C clock speed of port 1. Default:2(100K). The value can range\n"
		      "from 0 to 6, corresponding to 25K/50K/100K/200K/400K/800K/1M\n"
		      "respectively.");
static int sp2 = 2;
module_param(sp2, int, 0444);
MODULE_PARM_DESC(sp2, "The I2C clock speed of port 2. Default:2(100K). The value can range\n"
		      "from 0 to 6, corresponding to 25K/50K/100K/200K/400K/800K/1M\n"
		      "respectively.");
static int sp3 = 2;
module_param(sp3, int, 0444);
MODULE_PARM_DESC(sp3, "The I2C clock speed of port 3. Default:2(100K). The value can range\n"
		      "from 0 to 6, corresponding to 25K/50K/100K/200K/400K/800K/1M\n"
		      "respectively.");
static int sp4 = 2;
module_param(sp4, int, 0444);
MODULE_PARM_DESC(sp4, "The I2C clock speed of port 4. Default:2(100K). The value can range\n"
		      "from 0 to 6, corresponding to 25K/50K/100K/200K/400K/800K/1M\n"
		      "respectively.");
static int sp5 = 2;
module_param(sp5, int, 0444);
MODULE_PARM_DESC(sp5, "The I2C clock speed of port 5. Default:2(100K). The value can range\n"
		      "from 0 to 6, corresponding to 25K/50K/100K/200K/400K/800K/1M\n"
		      "respectively.");

struct nct6694_i2c_data {
	struct nct6694 *nct6694;
	struct nct6694_i2c_adapter *adaps;
	int nr_adapter;
};

struct nct6694_i2c_priv {
	unsigned char port;
	unsigned char br;
};

struct nct6694_i2c_adapter {
	struct i2c_adapter adapter;
	struct nct6694_i2c_data *data;

	struct nct6694_i2c_priv priv;
};

static void fill_in_buffer(u8 port, u8 br, u8 addr, u8 w_cnt, u8 r_cnt,
			   unsigned char *to, unsigned char *from)
{
	int i;

	to[I2C_PORT_IDX] = port;
	to[I2C_BR_IDX] = br;
	to[I2C_ADDR_IDX] = addr;
	to[I2C_W_CNT_IDX] = w_cnt;
	to[I2C_R_CNT_IDX] = r_cnt;

	for (i = 0; i < w_cnt; i++)
		to[i + I2C_WR_IDX] = from[i];
}

static int nct6694_xfer(struct i2c_adapter *adapter, struct i2c_msg *msgs,
			int num)
{
	struct nct6694_i2c_adapter *adap = adapter->algo_data;
	struct nct6694_i2c_data *data = adap->data;
	unsigned char buf[REQUEST_I2C_LEN] = {0};
	int ret, i;

	for (i = 0; i < num ; i++) {
		struct i2c_msg *msg_temp = &msgs[i];

		if (msg_temp->len > 64)
			return -EPROTO;

		if (msg_temp->flags & I2C_M_RD) {
			fill_in_buffer(adap->priv.port, adap->priv.br,
				       i2c_8bit_addr_from_msg(msg_temp), 0,
				       msg_temp->len, buf, msg_temp->buf);
			ret = nct6694_setusb_rdata(data->nct6694, REQUEST_I2C_MOD,
						   REQUEST_I2C_OFFSET, REQUEST_I2C_LEN,
						   I2C_RD_IDX, msg_temp->len, buf);
			memcpy(msg_temp->buf, buf, msg_temp->len);
			if (ret < 0)
				return 0;
		} else {
			fill_in_buffer(adap->priv.port, adap->priv.br,
				       i2c_8bit_addr_from_msg(msg_temp),
				       msg_temp->len, 0, buf, msg_temp->buf);
			ret = nct6694_setusb_wdata(data->nct6694, REQUEST_I2C_MOD,
						   REQUEST_I2C_OFFSET, REQUEST_I2C_LEN,
						   buf);
			if (ret < 0)
				return 0;
		}
	}

	return num;
}

static u32 nct6694_func(struct i2c_adapter *adapter)
{
	return (I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL);
}

static const struct i2c_algorithm algorithm = {
	.master_xfer = nct6694_xfer,
	.functionality = nct6694_func,
};

#define NCT6694_I2C_ADAPTER(_port)	\
{	\
	.adapter = {	\
		.owner = THIS_MODULE,	\
		.algo  = &algorithm,	\
	},	\
	.priv = {	\
		.port = _port	\
	}	\
}

static struct nct6694_i2c_adapter nct6694_i2c_adapter[] = {
	NCT6694_I2C_ADAPTER(0),
	NCT6694_I2C_ADAPTER(1),
	NCT6694_I2C_ADAPTER(2),
	NCT6694_I2C_ADAPTER(3),
	NCT6694_I2C_ADAPTER(4),
	NCT6694_I2C_ADAPTER(5),
};

static int nct6694_i2c_probe(struct platform_device *pdev)
{
	struct nct6694_i2c_data *data;
	struct nct6694 *nct6694 = dev_get_drvdata(pdev->dev.parent);
	int i;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->nct6694 = nct6694;
	data->adaps = nct6694_i2c_adapter;
	data->nr_adapter = ARRAY_SIZE(nct6694_i2c_adapter);

	//Init speed info by param or default value.
	SET_SPEED_FROM_PARAM(0, sp0);
	SET_SPEED_FROM_PARAM(1, sp1);
	SET_SPEED_FROM_PARAM(2, sp2);
	SET_SPEED_FROM_PARAM(3, sp3);
	SET_SPEED_FROM_PARAM(4, sp4);
	SET_SPEED_FROM_PARAM(5, sp5);

	platform_set_drvdata(pdev, data);

	/* Register each i2c adapter to I2C framework */
	for (i = 0; i < data->nr_adapter; i++) {
		struct nct6694_i2c_adapter *adapter = &data->adaps[i];

		adapter->priv.port = i;
		adapter->data = data;
		adapter->adapter.algo_data = adapter;
		sprintf(adapter->adapter.name, "NCT6694 I2C Adapter %d", i);

		i2c_add_adapter(&data->adaps[i].adapter);
	}

	dev_info(&pdev->dev, "Probe device :%s", pdev->name);

	return 0;
}

static int nct6694_i2c_remove(struct platform_device *pdev)
{
	struct nct6694_i2c_data *data = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < data->nr_adapter; i++)
		i2c_del_adapter(&data->adaps[i].adapter);

	kfree(data);

	return 0;
}

static struct platform_driver nct6694_i2c_driver = {
	.driver = {
		.name	= DRVNAME,
	},
	.probe		= nct6694_i2c_probe,
	.remove		= nct6694_i2c_remove,
};

static int __init nct6694_init(void)
{
	int err;

	err = platform_driver_register(&nct6694_i2c_driver);
	if (!err) {
		pr_info(DRVNAME ": platform_driver_register\n");
		if (err)
			platform_driver_unregister(&nct6694_i2c_driver);
	}

	return err;
}
subsys_initcall(nct6694_init);

static void __exit nct6694_exit(void)
{
	platform_driver_unregister(&nct6694_i2c_driver);
}
module_exit(nct6694_exit);

MODULE_DESCRIPTION("USB-i2c adapter driver for NCT6694");
MODULE_AUTHOR("Tzu-Ming Yu <tmyu0@nuvoton.com>");
MODULE_LICENSE("GPL");

