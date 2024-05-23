// SPDX-License-Identifier: GPL-2.0+
/*
 * Nuvoton NCT6694 WDT driver based on USB interface.
 *
 * Copyright (C) 2024 Nuvoton Technology Corp.
 */

#include <linux/watchdog.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mfd/nct6694.h>

#define DRVNAME "nct6694-wdt"

#define NR_WDT	2
#define WATCHDOG_TIMEOUT	10
#define WATCHDOG_PRETIMEOUT	0

/* Host interface */
#define REQUEST_WDT_MOD		0x07

/* Message Channel*/
/* Command 00h */
#define REQUEST_WDT_CMD0_LEN	0x0F
#define REQUEST_WDT_CMD0_OFFSET(idx) (idx ? 0x0100 : 0x0000)	/* OFFSET = SEL|CMD */
#define WDT_PRETIMEOUT_IDX	0x00
#define WDT_PRETIMEOUT_LEN	0x04	/* PRETIMEOUT(3byte) | ACT(1byte) */
#define WDT_IMEOUT_IDX		0x04
#define WDT_TIMEOUT_LEN		0x04	/* TIMEOUT(3byte) | ACT(1byte) */
#define WDT_COUNTDOWN_IDX	0x0C
#define WDT_COUNTDOWN_LEN	0x03

#define WDT_PRETIMEOUT_ACT	BIT(1)
#define WDT_TIMEOUT_ACT		BIT(1)


/* Command 01h */
#define REQUEST_WDT_CMD1_LEN	0x04
#define REQUEST_WDT_CMD1_OFFSET(idx) (idx ? 0x0101 : 0x0001)	/* OFFSET = SEL|CMD */
#define WDT_CMD_IDX		0x00
#define WDT_CMD_LEN		0x04

#define SET_BUF32(buf, u32_val) { \
	*(((u8 *)buf)+0) = u32_val & 0xFF; \
	*(((u8 *)buf)+1) = (u32_val>>8) & 0xFF; \
	*(((u8 *)buf)+2) = (u32_val>>16) & 0xFF; \
	*(((u8 *)buf)+3) = (u32_val>>24) & 0xFF; \
}

static unsigned int timeout;
module_param(timeout, int, 0);
MODULE_PARM_DESC(timeout, "Watchdog timeout in seconds");

static unsigned int pretimeout;
module_param(pretimeout, int, 0);
MODULE_PARM_DESC(pretimeout, "Watchdog pre-timeout in seconds");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
			   __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");


struct nct6694_wdt_data {
	struct nct6694 *nct6694;
	struct nct6694_wdt_dev *wdevs;
	int nr_wdev;
};

struct nct6694_wdt_dev {
	struct watchdog_device wdev;
	struct nct6694_wdt_data *data;
	unsigned int wdev_idx;
};


static int nct6694_wdt_start(struct watchdog_device *wdev)
{
	struct nct6694_wdt_dev *wdt = watchdog_get_drvdata(wdev);

	pr_info("%s: WDT(%d) Start\n", __func__, wdt->wdev_idx);

	return 0;
}

static int nct6694_wdt_stop(struct watchdog_device *wdev)
{
	struct nct6694_wdt_dev *wdt = watchdog_get_drvdata(wdev);
	struct nct6694 *nct6694 = wdt->data->nct6694;
	unsigned char buf[REQUEST_WDT_CMD1_LEN] = {'W', 'D', 'T', 'C'};
	int ret;

	pr_info("%s: WDT(%d) Close\n", __func__, wdt->wdev_idx);
	ret = nct6694_setusb_wdata(nct6694, REQUEST_WDT_MOD,
				   REQUEST_WDT_CMD1_OFFSET(wdt->wdev_idx),
				   REQUEST_WDT_CMD1_LEN, buf);
	if (ret)
		pr_err("%s: Failed to start WDT device!\n", __func__);

	return ret;
}

static int nct6694_wdt_ping(struct watchdog_device *wdev)
{
	struct nct6694_wdt_dev *wdt = watchdog_get_drvdata(wdev);
	struct nct6694 *nct6694 = wdt->data->nct6694;
	unsigned char buf[REQUEST_WDT_CMD1_LEN] = {'W', 'D', 'T', 'S'};
	int ret;

	pr_debug("%s: WDT(%d) Ping\n", __func__, wdt->wdev_idx);
	ret = nct6694_setusb_wdata(nct6694, REQUEST_WDT_MOD,
				   REQUEST_WDT_CMD1_OFFSET(wdt->wdev_idx),
				   REQUEST_WDT_CMD1_LEN, buf);
	if (ret)
		pr_err("%s: Failed to Ping WDT device!\n", __func__);

	return 0;
}

static int nct6694_wdt_set_timeout(struct watchdog_device *wdev,
				   unsigned int timeout)
{
	struct nct6694_wdt_dev *wdt = watchdog_get_drvdata(wdev);
	struct nct6694 *nct6694 = wdt->data->nct6694;
	unsigned int timeout_fmt, pretimeout_fmt;
	unsigned char buf[REQUEST_WDT_CMD0_LEN];
	int ret;

	if (timeout < wdev->pretimeout) {
		pr_err("%s: 'timeout' must be greater than 'pre timeout'!", __func__);
		return -EINVAL;
	}

	timeout_fmt = timeout * 1000 | (WDT_TIMEOUT_ACT << 24);
	pretimeout_fmt = wdev->pretimeout * 1000 | (WDT_PRETIMEOUT_ACT << 24);
	SET_BUF32(&buf[WDT_IMEOUT_IDX], le32_to_cpu(timeout_fmt));
	SET_BUF32(&buf[WDT_PRETIMEOUT_IDX], le32_to_cpu(pretimeout_fmt));

	ret = nct6694_setusb_wdata(nct6694, REQUEST_WDT_MOD,
				   REQUEST_WDT_CMD0_OFFSET(wdt->wdev_idx),
				   REQUEST_WDT_CMD0_LEN, buf);
	if (ret) {
		pr_err("%s: Don't write the setup command in Start stage!\n", __func__);
		return ret;
	}
	wdev->timeout = timeout;

	return 0;
}

static int nct6694_wdt_set_pretimeout(struct watchdog_device *wdev,
				      unsigned int pretimeout)
{
	struct nct6694_wdt_dev *wdt = watchdog_get_drvdata(wdev);
	struct nct6694 *nct6694 = wdt->data->nct6694;
	unsigned int timeout_fmt, pretimeout_fmt;
	unsigned char buf[REQUEST_WDT_CMD0_LEN];
	int ret;

	if (pretimeout > wdev->timeout) {
		pr_err("%s: 'pre timeout' must be less than 'timeout'!", __func__);
		return -EINVAL;
	}
	timeout_fmt = wdev->timeout * 1000 | (WDT_TIMEOUT_ACT << 24);
	pretimeout_fmt = pretimeout * 1000 | (WDT_PRETIMEOUT_ACT << 24);
	SET_BUF32(&buf[WDT_IMEOUT_IDX], le32_to_cpu(timeout_fmt));
	SET_BUF32(&buf[WDT_PRETIMEOUT_IDX], le32_to_cpu(pretimeout_fmt));

	ret = nct6694_setusb_wdata(nct6694, REQUEST_WDT_MOD,
					    REQUEST_WDT_CMD0_OFFSET(wdt->wdev_idx),
					    REQUEST_WDT_CMD0_LEN, buf);
	if (ret) {
		pr_info("%s: Don't write the setup command in Start stage!\n", __func__);
		return ret;
	}
	wdev->pretimeout = pretimeout;
	return 0;
}

static unsigned int nct6694_wdt_get_time(struct watchdog_device *wdev)
{
	struct nct6694_wdt_dev *wdt = watchdog_get_drvdata(wdev);
	struct nct6694 *nct6694 = wdt->data->nct6694;
	unsigned char buf[WDT_COUNTDOWN_LEN];
	unsigned int timeleft_ms;
	int ret;

	ret = nct6694_getusb(nct6694, REQUEST_WDT_MOD,
				      REQUEST_WDT_CMD0_OFFSET(wdt->wdev_idx),
				      REQUEST_WDT_CMD0_LEN, WDT_COUNTDOWN_IDX,
				      WDT_COUNTDOWN_LEN, buf);
	if (ret)
		pr_info("%s: Failed to get WDT device!\n", __func__);

	timeleft_ms = ((buf[2] << 16) | (buf[1] << 8) | buf[0]) & 0xFFFFFF;

	return timeleft_ms/1000;
}

static int nct6694_wdt_setup(struct watchdog_device *wdev)
{
	struct nct6694_wdt_dev *wdt = watchdog_get_drvdata(wdev);
	struct nct6694 *nct6694 = wdt->data->nct6694;
	unsigned char buf[REQUEST_WDT_CMD0_LEN] = {0};
	unsigned int timeout_fmt, pretimeout_fmt;
	int ret;

	if (timeout)
		wdev->timeout = timeout;

	if (pretimeout) {
		wdev->pretimeout = pretimeout;
		pretimeout_fmt = wdev->pretimeout*1000 | (WDT_PRETIMEOUT_ACT << 24);
	} else {
		pretimeout_fmt = 0;
	}

	timeout_fmt = wdev->timeout*1000 | (WDT_TIMEOUT_ACT << 24);
	SET_BUF32(&buf[WDT_IMEOUT_IDX], le32_to_cpu(timeout_fmt));
	SET_BUF32(&buf[WDT_PRETIMEOUT_IDX], le32_to_cpu(pretimeout_fmt));

	ret = nct6694_setusb_wdata(nct6694, REQUEST_WDT_MOD,
					    REQUEST_WDT_CMD0_OFFSET(wdt->wdev_idx),
					    REQUEST_WDT_CMD0_LEN, buf);
	if (ret)
		return ret;

	pr_info("Setting WDT(%d): timeout = %d, pretimeout = %d", wdt->wdev_idx,
								  wdev->timeout,
								  wdev->pretimeout);

	return 0;
}

static const struct watchdog_info nct6694_wdt_info = {
	.options = WDIOF_SETTIMEOUT		|
			   WDIOF_KEEPALIVEPING	|
				WDIOF_MAGICCLOSE	|
				WDIOF_PRETIMEOUT,
	.identity = DRVNAME,
};

static const struct watchdog_ops nct6694_wdt_ops = {
	.owner = THIS_MODULE,
	.start = nct6694_wdt_start,
	.stop = nct6694_wdt_stop,
	.ping = nct6694_wdt_ping,
	.set_timeout = nct6694_wdt_set_timeout,
	.set_pretimeout = nct6694_wdt_set_pretimeout,
	.get_timeleft = nct6694_wdt_get_time,
};

#define NCT6694_WDT_DEVICE(_wdev_idx)	\
{	\
	.wdev = {	\
		.info	= &nct6694_wdt_info,	\
		.ops  = &nct6694_wdt_ops,	\
		.timeout = WATCHDOG_TIMEOUT,	\
		.pretimeout = WATCHDOG_PRETIMEOUT,	\
		.min_timeout = 1,	\
		.max_timeout = 255,	\
	},	\
	.wdev_idx = _wdev_idx,	\
}

static struct nct6694_wdt_dev nct6694_wdt_dev[] = {
	NCT6694_WDT_DEVICE(0),
	NCT6694_WDT_DEVICE(1),
};


static int nct6694_wdt_probe(struct platform_device *pdev)
{
	struct nct6694_wdt_data *data;
	struct nct6694 *nct6694 = dev_get_drvdata(pdev->dev.parent);
	int i, ret;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->nct6694 = nct6694;
	data->wdevs = nct6694_wdt_dev;
	data->nr_wdev = ARRAY_SIZE(nct6694_wdt_dev);
	platform_set_drvdata(pdev, data);

	/* Register each wdt device to WDT framework */
	for (i = 0; i < data->nr_wdev; i++) {
		struct nct6694_wdt_dev *wdev = &data->wdevs[i];

		wdev->data = data;
		watchdog_set_drvdata(&data->wdevs[i].wdev, &data->wdevs[i]);
		watchdog_init_timeout(&data->wdevs[i].wdev, timeout, &pdev->dev);
		watchdog_set_nowayout(&data->wdevs[i].wdev, nowayout);
		watchdog_stop_on_reboot(&data->wdevs[i].wdev);

		ret = watchdog_register_device(&data->wdevs[i].wdev);
		if (ret) {
			dev_err(&pdev->dev, "Failed to register watchdog device: %d", ret);
			return ret;
		}

		ret = nct6694_wdt_setup(&data->wdevs[i].wdev);
		if (ret) {
			dev_err(&pdev->dev, "Failed to setup WDT device!");
			return ret;
		}
	}

	dev_info(&pdev->dev, "Probe device :%s", pdev->name);

	return 0;
}

static int nct6694_wdt_remove(struct platform_device *pdev)
{
	struct nct6694_wdt_data *data = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < data->nr_wdev; i++)
		watchdog_unregister_device(&data->wdevs[i].wdev);

	kfree(data);
	return 0;
}

static struct platform_driver nct6694_wdt_driver = {
	.driver = {
		.name	= DRVNAME,
	},
	.probe		= nct6694_wdt_probe,
	.remove		= nct6694_wdt_remove,
};

static int __init nct6694_init(void)
{
	int err;

	err = platform_driver_register(&nct6694_wdt_driver);
	if (!err) {
		pr_info(DRVNAME ": platform_driver_register\n");
		if (err)
			platform_driver_unregister(&nct6694_wdt_driver);
	}

	return err;
}
subsys_initcall(nct6694_init);

static void __exit nct6694_exit(void)
{
	platform_driver_unregister(&nct6694_wdt_driver);
}
module_exit(nct6694_exit);

MODULE_DESCRIPTION("USB-wdt driver for NCT6694");
MODULE_AUTHOR("Tzu-Ming Yu <tmyu0@nuvoton.com>");
MODULE_LICENSE("GPL");

