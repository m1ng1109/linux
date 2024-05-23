/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2024 Nuvoton Technology Corp.
 */

#ifndef __LINUX_MFD_NCT6694_H
#define __LINUX_MFD_NCT6694_H

#define NCT6694_VENDOR_ID		0x0416
#define NCT6694_PRODUCT_ID		0x200B
#define INT_IN_ENDPOINT			0x81
#define BULK_IN_ENDPOINT		0x82
#define BULK_OUT_ENDPOINT		0x03
#define MAX_PACKET_SZ			0x100

#define CMD_PACKET_SZ			0x8
#define HCTRL_SET			0x40
#define HCTRL_GET			0x80

#define REQUEST_MOD_IDX			0x01
#define REQUEST_CMD_IDX			0x02
#define REQUEST_SEL_IDX			0x03
#define REQUEST_HCTRL_IDX		0x04
#define REQUEST_LEN_L_IDX		0x06
#define REQUEST_LEN_H_IDX		0x07

#define RESPONSE_STS_IDX		0x01

#define GPIO_IRQ_STATUS			BIT(0)
#define CAN_IRQ_STATUS			BIT(2)
#define RTC_IRQ_STATUS			BIT(3)

#define URB_TIMEOUT			0

struct nct6694 {
	struct usb_device *udev;
	struct urb *int_in_urb;
	struct mutex access_lock;
	struct workqueue_struct *async_workqueue;
	struct work_struct async_work;

	unsigned char *tx_buffer;
	unsigned char *rx_buffer;
	unsigned char *cmd_buffer;
	unsigned char *int_buffer;
	unsigned char err_status;

	u8 MOD;
	u16 OFFSET;
	u16 LEN;
	unsigned char *buf;
	long timeout;
	int maxp;	/* Bulk pipe maximum packet for each transaction*/

	void (*gpio_handler)(struct nct6694 *nct6694);
	void (*can_handler)(struct nct6694 *nct6694);
	void (*rtc_handler)(struct nct6694 *nct6694);
};

extern int nct6694_getusb(struct nct6694 *nct6694, u8 MOD, u16 OFFSET, u16 LEN,
			  u8 rd_idx, u8 rd_len, unsigned char *buf);
extern int nct6694_setusb_rdata(struct nct6694 *nct6694, u8 MOD, u16 OFFSET,
				u16 LEN, u8 rd_idx, u8 rd_len, unsigned char *buf);
extern int nct6694_setusb_wdata(struct nct6694 *nct6694, u8 MOD, u16 OFFSET,
				u16 LEN, unsigned char *buf);
extern void nct6694_setusb_async(struct nct6694 *nct6694, u8 MOD, u16 OFFSET,
				 u16 LEN, unsigned char *buf);

#endif
