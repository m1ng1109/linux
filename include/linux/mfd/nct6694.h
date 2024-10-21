/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2024 Nuvoton Technology Corp.
 */

#define NCT6694_DEV_GPIO		"nct6694-gpio"
#define NCT6694_DEV_I2C			"nct6694-i2c"
#define NCT6694_DEV_CAN			"nct6694-can"
#define NCT6694_DEV_WDT			"nct6694-wdt"
#define NCT6694_DEV_IIO			"nct6694-iio"
#define NCT6694_DEV_HWMON		"nct6694-hwmon"
#define NCT6694_DEV_PWM			"nct6694-pwm"
#define NCT6694_DEV_RTC			"nct6694-rtc"

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

#define URB_TIMEOUT			1000

struct nct6694 {
	struct usb_device *udev;
	struct urb *int_in_urb;
	struct mutex access_lock;
	struct list_head handler_list;
	struct workqueue_struct *async_workqueue;
	struct work_struct async_work;
	unsigned char *tx_buffer;
	unsigned char *rx_buffer;
	unsigned char *cmd_buffer;
	unsigned char *int_buffer;
	unsigned char err_status;

	spinlock_t lock;
	long timeout;
	/* Bulk pipe maximum packet for each transaction*/
	int maxp;
};

struct nct6694_handler_entry {
	int bit_position;
	void (*handler)(void *private_data);
	void *private_data;
	struct list_head list;
};

int nct6694_register_handler(struct nct6694 *nct6694, int bit_position,
			     void (*handler)(void *), void *private_data);
int nct6694_read_msg(struct nct6694 *nct6694, u8 mod, u16 offset,
		     u16 length, u8 rd_idx, u8 rd_len, unsigned char *buf);
int nct6694_write_msg(struct nct6694 *nct6694, u8 mod, u16 offset,
		      u16 length, unsigned char *buf);
