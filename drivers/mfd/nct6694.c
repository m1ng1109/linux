// SPDX-License-Identifier: GPL-2.0+
/*
 * Nuvoton NCT6694 MFD driver based on USB interface.
 *
 * Copyright (C) 2024 Nuvoton Technology Corp.
 */

#include <linux/io.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mfd/core.h>
#include <linux/mfd/nct6694.h>

#define DRVNAME "nct6694"

/* MFD device resources */
static const struct mfd_cell nct6694_dev[] = {
	{
		.name = "nct6694-gpio",
	},
	{
		.name = "nct6694-i2c",
	},
	{
		.name = "nct6694-canfd",
	},
	{
		.name = "nct6694-wdt",
	},
	{
		.name = "nct6694-iio",
	},
	{
		.name = "nct6694-hwmon",
	},
	{
		.name = "nct6694-pwm",
	},
	{
		.name = "nct6694-rtc",
	},
};

/*
 * Get usb command packet
 * - Read the data which firmware provides.
 * - Packet format:
 *
 *		OUT	|RSV|MOD|CMD|SEL|HCTL|RSV|LEN_L|LEN_H|
 *		OUT	|SEQ|STS|RSV|RSV|RSV|RSV||LEN_L|LEN_H|
 *		IN	|-------D------A------D------A-------|
 *			|-------D------A------D------A-------|
 */
int nct6694_getusb(struct nct6694 *nct6694, u8 MOD, u16 OFFSET, u16 LEN,
		   u8 rd_idx, u8 rd_len, unsigned char *buf)
{
	int i, ret;
	unsigned char err_status;
	int len, packet_len, tx_len, rx_len;
	struct usb_device *udev = nct6694->udev;

	mutex_lock(&nct6694->access_lock);

	nct6694->cmd_buffer[REQUEST_MOD_IDX] = MOD;
	nct6694->cmd_buffer[REQUEST_CMD_IDX] = OFFSET & 0xFF;
	nct6694->cmd_buffer[REQUEST_SEL_IDX] = (OFFSET >> 8) & 0xFF;
	nct6694->cmd_buffer[REQUEST_HCTRL_IDX] = HCTRL_GET;
	nct6694->cmd_buffer[REQUEST_LEN_L_IDX] = LEN & 0xFF;
	nct6694->cmd_buffer[REQUEST_LEN_H_IDX] = (LEN >> 8) & 0xFF;

	ret = usb_bulk_msg(udev, usb_sndbulkpipe(udev, BULK_OUT_ENDPOINT),
			   nct6694->cmd_buffer, CMD_PACKET_SZ, &tx_len,
			   nct6694->timeout);
	if (ret)
		goto err;

	ret = usb_bulk_msg(udev, usb_rcvbulkpipe(udev, BULK_IN_ENDPOINT),
			   nct6694->rx_buffer, CMD_PACKET_SZ, &rx_len,
			   nct6694->timeout);
	if (ret)
		goto err;

	err_status = nct6694->rx_buffer[RESPONSE_STS_IDX];

	for (i = 0, len = LEN; len > 0; i++, len -= packet_len) {
		if (len > nct6694->maxp)
			packet_len = nct6694->maxp;
		else
			packet_len = len;

		ret = usb_bulk_msg(udev, usb_rcvbulkpipe(udev, BULK_IN_ENDPOINT),
				   nct6694->rx_buffer + nct6694->maxp * i,
				   packet_len, &rx_len, nct6694->timeout);
		if (ret)
			goto err;
	}

	for (i = 0; i < rd_len; i++)
		buf[i] = nct6694->rx_buffer[i + rd_idx];

	if (err_status) {
		pr_debug("%s: MSG CH status = %2Xh\n", __func__, err_status);
		ret = -EIO;
	}
err:
	mutex_unlock(&nct6694->access_lock);
	return ret;
}
EXPORT_SYMBOL(nct6694_getusb);

/*
 * Set usb command packet
 * - Read the data which firmware provides.
 * - Packet format:
 *
 *		OUT	|RSV|MOD|CMD|SEL|HCTL|RSV|LEN_L|LEN_H|
 *		OUT	|-------D------A------D------A-------|
 *		IN	|SEQ|STS|RSV|RSV|RSV|RSV||LEN_L|LEN_H|
 *		IN	|-------D------A------D------A-------|
 *			|-------D------A------D------A-------|
 */
int nct6694_setusb_rdata(struct nct6694 *nct6694, u8 MOD, u16 OFFSET, u16 LEN,
			 u8 rd_idx, u8 rd_len, unsigned char *buf)
{
	int i, ret;
	unsigned char err_status;
	int len, packet_len, tx_len, rx_len;
	struct usb_device *udev = nct6694->udev;

	mutex_lock(&nct6694->access_lock);

	nct6694->cmd_buffer[REQUEST_MOD_IDX] = MOD;
	nct6694->cmd_buffer[REQUEST_CMD_IDX] = OFFSET & 0xFF;
	nct6694->cmd_buffer[REQUEST_SEL_IDX] = (OFFSET >> 8) & 0xFF;
	nct6694->cmd_buffer[REQUEST_HCTRL_IDX] = HCTRL_SET;
	nct6694->cmd_buffer[REQUEST_LEN_L_IDX] = LEN & 0xFF;
	nct6694->cmd_buffer[REQUEST_LEN_H_IDX] = (LEN >> 8) & 0xFF;

	ret = usb_bulk_msg(udev, usb_sndbulkpipe(udev, BULK_OUT_ENDPOINT),
			   nct6694->cmd_buffer, CMD_PACKET_SZ, &tx_len,
			   nct6694->timeout);
	if (ret)
		goto err;

	for (i = 0, len = LEN; len > 0; i++, len -= packet_len) {
		if (len > nct6694->maxp)
			packet_len = nct6694->maxp;
		else
			packet_len = len;

		memcpy(nct6694->tx_buffer + nct6694->maxp * i,
		       buf + nct6694->maxp * i, packet_len);
		ret = usb_bulk_msg(udev, usb_sndbulkpipe(udev, BULK_OUT_ENDPOINT),
				   nct6694->tx_buffer + nct6694->maxp * i,
				   packet_len, &tx_len, nct6694->timeout);
		if (ret)
			goto err;
	}

	ret = usb_bulk_msg(udev, usb_rcvbulkpipe(udev, BULK_IN_ENDPOINT),
			   nct6694->rx_buffer, CMD_PACKET_SZ, &rx_len,
			   nct6694->timeout);
	if (ret)
		goto err;

	err_status = nct6694->rx_buffer[RESPONSE_STS_IDX];

	for (i = 0, len = LEN; len > 0; i++, len -= packet_len) {
		if (len > nct6694->maxp)
			packet_len = nct6694->maxp;
		else
			packet_len = len;

		ret = usb_bulk_msg(udev, usb_rcvbulkpipe(udev, BULK_IN_ENDPOINT),
				   nct6694->rx_buffer + nct6694->maxp * i,
				   packet_len, &rx_len, nct6694->timeout);
		if (ret)
			goto err;
	}

	for (i = 0; i < rd_len; i++)
		buf[i] = nct6694->rx_buffer[i + rd_idx];

	if (err_status) {
		pr_debug("%s: MSG CH status = %2Xh\n", __func__, err_status);
		ret = -1;
	}
err:
	mutex_unlock(&nct6694->access_lock);
	return ret;
}
EXPORT_SYMBOL(nct6694_setusb_rdata);

/*
 * Set usb command packet
 * - Write data to firmware.
 * - Packet format:
 *
 *		OUT	|RSV|MOD|CMD|SEL|HCTL|RSV|LEN_L|LEN_H|
 *		OUT	|-------D------A------D------A-------|
 *		IN	|SEQ|STS|RSV|RSV|RSV|RSV||LEN_L|LEN_H|
 *		IN	|-------D------A------D------A-------|
 *			|-------D------A------D------A-------|
 */
int nct6694_setusb_wdata(struct nct6694 *nct6694, u8 MOD, u16 OFFSET, u16 LEN,
			 unsigned char *buf)
{
	int i, ret;
	unsigned char err_status;
	int len, packet_len, tx_len, rx_len;
	struct usb_device *udev = nct6694->udev;

	mutex_lock(&nct6694->access_lock);

	nct6694->cmd_buffer[REQUEST_MOD_IDX] = MOD;
	nct6694->cmd_buffer[REQUEST_CMD_IDX] = OFFSET & 0xFF;
	nct6694->cmd_buffer[REQUEST_SEL_IDX] = (OFFSET >> 8) & 0xFF;
	nct6694->cmd_buffer[REQUEST_HCTRL_IDX] = HCTRL_SET;
	nct6694->cmd_buffer[REQUEST_LEN_L_IDX] = LEN & 0xFF;
	nct6694->cmd_buffer[REQUEST_LEN_H_IDX] = (LEN >> 8) & 0xFF;

	ret = usb_bulk_msg(udev, usb_sndbulkpipe(udev, BULK_OUT_ENDPOINT),
			   nct6694->cmd_buffer, CMD_PACKET_SZ, &tx_len,
			   nct6694->timeout);
	if (ret)
		goto err;

	for (i = 0, len = LEN; len > 0; i++, len -= packet_len) {
		if (len > nct6694->maxp)
			packet_len = nct6694->maxp;
		else
			packet_len = len;

		memcpy(nct6694->tx_buffer + nct6694->maxp * i,
		       buf + nct6694->maxp * i, packet_len);
		ret = usb_bulk_msg(udev, usb_sndbulkpipe(udev, BULK_OUT_ENDPOINT),
				   nct6694->tx_buffer + nct6694->maxp * i,
				   packet_len, &tx_len, nct6694->timeout);
		if (ret)
			goto err;
	}

	ret = usb_bulk_msg(udev, usb_rcvbulkpipe(udev, BULK_IN_ENDPOINT),
			   nct6694->rx_buffer, CMD_PACKET_SZ, &rx_len,
			   nct6694->timeout);
	if (ret)
		goto err;

	err_status = nct6694->rx_buffer[RESPONSE_STS_IDX];

	for (i = 0, len = LEN; len > 0; i++, len -= packet_len) {
		if (len > nct6694->maxp)
			packet_len = nct6694->maxp;
		else
			packet_len = len;

		ret = usb_bulk_msg(udev, usb_rcvbulkpipe(udev, BULK_IN_ENDPOINT),
				   nct6694->rx_buffer + nct6694->maxp * i,
				   packet_len, &rx_len, nct6694->timeout);
		if (ret)
			goto err;
	}

	if (err_status) {
		pr_debug("%s: MSG CH status = %2Xh\n", __func__, err_status);
		ret = -1;
	}
err:
	mutex_unlock(&nct6694->access_lock);
	return ret;
}
EXPORT_SYMBOL(nct6694_setusb_wdata);

static void setusb_work(struct work_struct *async_work)
{
	struct nct6694 *nct6694 = container_of(async_work, struct nct6694, async_work);
	int ret;

	ret = nct6694_setusb_wdata(nct6694, nct6694->MOD, nct6694->OFFSET,
				   nct6694->LEN, nct6694->buf);
}

/*
 * Set usb command packet
 * - Write data to firmware.
 * - Packet format:
 *
 *		OUT	|RSV|MOD|CMD|SEL|HCTL|RSV|LEN_L|LEN_H|
 *		OUT	|-------D------A------D------A-------|
 *		IN	|SEQ|STS|RSV|RSV|RSV|RSV||LEN_L|LEN_H|
 *		IN	|-------D------A------D------A-------|
 *			|-------D------A------D------A-------|
 */
void nct6694_setusb_async(struct nct6694 *nct6694, u8 MOD, u16 OFFSET, u16 LEN,
			  unsigned char *buf)
{
	nct6694->MOD = MOD;
	nct6694->OFFSET = OFFSET;
	nct6694->LEN = LEN;
	nct6694->buf = buf;
	queue_work(nct6694->async_workqueue, &nct6694->async_work);
}
EXPORT_SYMBOL(nct6694_setusb_async);

static void usb_int_callback(struct urb *urb)
{
	unsigned char *int_status = urb->transfer_buffer;
	struct nct6694 *nct6694 = urb->context;
	int ret;

	switch (urb->status) {
	case 0:
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		return;
	default:
		goto resubmit;
	}

	while (int_status[0]) {
		if (int_status[0] & GPIO_IRQ_STATUS) {
			if (nct6694->gpio_handler)
				nct6694->gpio_handler(nct6694);
			else
				pr_err("The GPIO interrupt handler is unattached!");
			int_status[0] &= ~GPIO_IRQ_STATUS;
		} else if (int_status[0] & CAN_IRQ_STATUS) {
			if (nct6694->can_handler)
				nct6694->can_handler(nct6694);
			else
				pr_err("The CAN interrupt handler is unattached!");
			int_status[0] &= ~CAN_IRQ_STATUS;
		} else if (int_status[0] & RTC_IRQ_STATUS) {
			if (nct6694->rtc_handler)
				nct6694->rtc_handler(nct6694);
			else {
				pr_err("The RTC interrupt handler is unattached!");
				int_status[0] &= ~RTC_IRQ_STATUS;
			}
		} else {
			pr_debug("Unsupported irq status! :%d", int_status[0]);
			break;
		}
	}

resubmit:
	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret)
		pr_debug("%s: Failed to resubmit urb, status %d", __func__, ret);
}

int nct6694_usb_probe(struct usb_interface *iface,
		      const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(iface);
	struct usb_host_interface *interface;
	struct usb_endpoint_descriptor *int_endpoint;
	struct nct6694 *nct6694;
	int ret = EINVAL;
	int pipe, maxp, bulk_pipe;

	interface = iface->cur_altsetting;
	/* Binding interface class : 0xFF */
	if (interface->desc.bInterfaceClass != USB_CLASS_VENDOR_SPEC ||
		interface->desc.bInterfaceSubClass != 0x00 ||
		interface->desc.bInterfaceProtocol != 0x00)
		return -ENODEV;

	int_endpoint = &interface->endpoint[0].desc;
	if (!usb_endpoint_is_int_in(int_endpoint))
		return -ENODEV;

	nct6694 = devm_kzalloc(&udev->dev, sizeof(*nct6694), GFP_KERNEL);
	if (!nct6694)
		return -ENOMEM;

	pipe = usb_rcvintpipe(udev, INT_IN_ENDPOINT);
	maxp = usb_maxpacket(udev, pipe);

	nct6694->cmd_buffer = devm_kzalloc(&udev->dev,
					   sizeof(unsigned char) * CMD_PACKET_SZ,
					   GFP_KERNEL);
	if (!(nct6694->cmd_buffer))
		return -ENOMEM;
	nct6694->rx_buffer = devm_kzalloc(&udev->dev,
					  sizeof(unsigned char) * MAX_PACKET_SZ,
					  GFP_KERNEL);
	if (!(nct6694->rx_buffer))
		return -ENOMEM;
	nct6694->tx_buffer = devm_kzalloc(&udev->dev,
					  sizeof(unsigned char) * MAX_PACKET_SZ,
					  GFP_KERNEL);
	if (!(nct6694->tx_buffer))
		return -ENOMEM;
	nct6694->int_buffer = devm_kzalloc(&udev->dev,
					   sizeof(unsigned char) * maxp,
					   GFP_KERNEL);
	if (!(nct6694->int_buffer))
		return -ENOMEM;
	nct6694->int_in_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!(nct6694->int_in_urb)) {
		dev_err(&udev->dev, "Failed to allocate INT-in urb!");
		return -ENOMEM;
	}

	/* Bulk pipe maximum packet for each transaction */
	bulk_pipe = usb_sndbulkpipe(udev, BULK_OUT_ENDPOINT);
	nct6694->maxp = usb_maxpacket(udev, bulk_pipe);

	mutex_init(&nct6694->access_lock);
	nct6694->udev = udev;
	nct6694->timeout = URB_TIMEOUT;	/* Wait until urb complete */

	usb_fill_int_urb(nct6694->int_in_urb, udev, pipe,
			 nct6694->int_buffer, maxp, usb_int_callback,
			 nct6694, int_endpoint->bInterval);
	ret = usb_submit_urb(nct6694->int_in_urb, GFP_KERNEL);
	if (ret)
		goto err_urb;

	dev_set_drvdata(&udev->dev, nct6694);
	usb_set_intfdata(iface, nct6694);

	ret = mfd_add_hotplug_devices(&udev->dev, nct6694_dev, ARRAY_SIZE(nct6694_dev));
	if (ret) {
		dev_err(&udev->dev, "Failed to add mfd's child device\n");
		goto err_mfd;
	}

	INIT_WORK(&nct6694->async_work, setusb_work);

	nct6694->async_workqueue = alloc_ordered_workqueue("asyn_workqueue", 0);

	dev_info(&udev->dev, "Probe device: (%04X:%04X)\n", id->idVendor, id->idProduct);
	return 0;

err_mfd:
	usb_kill_urb(nct6694->int_in_urb);
err_urb:
	usb_free_urb(nct6694->int_in_urb);

	return ret;
}

static void nct6694_usb_disconnect(struct usb_interface *iface)
{
	struct usb_device *udev = interface_to_usbdev(iface);
	struct nct6694 *nct6694 = usb_get_intfdata(iface);
	int ret;

	mfd_remove_devices(&udev->dev);
	ret = cancel_work_sync(&nct6694->async_work);
	flush_workqueue(nct6694->async_workqueue);
	destroy_workqueue(nct6694->async_workqueue);
	usb_set_intfdata(iface, NULL);
	usb_kill_urb(nct6694->int_in_urb);
	usb_free_urb(nct6694->int_in_urb);
}

static const struct usb_device_id nct6694_ids[] = {
	{ USB_DEVICE(NCT6694_VENDOR_ID, NCT6694_PRODUCT_ID)},
	{},
};
MODULE_DEVICE_TABLE(usb, nct6694_ids);

static struct usb_driver nct6694_usb_driver = {
	.name	= DRVNAME,
	.id_table = nct6694_ids,
	.probe = nct6694_usb_probe,
	.disconnect = nct6694_usb_disconnect,
};

module_usb_driver(nct6694_usb_driver);

MODULE_DESCRIPTION("USB-MFD driver for NCT6694");
MODULE_AUTHOR("Tzu-Ming Yu <tmyu0@nuvoton.com>");
MODULE_LICENSE("GPL");
