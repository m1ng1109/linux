// SPDX-License-Identifier: GPL-2.0+
/*
 * Nuvoton NCT6694 GPIO controller driver based on USB interface.
 *
 * Copyright (C) 2024 Nuvoton Technology Corp.
 */

#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/mfd/nct6694.h>

#define DRVNAME "nct6694-gpio"

/* Host interface */
#define REQUEST_GPIO_MOD		0xFF
#define REQUEST_GPIO_LEN		0x01

/* Report Channel */
#define GPIO_VER_REG			0x90
#define GPIO_VALID_REG			0x110
#define GPI_DATA_REG			0x120
#define GPO_DIR_REG			0x170
#define GPO_TYPE_REG			0x180
#define GPO_DATA_REG			0x190

#define GPI_STS_REG			0x130
#define GPI_CLR_REG			0x140
#define GPI_FALLING_REG			0x150
#define GPI_RISING_REG			0x160

#define GPIO_BANK_NR			0xF	/* 16 banks gpio */

struct nct6694_gpio_data {
	struct nct6694 *nct6694;
	struct nct6694_gpio_bank *bank;
	struct work_struct irq_work;

	struct mutex irq_lock;
	int nr_bank;
	u8 irq_trig_type[32];	/* irq_trig_type | FALLING | RISING |*/
};

struct nct6694_gpio_bank {
	struct gpio_chip gpio;
	struct nct6694_gpio_data *data;

	/* Current gpio bank */
	unsigned char group;
};

static int nct6694_get_direction(struct gpio_chip *gpio, unsigned int offset)
{
	struct nct6694_gpio_bank *bank = gpiochip_get_data(gpio);
	struct nct6694_gpio_data *data = bank->data;
	unsigned char ret, buf;

	ret = nct6694_getusb(data->nct6694, REQUEST_GPIO_MOD,
			     GPO_DIR_REG + bank->group, REQUEST_GPIO_LEN,
			     0, 1, &buf);
	if (ret < 0) {
		pr_err("%s: Failed to get data from usb device!\n", __func__);
		return -EINVAL;
	}

	return !(BIT(offset) & buf);
}

static int nct6694_direction_input(struct gpio_chip *gpio, unsigned int offset)
{
	struct nct6694_gpio_bank *bank = gpiochip_get_data(gpio);
	struct nct6694_gpio_data *data = bank->data;
	unsigned char ret, buf;

	ret = nct6694_getusb(data->nct6694, REQUEST_GPIO_MOD,
			     GPO_DIR_REG + bank->group, REQUEST_GPIO_LEN,
			     0, 1, &buf);
	if (ret < 0) {
		pr_err("%s: Failed to get data from usb device!\n", __func__);
		return -EINVAL;
	}
	buf &= ~(1 << offset);
	ret = nct6694_setusb_wdata(data->nct6694, REQUEST_GPIO_MOD,
				   GPO_DIR_REG + bank->group, REQUEST_GPIO_LEN,
				   &buf);
	if (ret < 0) {
		pr_err("%s: Failed to set data to usb device!\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int nct6694_direction_output(struct gpio_chip *gpio,
				    unsigned int offset, int val)
{
	struct nct6694_gpio_bank *bank = gpiochip_get_data(gpio);
	struct nct6694_gpio_data *data = bank->data;
	unsigned char ret, buf;

	/* Set direction to output */
	ret = nct6694_getusb(data->nct6694, REQUEST_GPIO_MOD,
			     GPO_DIR_REG + bank->group, REQUEST_GPIO_LEN,
			     0, 1, &buf);
	if (ret < 0) {
		pr_err("%s: Failed to get data from usb device!\n", __func__);
		return -EINVAL;
	}
	buf |= (1 << offset);
	ret = nct6694_setusb_wdata(data->nct6694, REQUEST_GPIO_MOD,
				   GPO_DIR_REG + bank->group, REQUEST_GPIO_LEN,
				   &buf);
	if (ret < 0) {
		pr_err("%s: Failed to set data to usb device!\n", __func__);
		return -EINVAL;
	}

	/* Then set output level */
	ret = nct6694_getusb(data->nct6694, 0xFF, GPO_DATA_REG + bank->group,
			     REQUEST_GPIO_LEN, 0, 1, &buf);
	if (ret < 0) {
		pr_err("%s: Failed to get data from usb device!\n", __func__);
		return -EINVAL;
	}
	if (val)
		buf |= (1 << offset);
	else
		buf &= ~(1 << offset);
	ret = nct6694_setusb_wdata(data->nct6694, REQUEST_GPIO_MOD,
				   GPO_DATA_REG + bank->group, REQUEST_GPIO_LEN,
				   &buf);
	if (ret < 0) {
		pr_err("%s: Failed to set data to usb device!\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int nct6694_get_value(struct gpio_chip *gpio, unsigned int offset)
{
	struct nct6694_gpio_bank *bank = gpiochip_get_data(gpio);
	struct nct6694_gpio_data *data = bank->data;
	unsigned char ret, buf;

	ret = nct6694_getusb(data->nct6694, REQUEST_GPIO_MOD,
			     GPO_DIR_REG + bank->group, REQUEST_GPIO_LEN,
			     0, 1, &buf);
	if (ret < 0) {
		pr_err("%s: Failed to get data from usb device!\n", __func__);
		return -EINVAL;
	}

	if (BIT(offset) & buf) {
		ret = nct6694_getusb(data->nct6694, REQUEST_GPIO_MOD,
				     GPO_DATA_REG + bank->group, REQUEST_GPIO_LEN,
				     0, 1, &buf);
		if (ret < 0) {
			pr_err("%s: Failed to get data from usb device!\n", __func__);
			return -EINVAL;
		}
		return !!(BIT(offset) & buf);
	}

	ret = nct6694_getusb(data->nct6694, REQUEST_GPIO_MOD,
			     GPI_DATA_REG + bank->group, REQUEST_GPIO_LEN,
			     0, 1, &buf);
	if (ret < 0) {
		pr_err("%s: Failed to get data from usb device!\n", __func__);
		return -EINVAL;
	}
	return !!(BIT(offset) & buf);
}

static void nct6694_set_value(struct gpio_chip *gpio, unsigned int offset, int val)
{
	struct nct6694_gpio_bank *bank = gpiochip_get_data(gpio);
	struct nct6694_gpio_data *data = bank->data;
	unsigned char ret, buf;

	ret = nct6694_getusb(data->nct6694, REQUEST_GPIO_MOD,
			     GPO_DATA_REG + bank->group, REQUEST_GPIO_LEN,
			     0, 1, &buf);
	if (ret < 0)
		pr_err("%s: Failed to get data from usb device!\n", __func__);

	if (val)
		buf |= (1 << offset);
	else
		buf &= ~(1 << offset);

	ret = nct6694_setusb_wdata(data->nct6694, REQUEST_GPIO_MOD,
				   GPO_DATA_REG + bank->group, REQUEST_GPIO_LEN,
				   &buf);
	if (ret < 0)
		pr_err("%s: Failed to set data to usb device!\n", __func__);
}

static int nct6694_set_config(struct gpio_chip *gpio, unsigned int offset,
			      unsigned long config)
{
	struct nct6694_gpio_bank *bank = gpiochip_get_data(gpio);
	struct nct6694_gpio_data *data = bank->data;
	unsigned char ret, buf;

	ret = nct6694_getusb(data->nct6694, REQUEST_GPIO_MOD,
			     GPO_TYPE_REG + bank->group, REQUEST_GPIO_LEN,
			     0, 1, &buf);
	if (ret < 0)
		pr_err("%s: Failed to get data from usb device!\n", __func__);

	switch (pinconf_to_config_param(config)) {
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		buf |= (1 << offset);
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		buf &= ~(1 << offset);
		break;
	default:
		return -ENOTSUPP;
	}
	ret = nct6694_setusb_wdata(data->nct6694, REQUEST_GPIO_MOD,
				   GPO_TYPE_REG + bank->group, REQUEST_GPIO_LEN,
				   &buf);
	if (ret < 0)
		pr_err("%s: Failed to set data to usb device!\n", __func__);

	return 0;
}

static int nct6694_init_valid_mask(struct gpio_chip *gpio,
				   unsigned long *valid_mask,
				   unsigned int ngpios)
{
	struct nct6694_gpio_bank *bank = gpiochip_get_data(gpio);
	struct nct6694_gpio_data *data = bank->data;
	unsigned char ret, buf;

	ret = nct6694_getusb(data->nct6694, REQUEST_GPIO_MOD,
			     GPIO_VALID_REG + bank->group, REQUEST_GPIO_LEN,
			     0, 1, &buf);
	if (ret < 0)
		pr_err("%s: Failed to get data from usb device!\n", __func__);

	*valid_mask = buf;

	return 0;
}

#define NCT6694_GPIO_BANK(_group, _ngpio, _label)	\
{	\
	.gpio = {	\
		.label            = _label,	\
		.owner            = THIS_MODULE,	\
		.direction_input  = nct6694_direction_input,	\
		.get              = nct6694_get_value,	\
		.direction_output = nct6694_direction_output,	\
		.set              = nct6694_set_value,	\
		.get_direction	  = nct6694_get_direction,	\
		.set_config	  = nct6694_set_config,	\
		.init_valid_mask  = nct6694_init_valid_mask,	\
		.base             = -1,	\
		.ngpio            = _ngpio,	\
		.can_sleep        = false,	\
	},	\
	.group = _group,	\
}

static struct nct6694_gpio_bank nct6694_gpio_bank[] = {
	NCT6694_GPIO_BANK(0, 8, "GPIO0"),
	NCT6694_GPIO_BANK(1, 8, "GPIO1"),
	NCT6694_GPIO_BANK(2, 8, "GPIO2"),
	NCT6694_GPIO_BANK(3, 8, "GPIO3"),
	NCT6694_GPIO_BANK(4, 8, "GPIO4"),
	NCT6694_GPIO_BANK(5, 8, "GPIO5"),
	NCT6694_GPIO_BANK(6, 8, "GPIO6"),
	NCT6694_GPIO_BANK(7, 8, "GPIO7"),
	NCT6694_GPIO_BANK(8, 8, "GPIO8"),
	NCT6694_GPIO_BANK(9, 8, "GPIO9"),
	NCT6694_GPIO_BANK(10, 8, "GPIOA"),
	NCT6694_GPIO_BANK(11, 8, "GPIOB"),
	NCT6694_GPIO_BANK(12, 8, "GPIOC"),
	NCT6694_GPIO_BANK(13, 8, "GPIOD"),
	NCT6694_GPIO_BANK(14, 8, "GPIOE"),
	NCT6694_GPIO_BANK(15, 8, "GPIOF"),
};

static void nct6694_irq(struct work_struct *irq_work)
{
	struct nct6694_gpio_bank *bank = nct6694_gpio_bank;
	struct nct6694_gpio_data *data = bank->data;
	int ret, i;
	u8 status[GPIO_BANK_NR];

	ret = nct6694_getusb(data->nct6694, REQUEST_GPIO_MOD,
			     GPI_STS_REG, 16, 0, 16, status);
	if (ret < 0)
		pr_err("%s: Failed to get data from usb device!\n", __func__);

	for (i = 0; i < GPIO_BANK_NR; i++) {
		u8 stat = status[i];

		if (!stat)
			continue;

		while (stat) {
			int bit = __ffs(stat);
			u8 val = BIT(bit);

			handle_nested_irq(irq_find_mapping(bank[i].gpio.irq.domain, bit));
			stat &= ~(1 << bit);
			ret = nct6694_setusb_wdata(data->nct6694, REQUEST_GPIO_MOD,
						   GPI_CLR_REG + i, REQUEST_GPIO_LEN,
						   &val);
			if (ret < 0)
				pr_err("%s: Failed to set data to usb device!\n", __func__);
		}
	}
}

void nct6694_start_irq(struct nct6694 *nct6694)
{
	struct nct6694_gpio_bank *bank = nct6694_gpio_bank;
	struct nct6694_gpio_data *data = bank->data;

	queue_work(nct6694->async_workqueue, &data->irq_work);
}

static int nct6694_irq_trig(struct nct6694_gpio_data *data)
{
	int ret;

	ret = nct6694_getusb(data->nct6694, REQUEST_GPIO_MOD, GPI_FALLING_REG,
			     32, 0, 32, data->irq_trig_type);

	return ret;
}


static void nct6694_irq_mask(struct irq_data *d)
{
	struct nct6694_gpio_bank *bank = irq_data_get_irq_chip_data(d);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	gpiochip_disable_irq(&bank->gpio, hwirq);
}

static void nct6694_irq_unmask(struct irq_data *d)
{
	struct nct6694_gpio_bank *bank = irq_data_get_irq_chip_data(d);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	gpiochip_enable_irq(&bank->gpio, hwirq);
}

static int nct6694_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct nct6694_gpio_bank *bank = irq_data_get_irq_chip_data(d);
	struct nct6694_gpio_data *data = bank->data;
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	switch (type) {
	case IRQ_TYPE_EDGE_RISING:
		data->irq_trig_type[bank->group + 0x10] |= BIT(hwirq);
		break;

	case IRQ_TYPE_EDGE_FALLING:
		data->irq_trig_type[bank->group] |= BIT(hwirq);
		break;

	case IRQ_TYPE_EDGE_BOTH:
		data->irq_trig_type[bank->group] |= BIT(hwirq);
		data->irq_trig_type[bank->group + 0x10] |= BIT(hwirq);
		break;

	default:
		return -ENOTSUPP;
	}

	nct6694_setusb_async(data->nct6694, REQUEST_GPIO_MOD, GPI_FALLING_REG,
			     32, data->irq_trig_type);

	return 0;
}

static void nct6694_irq_bus_lock(struct irq_data *d)
{
	struct nct6694_gpio_bank *bank = irq_data_get_irq_chip_data(d);
	struct nct6694_gpio_data *data = bank->data;

	mutex_lock(&data->irq_lock);
}

static void nct6694_irq_bus_sync_unlock(struct irq_data *d)
{
	struct nct6694_gpio_bank *bank = irq_data_get_irq_chip_data(d);
	struct nct6694_gpio_data *data = bank->data;

	mutex_unlock(&data->irq_lock);
}

static const struct irq_chip nct6694_irq_chip = {
	.name			= "nct6694",
	.irq_mask		= nct6694_irq_mask,
	.irq_unmask		= nct6694_irq_unmask,
	.irq_set_type		= nct6694_irq_set_type,
	.irq_bus_lock		= nct6694_irq_bus_lock,
	.irq_bus_sync_unlock	= nct6694_irq_bus_sync_unlock,
	.flags			= IRQCHIP_IMMUTABLE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static int nct6694_gpio_probe(struct platform_device *pdev)
{
	int ret, i;
	struct nct6694_gpio_data *data;
	struct nct6694 *nct6694 = dev_get_drvdata(pdev->dev.parent);

	nct6694->gpio_handler = nct6694_start_irq;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->nct6694 = nct6694;
	data->bank = nct6694_gpio_bank;
	data->nr_bank = ARRAY_SIZE(nct6694_gpio_bank);
	INIT_WORK(&data->irq_work, nct6694_irq);

	mutex_init(&data->irq_lock);

	platform_set_drvdata(pdev, data);

	ret = nct6694_irq_trig(data);
	if (ret) {
		pr_err("Failed to get interrupt trigger type\n");
		kfree(data);
		return ret;
	}

	/* Register each gpio bank to GPIO framework */
	for (i = 0; i < data->nr_bank; i++) {
		struct nct6694_gpio_bank *bank = &data->bank[i];
		struct gpio_irq_chip *girq;

		bank->data  = data;

		girq = &data->bank[i].gpio.irq;
		gpio_irq_chip_set_chip(girq, &nct6694_irq_chip);
		girq->parent_handler = NULL;
		girq->num_parents = 0;
		girq->parents = NULL;
		girq->default_type = IRQ_TYPE_NONE;
		girq->handler = handle_level_irq;
		girq->threaded = true;

		ret = gpiochip_add_data(&data->bank[i].gpio, &data->bank[i]);
		if (ret) {
			dev_err(&pdev->dev, "devm_gpiochip_add_data failed: %d", ret);
			return ret;
		}
	}

	dev_info(&pdev->dev, "Probe device :%s", pdev->name);

	return 0;
}

static int nct6694_gpio_remove(struct platform_device *pdev)
{
	struct nct6694_gpio_data *data = platform_get_drvdata(pdev);
	int ret, i;

	ret = cancel_work(&data->irq_work);

	for (i = 0; i < data->nr_bank; i++)
		gpiochip_remove(&data->bank[i].gpio);

	kfree(data);

	return 0;
}

static struct platform_driver nct6694_gpio_driver = {
	.driver = {
		.name	= DRVNAME,
	},
	.probe		= nct6694_gpio_probe,
	.remove		= nct6694_gpio_remove,
};

static int __init nct6694_init(void)
{
	int err;

	err = platform_driver_register(&nct6694_gpio_driver);
	if (!err) {
		pr_info(DRVNAME ": platform_driver_register\n");
		if (err)
			platform_driver_unregister(&nct6694_gpio_driver);
	}

	return err;
}
subsys_initcall(nct6694_init);

static void __exit nct6694_exit(void)
{
	platform_driver_unregister(&nct6694_gpio_driver);
}
module_exit(nct6694_exit);

MODULE_DESCRIPTION("USB-GPIO controller driver for NCT6694");
MODULE_AUTHOR("Tzu-Ming Yu <tmyu0@nuvoton.com>");
MODULE_LICENSE("GPL");

