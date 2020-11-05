/*
 * This driver creates fake 1-wire buses which can contain emulated devices,
 * implemented by a separate emulation driver. The API between this driver and
 * its emulators is defined by struct w1_emul_driver_api.
 *
 * Copyright (c) 2020 Alexander Mihajlovic <a@abxy.se>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_w1_emul_controller

#define LOG_LEVEL CONFIG_W1_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(w1_emul_ctlr);

#include <device.h>
#include <emul.h>
#include <drivers/w1.h>
#include <drivers/w1_emul.h>

/** Working data for the device */
struct w1_emul_data {
	/* List of struct w1_emul associated with the device */
	sys_slist_t emuls;
};

static int w1_emul_reset_bus(const struct device *dev)
{
	struct w1_emul_data *data = dev->data;
	sys_snode_t *node;

	int result = 0;

	SYS_SLIST_FOR_EACH_NODE(&data->emuls, node) {
		struct w1_emul *emul = NULL;
		const struct w1_emul_api *api;
		int ret;

		emul = CONTAINER_OF(node, struct w1_emul, node);

		api = emul->api;
		__ASSERT_NO_MSG(emul->api);
		__ASSERT_NO_MSG(emul->api->reset_bus);

		ret = api->reset_bus(emul);
		if (ret < 0) {
			return ret;
		}

		result = result || ret;
	}

	return result;
}

static int w1_emul_single_bit(const struct device *dev, int value)
{
	struct w1_emul_data *data = dev->data;
	sys_snode_t *node;

	__ASSERT_NO_MSG(value == 0 || value == 1);

	int result = value;

	SYS_SLIST_FOR_EACH_NODE(&data->emuls, node) {
		struct w1_emul *emul = NULL;
		const struct w1_emul_api *api;
		int bit;

		emul = CONTAINER_OF(node, struct w1_emul, node);

		api = emul->api;
		__ASSERT_NO_MSG(emul->api);
		__ASSERT_NO_MSG(emul->api->single_bit);

		bit = api->single_bit(emul, value);
		__ASSERT_NO_MSG(bit == 0 || bit == 1);

		/* If any device (including the controller) writes a 0,
		 * the result is 0. */
		result = result && bit;
	}

	return result;
}

static int w1_emul_read_byte(const struct device *dev)
{
	uint8_t value = 0;

	/* Read 8-bits, LSB first */
	for (int i = 0; i < 8; i++) {
		int ret;

		ret = w1_emul_single_bit(dev, 1);
		if (ret < 0) {
			return ret;
		}

		value |= ret << i;
	}

	return value;
}

static int w1_emul_write_byte(const struct device *dev, uint8_t value)
{
	/* Write 8-bits, LSB first */
	for (int i = 0; i < 8; i++) {
		int ret;
		int bit;

		bit = (value & (1 << i)) != 0;
		ret = w1_emul_single_bit(dev, bit);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int w1_emul_triplet(const struct device *dev, int search_dir)
{
	int bit1, bit2;
	uint8_t dir_taken;

	bit1 = w1_emul_single_bit(dev, 1);
	bit2 = w1_emul_single_bit(dev, 1);

	if (bit1 == 0 && bit2 == 0) {
		/* Collision, so take search_dir */
		dir_taken = search_dir;
	}
	else {
		dir_taken = bit1;
	}

	w1_emul_single_bit(dev, dir_taken);

	return bit1 << 2 | bit2 << 1 | dir_taken;
}

/**
 * Set up a new emulator and add it to the list
 *
 * @param dev 1-wire emulation controller device
 */
static int w1_emul_init(const struct device *dev)
{
	struct w1_emul_data *data = dev->data;
	const struct emul_list_for_bus *list = dev->config;
	int rc;

	sys_slist_init(&data->emuls);

	rc = emul_init_for_bus_from_list(dev, list);

	return rc;
}

int w1_emul_register(const struct device *dev, const char *name,
		     struct w1_emul *emul)
{
	struct w1_emul_data *data = dev->data;

	sys_slist_append(&data->emuls, &emul->node);

	LOG_INF("Register emulator '%s' at 1-wire addr %s\n", name, w1_addr_str(&emul->addr));

	return 0;
}

/* Device instantiation */

static struct w1_driver_api w1_emul_api = {
	.reset_bus = w1_emul_reset_bus,
	.read_byte = w1_emul_read_byte,
	.write_byte = w1_emul_write_byte,
	.triplet = w1_emul_triplet,
};

#define EMUL_LINK_AND_COMMA(node_id) {		\
	.label = DT_LABEL(node_id),		\
},

#define W1_EMUL_INIT(n) \
	static const struct emul_link_for_bus emuls_##n[] = { \
		DT_FOREACH_CHILD(DT_DRV_INST(n), EMUL_LINK_AND_COMMA) \
	}; \
	static struct emul_list_for_bus w1_emul_cfg_##n = { \
		.children = emuls_##n, \
		.num_children = ARRAY_SIZE(emuls_##n), \
	}; \
	static struct w1_emul_data w1_emul_data_##n; \
	DEVICE_AND_API_INIT(w1_##n, \
			    DT_INST_LABEL(n), \
			    w1_emul_init, \
			    &w1_emul_data_##n, \
			    &w1_emul_cfg_##n, \
			    POST_KERNEL, \
			    CONFIG_W1_INIT_PRIORITY, \
			    &w1_emul_api);

DT_INST_FOREACH_STATUS_OKAY(W1_EMUL_INIT)
