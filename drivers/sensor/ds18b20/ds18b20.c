/* Driver for Maxim DS18B20 1-wire temperature sensor */

/*
 * Copyright (c) 2020 Alexander Mihajlovic <a@abxy.se>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT maxim_ds18b20

#include <drivers/sensor.h>
#include <drivers/w1.h>
#include <logging/log.h>

#include "ds18b20.h"

LOG_MODULE_REGISTER(DS18B20, CONFIG_SENSOR_LOG_LEVEL);

struct ds18b20_data {
	/* 1-wire bus device */
	const struct device *w1;

	int16_t temp;

	struct w1_addr addr;
};

struct ds18b20_config {
	char *w1_name;
};

int ds18b20_set_addr(const struct device *dev, const struct w1_addr *addr)
{
	struct ds18b20_data *data = dev->data;

	if (addr == NULL) {
		data->addr.family = 0;
	}
	else {
		if (addr->family != DS18B20_FAMILIY_CODE) {
			return -EINVAL;
		}

		data->addr = *addr;
	}

	return 0;
}

static int ds18b20_select(const struct device *dev)
{
	struct ds18b20_data *data = dev->data;
	const struct w1_addr *addr = &data->addr;
	const struct device *w1 = data->w1;
	int ret;

	ret = w1_reset_bus(w1);
	if (ret < 0) {
		return ret;
	}
	else if (ret == 0) {
		return -ENODEV;
	}

	if (addr->family == 0) {
		return w1_write_byte(w1, W1_CMD_SKIP_ROM);
	}
	else {
		w1_write_byte(w1, W1_CMD_MATCH_ROM);
		w1_write_byte(w1, addr->family);

		for (int i = 0; i < sizeof(addr->serial); i++) {
			w1_write_byte(w1, addr->serial[i]);
		}

		w1_write_byte(w1, addr->crc);

		return 0;
	}
}

static inline k_timeout_t ds18b20_temp_conversion_time(const struct device *dev)
{
	return K_MSEC(750);
}

static int ds18b20_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct ds18b20_data *data = dev->data;
	const struct device *w1 = data->w1;
	int ret;

	ret = ds18b20_select(dev);
	if (ret) {
		return ret;
	}

	w1_write_byte(w1, DS18B20_CMD_CONVERT_T);

	k_sleep(ds18b20_temp_conversion_time(dev));

	ret = ds18b20_select(dev);
	if (ret) {
		return ret;
	}

	w1_write_byte(w1, DS18B20_CMD_READ_SCRATCHPAD);

	uint8_t temp_l = w1_read_byte(w1);
	uint8_t temp_h = w1_read_byte(w1);

	data->temp = (int16_t)((temp_h << 8) + temp_l);

	return 0;
}

static int ds18b20_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct ds18b20_data *data = dev->data;
	int32_t value;

	if (chan != SENSOR_CHAN_AMBIENT_TEMP) {
		return -ENOTSUP;
	}

	value = data->temp * DS18B20_TEMP_SCALE;
	val->val1 = value / 1000000;
	val->val2 = value % 1000000;

	return 0;
}


static int ds18b20_init(const struct device *dev)
{
	struct ds18b20_data *data = dev->data;
	const struct ds18b20_config *cfg = dev->config;

	data->w1 = device_get_binding(cfg->w1_name);
	if (data->w1 == NULL) {
		LOG_ERR("Failed to get pointer to %s device!", cfg->w1_name);
	}

	return 0;
}

static const struct sensor_driver_api ds18b20_driver_api = {
	.sample_fetch = ds18b20_sample_fetch,
	.channel_get = ds18b20_channel_get,
};

#define DS18B20_INIT(n)                                                 \
	static const struct ds18b20_config ds18b20_config_##n = {       \
		.w1_name = DT_INST_BUS_LABEL(n),                        \
	};                                                              \
								        \
	static struct ds18b20_data ds18b20_data_##n;                    \
								        \
	DEVICE_AND_API_INIT(ds18b20_##n, DT_INST_LABEL(n),              \
		    ds18b20_init,                                       \
		    &ds18b20_data_##n, &ds18b20_config_##n,             \
		    POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,           \
		    &ds18b20_driver_api);                               \

DT_INST_FOREACH_STATUS_OKAY(DS18B20_INIT)