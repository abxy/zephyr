/*
 * Copyright (c) 2020 Alexander Mihajlovic <a@abxy.se>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT maxim_ds18b20

#define LOG_LEVEL CONFIG_EMUL_DS18B20_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(EMUL_DS18B20);

#include <device.h>
#include <emul.h>
#include <drivers/w1.h>
#include <drivers/w1_emul.h>
#include <sys/crc.h>

enum {
	ST_INIT = 0,
	ST_RX_BITS,
	ST_TX_BITS,
	ST_ROM_COMMAND,
	ST_SEARCH_ROM_TX,
	ST_SEARCH_ROM_TX_INV,
	ST_SEARCH_ROM_RX,
	ST_MATCH_ROM,
	ST_FUNCTION_COMMAND,
	ST_WRITE_SCRATCHPAD,
	ST_READ_POWER_SUPPLY,
};

enum {
	EMUL_SINGLE_BIT = 0,
	EMUL_RX_BYTES,
	EMUL_TX_BYTES
};


#define DS18B20_FAMILIY_CODE 0x28

#define DS18B20_CMD_CONVERT_T         0x44
#define DS18B20_CMD_COPY_SCRATCHPAD   0x48
#define DS18B20_CMD_WRITE_SCRATCHPAD  0x4E
#define DS18B20_CMD_READ_SCRATCHPAD   0xBE
#define DS18B20_CMD_RECALL_EEPROM     0xB8
#define DS18B20_CMD_READ_POWER_SUPPLY 0xB4

#define DS18B20_POWER_ON_TEMP_LSB 0x50
#define DS18B20_POWER_ON_TEMP_MSB 0x05

#define DS18B20_SCRATCHPAD_SIZE 9
#define DS18B20_CRC_POLYNOMIAL 0x31

/** Run-time data used by the emulator */
struct ds18b20_emul_data {
	/** W1 emulator detail */
	struct w1_emul emul;
	/** Configuration information */
	const struct ds18b20_emul_cfg *cfg;

	/** Presence on the bus*/
	bool present;
	/** Alarm flag */
	bool alarm_flag;
	/** Emulator state-machine state */
	int state;

	union {
		uint8_t raw[DS18B20_SCRATCHPAD_SIZE];
		struct {
			uint8_t temp_lsb;
			uint8_t temp_msb;
			uint8_t alarm_hi;
			uint8_t alarm_lo;
			uint8_t config;
			uint8_t reserved1;
			uint8_t reserved2;
			uint8_t reserved3;
			uint8_t crc;
		} __packed;
	} scratchpad;

	struct {
		uint8_t alarm_hi;
		uint8_t alarm_lo;
		uint8_t config;
	} eeprom;

	/** Alarm flag */
	bool alarm_search;

	size_t bit_count;

	int transfer_state;
	size_t transfer_length;
	uint8_t transfer_buffer[16];
};

/** Static configuration for the emulator */
struct ds18b20_emul_cfg {
	/** Name of the 1-wire bus this emulator connects to */
	const char *w1_name;
	/** Pointer to run-time data */
	struct ds18b20_emul_data *data;
	/** Serial number */
	uint64_t serial;
};

static const char *ds18b20_emul_state_str(int state)
{
	switch (state)
	{
	case ST_INIT:
		return "ST_INIT";
	case ST_RX_BITS:
		return "ST_RX_BITS";
	case ST_ROM_COMMAND:
		return "ST_ROM_COMMAND";
	case ST_FUNCTION_COMMAND:
		return "ST_FUNCTION_COMMAND";
	default:
		return "Unknown";
	}
}


static void ds18b20_emul_begin_rx(struct w1_emul *emul, size_t len)
{
	struct ds18b20_emul_data *data;
	data = CONTAINER_OF(emul, struct ds18b20_emul_data, emul);

	__ASSERT_NO_MSG(len <= sizeof(data->transfer_buffer));

	data->bit_count = 0;
	data->transfer_state = EMUL_RX_BYTES;
	data->transfer_length = len;
	memset(data->transfer_buffer, 0, sizeof(data->transfer_buffer));
}

static void ds18b20_emul_begin_tx(struct w1_emul *emul, const uint8_t *src, size_t len)
{
	struct ds18b20_emul_data *data;
	data = CONTAINER_OF(emul, struct ds18b20_emul_data, emul);

	__ASSERT_NO_MSG(len <= sizeof(data->transfer_buffer));

	data->bit_count = 0;
	data->transfer_state = EMUL_TX_BYTES;
	data->transfer_length = len;
	memcpy(data->transfer_buffer, src, len);
}

static bool ds18b20_emul_handle_rx_bit(struct w1_emul *emul, int value)
{
	struct ds18b20_emul_data *data;
	data = CONTAINER_OF(emul, struct ds18b20_emul_data, emul);

	size_t b = data->bit_count;
	data->transfer_buffer[b / 8] |= value << (b % 8);

	data->bit_count++;

	bool done = (data->transfer_length * 8 == data->bit_count);

	if (done) {
		data->transfer_state = EMUL_SINGLE_BIT;
	}

	return !done;
}

static int ds18b20_emul_handle_tx_bit(struct w1_emul *emul)
{
	struct ds18b20_emul_data *data;
	data = CONTAINER_OF(emul, struct ds18b20_emul_data, emul);

	size_t b = data->bit_count;
	int out = (data->transfer_buffer[b / 8] >> (b % 8)) & 0b1;

	data->bit_count++;

	if (data->transfer_length * 8 == data->bit_count) {
		data->transfer_state = EMUL_SINGLE_BIT;
	}

	return out;
}

static int ds18b20_emul_reset_bus(struct w1_emul *emul)
{
	struct ds18b20_emul_data *data;
	data = CONTAINER_OF(emul, struct ds18b20_emul_data, emul);

	LOG_DBG("In state %s: reset_bus", ds18b20_emul_state_str(data->state));

	ds18b20_emul_begin_rx(emul, 1);
	data->state = ST_ROM_COMMAND;

	return data->present;
}

static inline uint8_t crc8_w1(const uint8_t *src, size_t len)
{
	return crc8(src, len, DS18B20_CRC_POLYNOMIAL, 0x00, false);
}

static void ds18b20_emul_convert_temp(struct w1_emul *emul, double temp)
{
	struct ds18b20_emul_data *data;

	data = CONTAINER_OF(emul, struct ds18b20_emul_data, emul);

	uint16_t temp_u16 = temp / (1.0 / 16);

	data->scratchpad.temp_msb = (temp_u16 >> 8) & 0xFF;
	data->scratchpad.temp_lsb = (temp_u16 >> 0) & 0xFF;
}

static int ds18b20_emul_single_bit(struct w1_emul *emul, int value)
{
	struct ds18b20_emul_data *data;
	const struct ds18b20_emul_cfg *cfg;

	data = CONTAINER_OF(emul, struct ds18b20_emul_data, emul);
	cfg = data->cfg;

	/* Exit early if not present on the bus. */
	if (!data->present) {
		return 1;
	}

	if (data->transfer_state == EMUL_RX_BYTES) {
		if (ds18b20_emul_handle_rx_bit(emul, value)) {
			return 1;
		}
	}
	else if (data->transfer_state == EMUL_TX_BYTES) {
		return ds18b20_emul_handle_tx_bit(emul);
	}

	switch (data->state) {
	case ST_INIT:
		/* A bus reset is required to proceed from this state. */
		break;
	case ST_ROM_COMMAND:
		switch (data->transfer_buffer[0])
		{
			case W1_CMD_READ_ROM:
				ds18b20_emul_begin_tx(emul, (uint8_t*)&emul->addr, sizeof(emul->addr));
				data->state = ST_INIT;
				break;
			case W1_CMD_SEARCH:
				data->alarm_search = false;
				data->state = ST_SEARCH_ROM_TX;
				data->bit_count = 0;
				break;
			case W1_CMD_ALARM_SEARCH:
				data->alarm_search = true;
				data->state = ST_SEARCH_ROM_TX;
				data->bit_count = 0;
				break;
			case W1_CMD_MATCH_ROM:
				data->state = ST_MATCH_ROM;
				data->bit_count = 0;
				break;
			case W1_CMD_SKIP_ROM:
				ds18b20_emul_begin_rx(emul, 1);
				data->state = ST_FUNCTION_COMMAND;
				break;
			default:
				data->state = ST_INIT;
				break;
		}
		break;

	case ST_SEARCH_ROM_TX:
		data->state = ST_SEARCH_ROM_TX_INV;
		return ((w1_addr_to_u64(&emul->addr) >> data->bit_count) & 0b1);

	case ST_SEARCH_ROM_TX_INV:
		data->state = ST_SEARCH_ROM_RX;
		return (~(w1_addr_to_u64(&emul->addr) >> data->bit_count) & 0b1);

	case ST_SEARCH_ROM_RX:
		if (((w1_addr_to_u64(&emul->addr) >> data->bit_count) & 0b1) != value ||
		    (data->alarm_search && !data->alarm_flag)) {
			data->state = ST_INIT;
		}
		else {
			data->state = ST_SEARCH_ROM_TX;
			data->bit_count++;

			if (data->bit_count == W1_ROM_BITS) {
				data->state = ST_INIT;
			}
		}
		break;

	case ST_MATCH_ROM:
		if (((w1_addr_to_u64(&emul->addr) >> data->bit_count) & 0b1) != value) {
			data->state = ST_INIT;
		}
		else {
			data->bit_count++;

			if (data->bit_count == 64) {
				ds18b20_emul_begin_rx(emul, 1);
				data->state = ST_FUNCTION_COMMAND;
			}
		}
		break;

	case ST_FUNCTION_COMMAND:
		switch (data->transfer_buffer[0]) {
			case DS18B20_CMD_CONVERT_T:
				ds18b20_emul_convert_temp(data, 20.0f + cfg->serial);
				data->state = ST_INIT;
				break;

			case DS18B20_CMD_COPY_SCRATCHPAD:
				data->eeprom.alarm_hi = data->scratchpad.alarm_hi;
				data->eeprom.alarm_lo = data->scratchpad.alarm_lo;
				data->eeprom.config = data->scratchpad.config;
				data->state = ST_INIT;
				break;

			case DS18B20_CMD_WRITE_SCRATCHPAD:
				ds18b20_emul_begin_rx(emul, DS18B20_SCRATCHPAD_SIZE);
				data->state = ST_WRITE_SCRATCHPAD;
				break;

			case DS18B20_CMD_READ_SCRATCHPAD:
				data->scratchpad.crc = crc8_w1(data->scratchpad.raw, DS18B20_SCRATCHPAD_SIZE - 1);
				ds18b20_emul_begin_tx(emul, data->scratchpad.raw, DS18B20_SCRATCHPAD_SIZE);
				data->state = ST_INIT;
				break;

			case DS18B20_CMD_RECALL_EEPROM:
				data->scratchpad.alarm_hi = data->eeprom.alarm_hi;
				data->scratchpad.alarm_lo = data->eeprom.alarm_lo;
				data->scratchpad.config = data->eeprom.config;
				data->state = ST_INIT;
				break;

			case DS18B20_CMD_READ_POWER_SUPPLY:
				data->state = ST_READ_POWER_SUPPLY;
				break;

			default:
				data->state = ST_INIT;
				break;
		}
		break;

	case ST_WRITE_SCRATCHPAD:
		data->scratchpad.alarm_hi = data->transfer_buffer[0];
		data->scratchpad.alarm_lo = data->transfer_buffer[1];
		data->scratchpad.config = data->transfer_buffer[2];
		data->state = ST_INIT;
		break;

	case ST_READ_POWER_SUPPLY:
		data->state = ST_INIT;
		return 0;
		break;

	default:
		break;
	}

	return 1;
}


/* Device instantiation */

static struct w1_emul_api ds18b20_emul_api = {
	.reset_bus = ds18b20_emul_reset_bus,
	.single_bit = ds18b20_emul_single_bit,
};

/**
 * Set up a new DS18B20 emulator
 *
 * This should be called for each DS18B20 sensor that needs to be emulated.
 * It registers it with the 1-wire emulation controller.
 *
 * @param emul Emulation information
 * @param parent Device to emulate (must use DS18B20 driver)
 * @return 0 indicating success (always)
 */
static int emul_ds18b20_init(const struct emul *emul,
			     const struct device *parent)
{
	const struct ds18b20_emul_cfg *cfg = emul->cfg;
	struct ds18b20_emul_data *data = cfg->data;

	__ASSERT_NO_MSG(cfg->serial < 0xFFFFFFFFFFFFULL);

	data->cfg = cfg;

	data->emul.api = &ds18b20_emul_api;
	data->emul.addr.family = DS18B20_FAMILIY_CODE;
	for (int i = 0; i < 6; i++) {
		data->emul.addr.serial[i] = (cfg->serial >> (i * 8)) & 0xFF;
	}
	data->emul.addr.crc = crc8_w1(data->emul.addr.raw, sizeof(data->emul.addr.raw) - 1);

	data->present = 1;

	data->scratchpad.temp_msb = DS18B20_POWER_ON_TEMP_MSB;
	data->scratchpad.temp_lsb = DS18B20_POWER_ON_TEMP_LSB;
	data->scratchpad.alarm_hi = data->eeprom.alarm_hi;
	data->scratchpad.alarm_lo = data->eeprom.alarm_lo;
	data->scratchpad.config = data->eeprom.config;
	data->scratchpad.reserved1 = 0xFF; /* suggested by datasheet */
	data->scratchpad.reserved3 = 0x10; /* suggested by datasheet */

	int rc = w1_emul_register(parent, emul->dev_label, &data->emul);

	return rc;
}

#define DS18B20_EMUL(n) \
	static struct ds18b20_emul_data ds18b20_emul_data_##n; \
	static const struct ds18b20_emul_cfg ds18b20_emul_cfg_##n = { \
		.w1_name = DT_INST_BUS_LABEL(n), \
		.data = &ds18b20_emul_data_##n, \
		.serial = (n), \
	}; \
	EMUL_DEFINE(emul_ds18b20_init, DT_DRV_INST(n), &ds18b20_emul_cfg_##n)

DT_INST_FOREACH_STATUS_OKAY(DS18B20_EMUL)
