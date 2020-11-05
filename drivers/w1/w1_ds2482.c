
#include <device.h>
#include <drivers/i2c.h>

#define DS2482_CMD_DEVICE_RESET 0xF0
#define DS2482_CMD_SET_READ_PTR 0xE1
#define DS2482_CMD_WRITE_CONFIG 0xD2

#define DS2482_CMD_1W_RESET 0xB4
#define DS2482_CMD_1W_SINGLE_BIT 0x87
#define DS2482_CMD_1W_TRIPLET 0x78
#define DS2482_CMD_1W_WRITE_BYTE 0xA5
#define DS2482_CMD_1W_READ_BYTE 0x96

#define DS2482_PTR_CODE_STATUS 0xF0
#define DS2482_PTR_CODE_DATA   0xE1
#define DS2482_PTR_CODE_CONFIG 0xC3

#define DS2482_REG_STATUS_1WB 0x01
#define DS2482_REG_STATUS_PPD 0x02
#define DS2482_REG_STATUS_SD  0x04
#define DS2482_REG_STATUS_LL  0x08
#define DS2482_REG_STATUS_RST 0x10
#define DS2482_REG_STATUS_SBR 0x20
#define DS2482_REG_STATUS_TSB 0x40
#define DS2482_REG_STATUS_DIR 0x80

#define DS2482_WAIT_IDLE_MAX_RETRIES 100


struct ds2482_config {
	const char *bus_name;
	const uint16_t i2c_addr;
};

struct ds2482_data {
	// FIXME: const struct device *dev;
	const struct device *i2c;
};

int ds2482_set_read_ptr(struct ds2482_data *dev, u8_t read_ptr)
{
    if (dev->read_ptr != read_ptr) {
        u8_t cmd[2] = {DS2482_CMD_SET_READ_PTR, read_ptr};
        if (i2c_write(dev->i2c_device, cmd, sizeof(cmd), dev->i2c_addr)) {
            return -1;
        }
        dev->read_ptr = read_ptr;
    }

    return 0;
}

int ds2482_send_cmd(struct ds2482_data *dev, u8_t cmd)
{
    if (i2c_write(dev->i2c_device, &cmd, sizeof(cmd), dev->i2c_addr)) {
        return -1;
    }

    dev->read_ptr = DS2482_PTR_CODE_STATUS;

    return 0;
}

int ds2482_send_cmd_param(struct ds2482_data *dev, u8_t cmd, u8_t param)
{
    u8_t buf[] = {cmd, param};
    if (i2c_write(dev->i2c_device, buf, sizeof(buf), dev->i2c_addr)) {
        return -1;
    }

    dev->read_ptr = cmd == DS2482_CMD_WRITE_CONFIG ?
        DS2482_PTR_CODE_CONFIG : DS2482_PTR_CODE_STATUS;

    return 0;
}

void ds2482_w1_write_byte(const struct device *dev, u8_t value)
{
    ds2482_wait_1w_idle(dev);
    ds2482_send_cmd_param(dev, DS2482_CMD_1W_WRITE_BYTE, value);
}

int ds2482_w1_read_byte(struct ds2482_data *dev)
{

    ds2482_wait_1w_idle(dev);
    ds2482_send_cmd(dev, DS2482_CMD_1W_READ_BYTE);
    ds2482_wait_1w_idle(dev);
    ds2482_set_read_ptr(dev, DS2482_PTR_CODE_DATA);

    u8_t value = 0;
    i2c_read(dev->i2c_device, &value, sizeof(value), dev->i2c_addr);

    return value;
}

int ds2482_w1_reset_bus(struct ds2482_data *dev) {
	ds2482_send_cmd(dev, DS2482_CMD_1W_RESET);
	u8_t status = ds2482_wait_1w_idle(dev);

	if ((status & DS2482_REG_STATUS_PPD) == 0) {
		return -1;
	}

	return 0;
}


static int ds2482_init(const struct device *dev)
{
	struct ds2482_data *data = dev->data;
	const struct ds2482_config *cfg = dev->config;
	const struct device *i2c = device_get_binding(cfg->bus_name);

	if (i2c == NULL) {
		LOG_DBG("Failed to get pointer to %s device!",
			cfg->bus_name);
		return -EINVAL;
	}
	data->i2c = i2c;

	if (!cfg->base_address) {
		LOG_DBG("No I2C address");
		return -EINVAL;
	}
	data->dev = dev;
}

static int ds2482_write_configuration(struct device *dev, uint8_t value)
{
	struct ds2482_data *data = dev->data;
	const struct ds2482_config *cfg = dev->config;

	/* Set upper nibble to ones' complement of lower nibble,
	   as required by write config operation. */
	uint8_t write_value = value | (~value << 4);
	int ret = i2c_write(data->i2c, &write_value, 1, cfg->i2c_addr);

	if (ret) {
		return ret;
	}
}

static int ds2482_wait_1w_idle(struct device *dev)
{
	struct ds2482_data *data = dev->data;
	const struct ds2482_config *cfg = dev->config;

	int retries = 0;

	if (ds2482_set_read_ptr(dev, DS2482_PTR_CODE_STATUS)) {
		return -EIO;
	}

	do {
		uint8_t status;

		if (i2c_read(data->i2c, &status, sizeof(status), cfg->i2c_addr)) {
			return -EIO;
		}

		if ((status & DS2482_REG_STATUS_1WB) == 0) {
			return status;
		}
	} while (++retries < DS2482_WAIT_IDLE_MAX_RETRIES);

	LOG_DBG("ds2482_wait_1w_idle timed out");

	return -EAGAIN;
}


int ds2482_reset_bus(const struct device *dev) {
	ds2482_send_cmd(dev, DS2482_CMD_1W_RESET);
	int status = ds2482_wait_1w_idle(dev);

	if ((status & DS2482_REG_STATUS_PPD) == 0) {
		return -1;
	}

	return 0;
}

int ds2482_read_byte(const struct device *dev) {
	ds2482_send_cmd(dev, DS2482_CMD_1W_RESET);
	u8_t status = ds2482_wait_1w_idle(dev);

	if ((status & DS2482_REG_STATUS_PPD) == 0) {
	return -1;
	}

	return 0;
}

int ds2482_write_byte(const struct device *dev) {
	ds2482_send_cmd(dev, DS2482_CMD_1W_RESET);
	u8_t status = ds2482_wait_1w_idle(dev);

	if ((status & DS2482_REG_STATUS_PPD) == 0) {
	return -1;
	}

	return 0;
}

static const struct w1_driver_api w1_driver_api = {
	.reset_bus = ds2482_reset_bus,
	.read_byte = ds2482_reset_bus,
	.write_byte = ds2482_reset_bus,
};

struct ds2482_data ds2482_0_driver;

static const struct ds2482_config ds2482_0_cfg = {
	.bus_name = DT_INST_BUS_LABEL(0),
	.i2c_addr = DT_INST_REG_ADDR(0),
};

DEVICE_AND_API_INIT(ds2482_0, DT_INST_LABEL(0),
			ds2482_init, &ds2482_0_driver, &ds2482_0_cfg,
			POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
			&sht3xd_driver_api);
