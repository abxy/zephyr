/*
 * Copyright (c) 2020 Alexander Mihajlovic <a@abxy.se>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_DS18B20_DS18B20_H_
#define ZEPHYR_DRIVERS_SENSOR_DS18B20_DS18B20_H_

#include <drivers/sensor/ds18b20.h>

#define DS18B20_CMD_CONVERT_T         0x44
#define DS18B20_CMD_COPY_SCRATCHPAD   0x48
#define DS18B20_CMD_WRITE_SCRATCHPAD  0x4E
#define DS18B20_CMD_READ_SCRATCHPAD   0xBE
#define DS18B20_CMD_RECALL_EEPROM     0xB8
#define DS18B20_CMD_READ_POWER_SUPPLY 0xB4

#define DS18B20_SCRATCHPAD_SIZE 9

/* Scale in micro degrees Celsius */
#define DS18B20_TEMP_SCALE 62500

#endif /* ZEPHYR_DRIVERS_SENSOR_DS18B20_DS18B20_H_ */