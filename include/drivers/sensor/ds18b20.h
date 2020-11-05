/*
 * Copyright (c) 2020 Alexander Mihajlovic <a@abxy.se>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Extended public API for DS18B20 1-wire temperature sensor
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_DS18B20_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_DS18B20_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <device.h>
#include <drivers/sensor.h>
#include <drivers/w1.h>

/** Family code of DS18B20 temperature sensors. */
#define DS18B20_FAMILIY_CODE 0x28

/**
 * @brief Fetch operating mode and version information.
 *
 * @param dev Pointer to the sensor device
 *
 * @param ptr Pointer to where the returned information should be stored
 *
 * @return 0 on success, or a negative errno code on failure.
 */
int ds18b20_set_addr(const struct device *dev, const struct w1_addr *addr);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_DS18B20_H_ */
