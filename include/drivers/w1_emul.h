/**
 * @file
 *
 * @brief Public APIs for the 1-wire emulation drivers.
 */

/*
 * Copyright (c) 2020 Alexander Mihajlovic <a@abxy.se>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_DRIVERS_W1_EMUL_H_
#define ZEPHYR_INCLUDE_DRIVERS_W1_EMUL_H_

/**
 * @brief 1-Wire Emulation Interface
 * @defgroup w1_emul_interface 1-Wire Emulation Interface
 * @ingroup io_emulators
 * @{
 */

#include <zephyr/types.h>
#include <device.h>

#ifdef __cplusplus
extern "C" {
#endif

struct w1_emul_api;

/** Node in a linked list of emulators for 1-wire devices */
struct w1_emul {
	sys_snode_t node;

	/* API provided for this device */
	const struct w1_emul_api *api;

	/* Address of the emulated device */
	struct w1_addr addr;
};

/**
 * Generates reset/presence detect on the bus.
 * The emulator should reset its internal state and return a 1 or 0
 * to indicate its presence on the bus.
 *
 * @param emul Emulator instance
 * @param value Value written by the bus master
 *
 * @retval 0 or 1 If successful.
 * @retval -EIO General input / output error.
 */
typedef int (*w1_emul_reset_bus_t)(struct w1_emul *emul);

/**
 * Generates single bit read/write cycle on the emulator bus.
 * The device emulator returns a 0 or 1.
 *
 * @param emul Emulator instance
 * @param value Value written by the bus master
 *
 * @retval 0 or 1 If successful.
 * @retval -EIO General input / output error.
 */
typedef int (*w1_emul_single_bit_t)(struct w1_emul *emul, int value);

/**
 * Register an emulated device on the controller
 *
 * @param dev Device that will use the emulator
 * @param name User-friendly name for this emulator
 * @param emul 1-wire emulator to use
 * @return 0 indicating success (always)
 */
int w1_emul_register(const struct device *dev, const char *name,
		     struct w1_emul *emul);

/** Definition of the emulator API */
struct w1_emul_api {
	w1_emul_reset_bus_t reset_bus;
	w1_emul_single_bit_t single_bit;
};

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_DRIVERS_W1_EMUL_H_ */
