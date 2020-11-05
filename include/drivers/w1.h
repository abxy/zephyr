/*
 * Copyright (c) 2020 Alexander Mihajlovic <a@abxy.se>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public API for 1-wire drivers.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_W1_H_
#define ZEPHYR_INCLUDE_DRIVERS_W1_H_

#include <device.h>
#include <sys/byteorder.h>

#ifdef __cplusplus
extern "C" {
#endif

#define W1_CMD_SEARCH       (0xF0U)
#define W1_CMD_ALARM_SEARCH (0xECU)
#define W1_CMD_SKIP_ROM     (0xCCU)
#define W1_CMD_MATCH_ROM    (0x55U)
#define W1_CMD_READ_ROM     (0x33U)

#define W1_FAMILY_CODE_BITS 8
#define W1_SERIAL_NO_BITS 48
#define W1_CRC_BITS 8
#define W1_ROM_BITS (W1_FAMILY_CODE_BITS + W1_SERIAL_NO_BITS + W1_CRC_BITS)

/*

API questions:

Does it make sense to return -ENODEV, e.g. in w1_read_addr(),
when w1_reset_bus() returns 0 (no device present)?

If yes, should w1_reset_bus() itself return -ENODEV instead of 0 in that case?

Likewise, should w1_triplet() return the raw bits of the triplet,
leaving it up to the caller to identify the "no device" & collision cases?
*/


/*
 * The following #defines are used with w1_search_init().
 */

/** Search all peripherals */
#define W1_SEARCH_ALL   (W1_CMD_SEARCH)

/** Search peripherals that have the alarm flag set. */
#define W1_SEARCH_ALARM (W1_CMD_ALARM_SEARCH)

/*
 * The following #defines are used to decode the result of w1_triplet().
 */

/* */
#define W1_TRIPLET_DIR_TAKEN_POS (0)
#define W1_TRIPLET_DIR_TAKEN     (1 << W1_TRIPLET_DIR_TAKEN_POS)

#define W1_TRIPLET_BIT1_POS      (1)
#define W1_TRIPLET_BIT1          (1 << W1_TRIPLET_BIT1_POS)

#define W1_TRIPLET_BIT2_POS      (2)
#define W1_TRIPLET_BIT2          (1 << W1_TRIPLET_BIT2_POS)

/* */
#define W1_TRIPLET_BIT1_BIT2     (W1_TRIPLET_BIT1 | W1_TRIPLET_BIT2)

#define W1_TRIPLET_COLLISION(rc) ((rc & W1_TRIPLET_BIT1_BIT2) == 0)

/**
 *  @brief 1-wire address structure
 *
 *  Used to hold the address for a 1-wire peripheral
 */
struct w1_addr {
	union {
		struct {
			uint8_t family;
			uint8_t serial[6];
			uint8_t crc;
		} __packed;
		uint8_t raw[8];
		uint64_t le64;
	};
};

struct w1_search {
	/* 1-wire bus device whose which is being searched for peripherals. */
	const struct device *dev;

	/* Bit index of the last collision in the previous search step. */
	int8_t last_collision;

	/* The search command which is being used (normal or alarm search)*/
	uint8_t search_command;
};

typedef int (*w1_api_reset_bus_t)(const struct device *dev);
typedef int (*w1_api_read_byte_t)(const struct device *dev);
typedef int (*w1_api_write_byte_t)(const struct device *dev, uint8_t value);
typedef int (*w1_api_triplet_t)(const struct device *dev, int search_dir);

__subsystem struct w1_driver_api {
	w1_api_reset_bus_t reset_bus;
	w1_api_read_byte_t read_byte;
	w1_api_write_byte_t write_byte;
	w1_api_triplet_t triplet;
};

/**
 * @brief Perform a 1-wire reset and presence detect cycle.
 *
 * @param dev Pointer to the 1-wire bus device driver struct.
 *
 * @retval 0 or 1 indicating if a device presence pulse was detected.
 * @retval -EIO General input / output error.
 */
__syscall int w1_reset_bus(const struct device *dev);

static inline int z_impl_w1_reset_bus(const struct device *dev)
{
	const struct w1_driver_api *api =
		(const struct w1_driver_api *)dev->api;

	return api->reset_bus(dev);
}

/**
 * @brief Perform a single byte read.
 *
 * @param dev Pointer to the 1-wire bus device driver struct.
 *
 * @retval 0-255 If successful
 * @retval -EIO General input / output error.
 */
__syscall int w1_read_byte(const struct device *dev);

static inline int z_impl_w1_read_byte(const struct device *dev)
{
	const struct w1_driver_api *api =
		(const struct w1_driver_api *)dev->api;

	return api->read_byte(dev);
}

/**
 * @brief Perform a single byte write.
 *
 * @param dev Pointer to the 1-wire bus device driver struct.
 * @param value Byte value to write.
 *
 * @retval 0 If successful
 * @retval -EIO General input / output error.
 */
__syscall int w1_write_byte(const struct device *dev, uint8_t value);

static inline int z_impl_w1_write_byte(const struct device *dev, uint8_t value)
{
	const struct w1_driver_api *api =
		(const struct w1_driver_api *)dev->api;

	return api->write_byte(dev, value);
}

/**
 * @brief Perform a 1-wire "triplet" operation on the bus.
 *
 * The triplet operation consists of two read cycles followed
 * by one write cycle.
 *
 * @param dev Pointer to the 1-wire bus device driver struct.
 * @param search_dir Search direction to use if
 *
 * @retval 0 or 1
 * @retval W1_TRIPLET_COLLISION
 * @retval -EIO General input / output error.
 */
__syscall int w1_triplet(const struct device *dev, int search_dir);

static inline int z_impl_w1_triplet(const struct device *dev, int search_dir)
{
	const struct w1_driver_api *api =
		(const struct w1_driver_api *)dev->api;

	return api->triplet(dev, search_dir);
}

/**
 * @brief Formats a 1-wire address as a string.
 *
 * The returned string may be modified by a subsequent
 * call to w1_addr_str().
 *
 * @param addr Pointer to the w1_addr structure.
 *
 * @retval A string representation of the given 1-wire address.
 */
const char *w1_addr_str(struct w1_addr *addr);


/**
 * @brief Reads the address of the single device on the 1-wire bus.
 *
 * Performs a complete Read ROM operation, including bus reset,
 * and reads the result into a the given address structure.
 *
 * If more than 1 device can be present on the bus, this operation
 * will not work. See w1_search_start() for a solution in that scenario.
 *
 * @param dev Pointer to the 1-wire bus device driver struct.
 * @param addr Pointer to the w1_addr structure to populate.
 *
 * @retval 0 If successful.
 * @retval -ENODEV If no presence pulse is detected on the bus.
 * @retval -EIO General input / output error.
 */
int w1_read_addr(const struct device *dev, struct w1_addr *addr);

/**
 * @brief Converts a 1-wire address to a 64-bit unsigned integer.
 *
 * @param addr Pointer to a w1_addr structure.
 *
 * @retval Unisigned integer interpretation of the given address.
 *
 */
static inline uint64_t w1_addr_to_u64(struct w1_addr *addr)
{
	return sys_le64_to_cpu(addr->le64);
}

/**
 * @brief Initialize a w1_search structure to
 *
 * @param state Pointer to a w1_search structure.
 * @param dev Pointer to the 1-wire bus device
 * @param dev Pointer to the 1-wire bus device (See )
 *
 * @retval Unisigned integer interpretation of the given address.
 *
 */
void w1_search_init(struct w1_search *search, const struct device *dev, uint8_t search_command);

/**
 * @brief Converts a 1-wire address to a 64-bit unsigned integer.
 *
 * @param search Pointer to a w1_addr structure.
 * @param addr Pointer to a w1_addr structure.
 *
 * @retval 0 If the search finished
 * @retval 1
 * @retval -ENODEV If no device responds to the search command
 * @retval -EIO General input / output error.
 */
int w1_search_next(struct w1_search *search, struct w1_addr *addr);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#include <syscalls/w1.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_W1_H_ */