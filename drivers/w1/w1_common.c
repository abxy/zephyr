/*
 * Copyright (c) 2020 Alexander Mihajlovic <a@abxy.se>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <stdio.h>
#include <drivers/w1.h>

#define LOG_LEVEL CONFIG_W1_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(w1);

#define W1_PRIMARY_SEARCH_DIR (0)

const char *w1_addr_str(struct w1_addr *addr)
{
	static char buf[sizeof("00.112233445566.00")];

	snprintf(buf, sizeof(buf), "%02X.%02X%02X%02X%02X%02X%02X.%02X",
		addr->family, addr->serial[0], addr->serial[1], addr->serial[2],
		addr->serial[3], addr->serial[4], addr->serial[5], addr->crc);

	return buf;
}

int w1_read_addr(const struct device *dev, struct w1_addr *addr)
{
	int rc;

	rc = w1_reset_bus(dev);
	if (rc < 0) {
		return -EIO;
	}
	else if (rc == 0) {
		return -ENODEV;
	}

	rc = w1_write_byte(dev, W1_CMD_READ_ROM);
	if (rc) {
		return -EIO;
	}

	for (int i = 0; i < sizeof(addr->raw); i++)
	{
		rc = w1_read_byte(dev);
		if (rc < 0) {
			return -EIO;
		}
		addr->raw[i] = rc;
	}

	return 0;
}

void w1_search_init(struct w1_search *state, const struct device *dev, uint8_t search_command)
{
	state->search_command = search_command;
	state->dev = dev;
	state->last_collision = -1;
}

int w1_search_next(struct w1_search *state, struct w1_addr *addr)
{
	const struct device *dev = state->dev;
	int rc;

	rc = w1_reset_bus(dev);
	if (rc < 0) {
		return -EIO;
	}
	else if (rc == 0) {
		if (state->last_collision == -1) {
			/* No device present at the beginning of a search is not an error,
			 * it just means that we are done (empty result). */
			return 0;
		}
		else {
			/* Devices that were here before have unexpectedly disappeared. */
			return -ENODEV;
		}
	}

	w1_write_byte(dev, state->search_command);
	if (rc < 0) {
		return -EIO;
	}

	uint64_t prev_addr_bits = w1_addr_to_u64(addr);
	uint64_t addr_bits = 0;
	int8_t last_collision = -1;

	/*
	 * Do a depth first search to the next address on the bus.
	 *
	 * First,
	 *
	 */
	for (uint8_t i = 0; i < 64; i++) {
		int search_dir = W1_PRIMARY_SEARCH_DIR;

		if (i < state->last_collision) {
			search_dir = (prev_addr_bits >> i) & 0b1;
		}
		else if (i == state->last_collision) {
			search_dir = !search_dir;
		}

		rc = w1_triplet(dev, search_dir);
		if (rc < 0) {
			LOG_ERR("Triplet error (%d) at bit position %d", rc, i);
			return rc; /* -EIO */
		}

		uint32_t triplet = (uint32_t)rc;
		uint32_t dir_taken = triplet & W1_TRIPLET_DIR_TAKEN;

		/* Keep the position of the last collision which we should revisit.
		 * We only need to revisit a collision after taking the primary
		 * search direction. If we didn't take the primary search direction
		 * this time, we should have taken it in a previous step. */
		if (W1_TRIPLET_COLLISION(triplet) && (dir_taken == W1_PRIMARY_SEARCH_DIR)) {
			last_collision = i;
		}

		addr_bits |= (uint64_t)dir_taken << i;
	}

	state->last_collision = last_collision;
	addr->le64 = sys_cpu_to_le64(addr_bits);

	if (last_collision == -1) {
		return 0; /* Done */
	}
	else {
		return 1;
	}
}
