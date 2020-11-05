/*
 * Copyright (c) 2020 Alexander Mihajlovic <a@abxy.se>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <drivers/w1.h>
#include <drivers/sensor/ds18b20.h>
#include <sys/printk.h>
#include <stdio.h>

#define W1_BUS_NAME "1WIRE_0"

void main(void)
{
	const struct device *w1, *dev;
	struct sensor_value temp;
	int ret;

	w1 = device_get_binding(W1_BUS_NAME);
	if (w1 == NULL)
	{
		printk("Dev not found: %s\n", W1_BUS_NAME);
		return;
	}

	dev = device_get_binding("DS18B20");
	if (dev == NULL)
	{
		printk("Dev not found: %s\n", "DS18B20");
		return;
	}

	struct w1_search search;
	struct w1_addr addr;
	int peripheral_count;

	while (1) {
		printk("Searching...\n\n");

		w1_search_init(&search, w1, W1_SEARCH_ALL);
		peripheral_count = 0;

		do
		{
			ret = w1_search_next(&search, &addr);
			if (ret < 0) {
				printk("1-wire search error: %d", ret);
				break;
			}

			peripheral_count++;

			if (addr.family == DS18B20_FAMILIY_CODE) {
				printk("Address: %s\n", w1_addr_str(&addr));

				ds18b20_set_addr(dev, &addr);
				sensor_sample_fetch(dev);
				sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
				printk("Temperature: %d.%06d °C\n\n", temp.val1, temp.val2);
			}
		} while (ret != 0);

		printk("\nFound %d peripherals on %s!\n--\n\n", peripheral_count, W1_BUS_NAME);

		k_sleep(K_MSEC(5000));
	}


	printk("Done!\n");
}

