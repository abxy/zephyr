# SPDX-License-Identifier: Apache-2.0

# Once we have more than 10 devices we should consider splitting them into
# subdirectories to match the drivers/ structure.
zephyr_library_sources_ifdef(CONFIG_EMUL_DS18B20	emul_ds18b20.c)
