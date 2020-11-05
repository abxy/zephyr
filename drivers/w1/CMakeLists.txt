# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(w1_common.c)

zephyr_library_sources_ifdef(CONFIG_W1_DS2482		w1_ds2482.c)
zephyr_library_sources_ifdef(CONFIG_W1_EMUL		w1_emul.c)
