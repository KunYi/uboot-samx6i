#
# (C) Copyright 2016 Ubiqconn Technology Inc.
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y  := samx6.o

extra-$(CONFIG_USE_PLUGIN) :=  plugin.bin
$(obj)/plugin.bin: $(obj)/plugin.o
	$(OBJCOPY) -O binary --gap-fill 0xff $< $@
