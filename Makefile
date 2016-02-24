obj-m	+= menable.o

.PHONY: clean install archive

SOURCE_DIR = $(shell pwd)
BUILD_DIR = /lib/modules/$(shell uname -r)/build

INSTALL = /usr/bin/install
DEPMOD = /sbin/depmod

#menable-objs := uiq.o menable_design.o menable_dma.o menable_core.o menable3.o fgrab.o menable4.o menable5.o menable_mem.o menable_ioctl.o linux_version.o
menable-objs := uiq.o menable_design.o menable_dma.o menable_core.o menable3.o fgrab.o menable4.o  menable_mem.o menable_ioctl.o linux_version.o

all: menable.ko

install: menable.ko udev/10-siso.rules udev/men_path_id udev/men_uiq
	$(MAKE) -C $(BUILD_DIR) M=$(SOURCE_DIR) modules_install
	$(DEPMOD)
	$(INSTALL) udev/10-siso.rules /etc/udev/rules.d/
	$(INSTALL) udev/men_path_id udev/men_uiq /sbin/

menable.ko: *.c *.h men_ioctl_codes.h
	$(MAKE) -C $(BUILD_DIR) M=$(SOURCE_DIR) modules

archive: men_ioctl_codes.h
	make clean
	tar cjf ../menable.tar.gz *

men_ioctl_codes.h:
	cp ../men_ioctl_codes.h .

clean:
	$(MAKE) -C $(BUILD_DIR) M=$(SOURCE_DIR) clean
