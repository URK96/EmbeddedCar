CP_PATH=$(abspath ../..)/CrossCompiler
CROSS_COMPILE=$(CP_PATH)/gcc-linaro-arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
ROOTFS=$(CP_PATH)/sysroots/cortexa15t2hf-vfp-neon-linux-gnueabi

ARCH=arm
CC=$(CROSS_COMPILE)gcc

INC += -I protocol
INC += -I$(ROOTFS)/include
INC += -I$(ROOTFS)/usr/include
INC += -I$(ROOTFS)/usr/include/omap
INC += -I$(ROOTFS)/usr/include/libdrm
INC += -I$(ROOTFS)/usr/include/gbm
LIBDIR := $(ROOTFS)/usr/lib

CFLAGS := -O1 -g -Wall -fPIC -mfloat-abi=hard -mfpu=neon -Wl,-rpath,$(ROOTFS)/lib -Wl,-rpath,$(ROOTFS)/usr/lib $(INC)

LDFLAGS = -lm -lpthread -L$(LIBDIR) -lrt -ldrm -lmtdev -ldrm_omap -lstdc++

TARGET = testParking

all: ${TARGET}

clean:
	rm -rf *.o *.lo $(TARGET)

${TARGET}: testParking.c util.lo car_lib.lo
	${CC} ${CFLAGS} -o $@ testParking.c util.lo car_lib.lo $(LDFLAGS)

util.lo: util.c util.h
	${CC} ${CFLAGS} -c -o $@ util.c

car_lib.lo: car_lib.c car_lib.h
	${CC} ${CFLAGS} -c -o $@ car_lib.c