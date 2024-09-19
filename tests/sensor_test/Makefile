cur_dir := $(dir $(lastword $(MAKEFILE_LIST)))

obj-y := $(patsubst $(cur_dir)%,%,$(wildcard $(cur_dir)*.c $(cur_dir)*.cpp $(cur_dir)*.S))
obj-y := $(obj-y:.c=.o)
obj-y := $(obj-y:.cpp=.o)
obj-y := $(obj-y:.S=.o)

obj-y += ../../platform/drivers/sensor/

ccflags-y +=-Iplatform/drivers/sensor/bosch \
	-Iplatform/drivers/sensor/st \
	-Iplatform/drivers/sensor/senodia \

ifeq ($(SENSOR_TEST),1)
ccflags-y += -DSENSOR_TEST
endif

ifeq ($(CHIP_HAS_SPI),1)
ccflags-y += -DCHIP_HAS_SPI
endif

ifeq ($(CHIP_HAS_SPILCD),1)
ccflags-y += -DCHIP_HAS_SPILCD
endif
