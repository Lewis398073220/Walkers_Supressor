SENSOR_DRV_TYPE ?= all
ifeq ($(SENSOR_DRV_TYPE),all)
obj-y := st/ bosch/ senodia/
else ifeq ($(SENSOR_DRV_TYPE), st)
obj-y := st/
else ifeq ($(SENSOR_DRV_TYPE), bosch)
obj-y := bosch/
else ifeq ($(SENSOR_DRV_TYPE), senodia)
obj-y := senodia/
else
$(error invalid SENSOR_DRV_TYPE=$(SENSOR_DRV_TYPE))
endif

