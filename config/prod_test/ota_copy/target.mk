export SINGLE_WIRE_DOWNLOAD ?= 0
export DIRECT_TO_SINGLE_DLD ?= 0
export OTA_BIN_COMPRESSED ?= 0
include config/programmer_inflash/target.mk

export APP_IMAGE_FLASH_OFFSET ?= 0x10000
KBUILD_CPPFLAGS += -D__APP_IMAGE_FLASH_OFFSET__=$(APP_IMAGE_FLASH_OFFSET) 

export APP_ENTRY_ADDRESS ?= 0x3c010000
KBUILD_CPPFLAGS += -DAPP_ENTRY_ADDRESS=$(APP_ENTRY_ADDRESS) 

core-y += tests/programmer_ext/ota_copy/
