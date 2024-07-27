PLATFORMSRC_CONTRIB := ${CHIBIOS}/os/hal/ports/common/ARMCMx/nvic.c \
                       ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/MIMXRT1062/hal_lld.c \
                       ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/MIMXRT1062/clock_config.c \
                       ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/MIMXRT1062/printf_debug.c \
                       ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/MIMXRT1062/bootable_image.c \
                       ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/MIMXRT1062/mpu.c

PLATFORMINC_CONTRIB := ${CHIBIOS}/os/hal/ports/common/ARMCMx \
                       ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/LLD \
                       ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/MIMXRT1062 \
                       ${CHIBIOS_CONTRIB}/ext/mcux-sdk/devices/MIMXRT1062/drivers

ifeq ($(USE_SMART_BUILD),yes)

# Configuration files directory
ifeq ($(CONFDIR),)
  CONFDIR = .
endif

HALCONF := $(strip $(shell cat $(CONFDIR)/halconf.h $(CONFDIR)/halconf_community.h | egrep -e "\#define"))

endif

include ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/LLD/GPIOv1/driver.mk
include ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/LLD/UARTv1/driver.mk
include ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/LLD/PITv1/driver.mk
include ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/LLD/USBHSv1/driver.mk

# Shared variables
ALLCSRC += $(PLATFORMSRC_CONTRIB)
ALLINC  += $(PLATFORMINC_CONTRIB)

MIMXRT1062_DEFS = -DCPU_MIMXRT1062DVL6A -DXIP_EXTERNAL_FLASH=1

# The ChibiOS build system uses DDEFS:
DDEFS += ${MIMXRT1062_DEFS}
# The QMK ChibiOS build rules use OPT_DEFS:
OPT_DEFS += ${MIMXRT1062_DEFS}
