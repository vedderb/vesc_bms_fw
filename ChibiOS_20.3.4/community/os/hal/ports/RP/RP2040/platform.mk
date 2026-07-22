include ${CHIBIOS}/os/hal/ports/RP/RP2040/platform.mk

ifeq ($(USE_SMART_BUILD),yes)

# Configuration files directory
ifeq ($(CONFDIR),)
	CONFDIR = .
endif

HALCONF := $(strip $(shell cat $(CONFDIR)/halconf.h $(CONFDIR)/halconf_community.h | egrep -e "\#define"))

else
endif

include ${CHIBIOS_CONTRIB}/os/hal/ports/RP/LLD/I2Cv1/driver.mk
include ${CHIBIOS_CONTRIB}/os/hal/ports/RP/LLD/ADCv1/driver.mk
include ${CHIBIOS_CONTRIB}/os/hal/ports/RP/LLD/USBDv1/driver.mk

# Shared variables
ALLCSRC += $(PLATFORMSRC_CONTRIB)
ALLINC  += $(PLATFORMINC_CONTRIB)
