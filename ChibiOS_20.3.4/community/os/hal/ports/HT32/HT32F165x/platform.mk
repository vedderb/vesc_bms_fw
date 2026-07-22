# List of all the platform files.
PLATFORMSRC = $(CHIBIOS)/os/hal/ports/common/ARMCMx/nvic.c \
			$(CHIBIOS_CONTRIB)/os/hal/ports/HT32/HT32F165x/hal_lld.c


# Required include directories
PLATFORMINC = $(CHIBIOS)/os/hal/ports/common/ARMCMx \
              $(CHIBIOS_CONTRIB)/os/hal/ports/HT32/HT32F165x

# Optional platform files.
ifeq ($(USE_SMART_BUILD),yes)

# Configuration files directory
ifeq ($(HALCONFDIR),)
  ifeq ($(CONFDIR),)
    HALCONFDIR = .
  else
    HALCONFDIR := $(CONFDIR)
  endif
endif

HALCONF := $(strip $(shell cat $(HALCONFDIR)/halconf.h | egrep -e "\#define"))
endif #ifeq ($(USE_SMART_BUILD), yes)

# Drivers compatible with the platform.
include $(CHIBIOS_CONTRIB)/os/hal/ports/HT32/LLD/TIMv1/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/HT32/LLD/SPIv1/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/HT32/LLD/GPIOv1/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/HT32/LLD/USBv1/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/HT32/LLD/USART_F165x/driver.mk


# Shared variables
ALLCSRC += $(PLATFORMSRC)
ALLINC  += $(PLATFORMINC)
