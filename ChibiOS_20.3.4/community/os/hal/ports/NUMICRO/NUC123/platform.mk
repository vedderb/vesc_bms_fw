PLATFORMSRC := $(CHIBIOS)/os/hal/ports/common/ARMCMx/nvic.c \
               $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/NUC123/hal_lld.c

# Required include directories.
PLATFORMINC := $(CHIBIOS)/os/hal/ports/common/ARMCMx \
               $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/NUC123 \
               $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/LLD

# Optional platform files.
ifeq ($(USE_SMART_BUILD),yes)
HALCONF := $(strip $(shell cat $(CONFDIR)/halconf.h $(CONFDIR)/halconf_community.h | egrep -e "\#define"))
endif

# Drivers compatible with the platform.
include $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/LLD/GPIOv1/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/LLD/TIMv1/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/LLD/USBv1/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/LLD/SERIALv1/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/LLD/I2Cv1/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/LLD/FLASHv1/driver.mk

# Shared variables
ALLCSRC    += $(PLATFORMSRC)
ALLXASMSRC += $(PLATFORMASM)
ALLINC     += $(PLATFORMINC)
