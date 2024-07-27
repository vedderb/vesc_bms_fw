# List of all the LPC11Uxx platform files.
PLATFORMSRC := 	${CHIBIOS}/os/hal/ports/common/ARMCMx/nvic.c \
				${CHIBIOS_CONTRIB}/os/hal/ports/LPC/LPC11Uxx/hal_lld.c

# Required include directories
PLATFORMINC =	${CHIBIOS}/os/hal/ports/common/ARMCMx \
						${CHIBIOS_CONTRIB}/os/hal/ports/LPC/LPC11Uxx

include $(CHIBIOS_CONTRIB)/os/hal/ports/LPC/LLD/STM/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/LPC/LLD/GPIO/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/LPC/LLD/USB/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/LPC/LLD/SPI/driver.mk
include $(CHIBIOS_CONTRIB)/os/hal/ports/LPC/LLD/UART/driver.mk

# Shared variables
ALLCSRC += $(PLATFORMSRC)
ALLINC  += $(PLATFORMINC)
