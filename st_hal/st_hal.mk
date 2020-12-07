# ST HAL files.
STHALSRC = st_hal/stm32l4xx_hal_flash.c \
	st_hal/stm32l4xx_hal_flash_ex.c \
	st_hal/hal_functions.c \
	st_hal/stm32l4xx_hal_flash_ramfunc.c

STHALINC = st_hal

# Shared variables
ALLCSRC += $(STHALSRC)
ALLINC  += $(STHALINC)
