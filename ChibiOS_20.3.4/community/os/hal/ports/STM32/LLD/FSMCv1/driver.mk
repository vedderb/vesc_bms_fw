ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_SDRAM TRUE,$(HALCONF)),)
PLATFORMSRC_CONTRIB += ${CHIBIOS_CONTRIB}/os/hal/ports/STM32/LLD/FSMCv1/hal_sdram_lld.c
endif
ifneq ($(findstring HAL_USE_SRAM TRUE,$(HALCONF)),)
PLATFORMSRC_CONTRIB += ${CHIBIOS_CONTRIB}/os/hal/ports/STM32/LLD/FSMCv1/hal_sram_lld.c
endif
ifneq ($(findstring HAL_USE_NAND TRUE,$(HALCONF)),)
PLATFORMSRC_CONTRIB += ${CHIBIOS_CONTRIB}/os/hal/ports/STM32/LLD/FSMCv1/hal_nand_lld.c
endif
else
PLATFORMSRC_CONTRIB += ${CHIBIOS_CONTRIB}/os/hal/ports/STM32/LLD/FSMCv1/hal_sram_lld.c  \
                       ${CHIBIOS_CONTRIB}/os/hal/ports/STM32/LLD/FSMCv1/hal_sdram_lld.c \
                       ${CHIBIOS_CONTRIB}/os/hal/ports/STM32/LLD/FSMCv1/hal_nand_lld.c
endif

PLATFORMINC_CONTRIB += ${CHIBIOS_CONTRIB}/os/hal/ports/STM32/LLD/FSMCv1
