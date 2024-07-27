UARTSRC = ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/LLD/UARTv1/hal_serial_lld.c \
          ${CHIBIOS_CONTRIB}/ext/mcux-sdk/drivers/lpuart/fsl_lpuart.c \
          ${CHIBIOS_CONTRIB}/ext/mcux-sdk/devices/MIMXRT1062/drivers/fsl_clock.c

ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_SERIAL TRUE,$(HALCONF)),)
PLATFORMSRC_CONTRIB += $(UARTSRC)
endif
else
PLATFORMSRC_CONTRIB += $(UARTSRC)
endif

PLATFORMINC_CONTRIB += ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/LLD/UARTv1 \
                       ${CHIBIOS_CONTRIB}/ext/mcux-sdk/drivers/common \
                       ${CHIBIOS_CONTRIB}/ext/mcux-sdk/drivers/lpuart \
                       ${CHIBIOS_CONTRIB}/ext/mcux-sdk/devices/MIMXRT1062
