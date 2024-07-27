ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_USB TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32VF103/OTG/hal_usb_lld.c
endif
else
PLATFORMSRC += ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32VF103/OTG/hal_usb_lld.c
endif

PLATFORMINC += ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32VF103/OTG
