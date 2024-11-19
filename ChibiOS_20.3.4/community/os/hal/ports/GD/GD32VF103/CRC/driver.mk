ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_CRC TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32VF103/CRC/hal_crc_lld.c \
               ${CHIBIOS_CONTRIB}/os/various/crcsw.c
endif
else
PLATFORMSRC += ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32VF103/CRC/hal_crc_lld.c \
               ${CHIBIOS_CONTRIB}/os/various/crcsw.c
endif

PLATFORMINC += ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32VF103/CRC
