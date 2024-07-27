ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_RTC TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32VF103/RTC/hal_rtc_lld.c
endif
else
PLATFORMSRC += ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32VF103/RTC/hal_rtc_lld.c
endif

PLATFORMINC += ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32VF103/RTC
