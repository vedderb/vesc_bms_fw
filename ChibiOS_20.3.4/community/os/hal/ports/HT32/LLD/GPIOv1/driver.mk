ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_PAL TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS_CONTRIB)/os/hal/ports/HT32/LLD/GPIOv1/hal_pal_lld.c
endif
else
PLATFORMSRC += $(CHIBIOS_CONTRIB)/os/hal/ports/HT32/LLD/GPIOv1/hal_pal_lld.c
endif

PLATFORMINC += $(CHIBIOS_CONTRIB)/os/hal/ports/HT32/LLD/GPIOv1
