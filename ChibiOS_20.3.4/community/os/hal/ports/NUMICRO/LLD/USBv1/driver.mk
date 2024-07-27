ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_USB TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/LLD/USBv1/hal_usb_lld.c
PLATFORMASM += $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/LLD/USBv1/usb_memcpy.S
endif
else
PLATFORMSRC += $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/LLD/USBv1/hal_usb_lld.c
PLATFORMASM += $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/LLD/USBv1/usb_memcpy.S
endif

PLATFORMINC += $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/LLD/USBv1
