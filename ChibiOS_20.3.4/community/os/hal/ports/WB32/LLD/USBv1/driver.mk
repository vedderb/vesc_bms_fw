ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_USB TRUE,$(HALCONF)),)
PLATFORMSRC_CONTRIB += $(CHIBIOS_CONTRIB)/os/hal/ports/WB32/LLD/USBv1/hal_usb_lld.c
endif
else
PLATFORMSRC_CONTRIB += $(CHIBIOS_CONTRIB)/os/hal/ports/WB32/LLD/USBv1/hal_usb_lld.c
endif

PLATFORMINC_CONTRIB += $(CHIBIOS_CONTRIB)/os/hal/ports/WB32/LLD/USBv1
