
HAL_USB_SRC = ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/LLD/USBHSv1/hal_usb_lld.c \
              ${CHIBIOS_CONTRIB}/ext/nxp-middleware-usb/device/usb_device_dci.c \
              ${CHIBIOS_CONTRIB}/ext/nxp-middleware-usb/device/usb_device_ehci.c \
              ${CHIBIOS_CONTRIB}/ext/nxp-middleware-usb/phy/usb_phy.c \
              ${CHIBIOS_CONTRIB}/ext/mcux-sdk/components/osa/fsl_os_abstraction_bm.c \
              ${CHIBIOS_CONTRIB}/ext/mcux-sdk/components/lists/fsl_component_generic_list.c

ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_USB TRUE,$(HALCONF)),)
PLATFORMSRC_CONTRIB += ${HAL_USB_SRC}
endif
else
PLATFORMSRC_CONTRIB += ${HAL_USB_SRC}
endif

PLATFORMINC_CONTRIB += ${CHIBIOS_CONTRIB}/os/hal/ports/MIMXRT1062/LLD/USBHSv1 \
                       ${CHIBIOS_CONTRIB}/ext/nxp-middleware-usb/include \
                       ${CHIBIOS_CONTRIB}/ext/nxp-middleware-usb/device \
                       ${CHIBIOS_CONTRIB}/ext/nxp-middleware-usb/phy \
                       ${CHIBIOS_CONTRIB}/ext/mcux-sdk/components/osa \
                       ${CHIBIOS_CONTRIB}/ext/mcux-sdk/components/lists
