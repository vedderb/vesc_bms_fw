
TINYUSBSRC = $(shell find $(TINYUSB)/src/ -type f -name "*.c")

TINYUSBINC = \
    $(CHIBIOS_CONTRIB)/os/various/tinyusb_bindings \
	$(shell find $(TINYUSB)/src/ -type d)

ALLCSRC += $(TINYUSBSRC)
ALLINC += $(TINYUSBINC)
