DRIVERSRC = 	drivers/ltc6813.c \
				drivers/hdc1080.c \
				drivers/sht30.c \
				drivers/shtc3.c \
				drivers/bme280_if.c \
				drivers/bme280_driver/bme280.c

DRIVERINC = 	drivers \
				drivers/bme280_driver

# Shared variables
ALLCSRC += $(DRIVERSRC)
ALLINC  += $(DRIVERINC)
