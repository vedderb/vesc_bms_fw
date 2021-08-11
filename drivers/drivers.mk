DRIVERSRC = 	drivers/ltc6813.c \
				drivers/hdc1080.c \
				drivers/sht30.c

DRIVERINC = 	drivers

# Shared variables
ALLCSRC += $(DRIVERSRC)
ALLINC  += $(DRIVERINC)
