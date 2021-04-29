DRIVERSRC = 	drivers/ltc6813.c \
				drivers/hdc1080.c \
				drivers/bq76940.c
DRIVERINC = 	drivers

# Shared variables
ALLCSRC += $(DRIVERSRC)
ALLINC  += $(DRIVERINC)
