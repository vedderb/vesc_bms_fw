ALLINC		+= 	$(CHIBIOS_CONTRIB)/os/various/fault_handlers		\
				$(CHIBIOS_CONTRIB)/os/common/ports/ARMCMx/compilers/GCC/utils/
ALLCSRC 	+= 	$(CHIBIOS_CONTRIB)/os/common/ports/ARMCMx/compilers/GCC/utils/fault_handlers_v7m.c
ALLXASMSRC 	+= 	$(CHIBIOS_CONTRIB)/os/common/ports/ARMCMx/compilers/GCC/utils/hardfault_handler_v7m.S
