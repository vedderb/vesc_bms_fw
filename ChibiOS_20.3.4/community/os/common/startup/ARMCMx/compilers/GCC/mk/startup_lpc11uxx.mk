# List of the ChibiOS generic LPC11Uxx startup and CMSIS files.
STARTUPSRC = 	$(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/crt1.c


STARTUPASM = 	$(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/crt0_v6m.S \
				$(CHIBIOS_CONTRIB)/os/common/startup/ARMCMx/compilers/GCC/vectors_lpc.S


STARTUPINC = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/ld \
             $(CHIBIOS_CONTRIB)/os/common/startup/ARMCMx/devices/LPC11Uxx \
             $(CHIBIOS)/os/common/ext/CMSIS/include \
             $(CHIBIOS)/os/common/ext/ARM/CMSIS/Core/Include \
             $(CHIBIOS_CONTRIB)/os/common/ext/CMSIS/LPC

STARTUPLD  = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/ld
STARTUPLD_CONTRIB  = $(CHIBIOS_CONTRIB)/os/common/startup/ARMCMx/compilers/GCC/ld

ALLXASMSRC += $(STARTUPASM)
ALLCSRC    += $(STARTUPSRC)
ALLINC     += $(STARTUPINC)
