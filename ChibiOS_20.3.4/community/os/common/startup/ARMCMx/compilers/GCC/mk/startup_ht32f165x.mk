# ChibiOS/RT - Copyright (c) 2020 Yaotian Feng
# startup_ht32f165x.mk - Makefile for Holtek HT32F165x series uC.


# List of the ChibiOS generic HT32F165x startup and CMSIS files.
STARTUPSRC = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/crt1.c

STARTUPASM = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/crt0_v7m.S \
             $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/vectors.S

STARTUPINC = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC \
             $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/ld \
             $(CHIBIOS_CONTRIB)/os/common/startup/ARMCMx/devices/HT32F165x \
             $(CHIBIOS)/os/common/ext/ARM/CMSIS/Core/Include \
             $(CHIBIOS_CONTRIB)/os/common/ext/CMSIS/HT32/HT32F165x

STARTUPLD  = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/ld
STARTUPLD_CONTRIB  = $(CHIBIOS_CONTRIB)/os/common/startup/ARMCMx/compilers/GCC/ld

ALLXASMSRC += $(STARTUPASM)
ALLCSRC    += $(STARTUPSRC)
ALLINC     += $(STARTUPINC)
