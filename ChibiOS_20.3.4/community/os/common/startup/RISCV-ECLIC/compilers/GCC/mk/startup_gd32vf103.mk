# List of the ChibiOS generic FE310 startup files.
STARTUPSRC = $(CHIBIOS_CONTRIB)/os/common/startup/RISCV-ECLIC/compilers/GCC/crt1.c

STARTUPASM = $(CHIBIOS_CONTRIB)/os/common/startup/RISCV-ECLIC/compilers/GCC/crt0.S \
             $(CHIBIOS_CONTRIB)/os/common/startup/RISCV-ECLIC/compilers/GCC/vectors.S

STARTUPINC = $(CHIBIOS)/os/common/portability/GCC \
             $(CHIBIOS_CONTRIB)/os/common/startup/RISCV-ECLIC/compilers/GCC \
             $(CHIBIOS_CONTRIB)/os/common/startup/RISCV-ECLIC/devices/GD32VF103 \
             $(CHIBIOS_CONTRIB)/os/common/ext/NMSIS/Core/Include

STARTUPLD  = $(CHIBIOS_CONTRIB)/os/common/startup/RISCV-ECLIC/compilers/GCC/ld

# Shared variables
ALLXASMSRC += $(STARTUPASM)
ALLCSRC    += $(STARTUPSRC)
ALLINC     += $(STARTUPINC)

