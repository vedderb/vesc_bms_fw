# List of the ChibiOS/RT RISCV generic port files.
PORTSRC = $(CHIBIOS_CONTRIB)/os/common/ports/RISCV-ECLIC/chcore.c

PORTASM = $(CHIBIOS_CONTRIB)/os/common/ports/RISCV-ECLIC/compilers/GCC/chcoreasm.S

PORTINC = $(CHIBIOS_CONTRIB)/os/common/ports/RISCV-ECLIC \
          $(CHIBIOS)/os/common/ports/RISCV-ECLIC/compilers/GCC

# Shared variables
ALLXASMSRC += $(PORTASM)
ALLCSRC    += $(PORTSRC)
ALLINC     += $(PORTINC)
