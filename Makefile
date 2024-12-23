##############################################################################
# Build global options
# NOTE: Can be overridden externally.
#


ifndef GIT_COMMIT_HASH
  GIT_COMMIT_HASH := $(shell git rev-parse HEAD)
endif

ifndef GIT_BRANCH_NAME
  GIT_BRANCH_NAME := $(shell git rev-parse --abbrev-ref HEAD)
endif

ifdef HW_SRC
  ifndef HW_HEADER
    $(error HW_HEADER not defined while HW_SRC was set, you must set both!)
  endif
  USE_CUSTOM_HW := 1
endif
ifdef HW_HEADER
  ifndef HW_SRC
    $(error HW_SRC not defined while HW_HEADER was set, you must set both!)
  endif
  USE_CUSTOM_HW := 1
endif

# Compiler options here.
ifeq ($(USE_OPT),)
  USE_OPT = -Os -ggdb -fomit-frame-pointer -falign-functions=16 -D_GNU_SOURCE
  USE_OPT += -DBOARD_OTG_NOVBUSSENS -DGIT_COMMIT_HASH=\"$(GIT_COMMIT_HASH)\" -DGIT_BRANCH_NAME=\"$(GIT_BRANCH_NAME)\"
  ifdef USER_GIT_COMMIT_HASH
    USE_OPT += -DUSER_GIT_COMMIT_HASH=\"$(USER_GIT_COMMIT_HASH)\"
  endif
  ifdef USER_GIT_BRANCH_NAME
    USE_OPT += -DUSER_GIT_BRANCH_NAME=\"$(USER_GIT_BRANCH_NAME)\"
  endif
  
  ifneq ($(USE_CUSTOM_HW),)
    USE_OPT += -DHW_SOURCE="\"$(HW_SRC)\"" -DHW_HEADER="\"$(HW_HEADER)\""
  endif
  
  USE_OPT += $(build_args)
endif

# C specific options here (added to USE_OPT).
ifeq ($(USE_COPT),)
  USE_COPT =
endif

# C++ specific options here (added to USE_OPT).
ifeq ($(USE_CPPOPT),)
  USE_CPPOPT = -fno-rtti
endif

# Enable this if you want the linker to remove unused code and data
ifeq ($(USE_LINK_GC),)
  USE_LINK_GC = yes
endif

# Linker extra options here.
ifeq ($(USE_LDOPT),)
  USE_LDOPT =
endif

# Enable this if you want link time optimizations (LTO)
ifeq ($(USE_LTO),)
  USE_LTO = yes
endif

# If enabled, this option allows to compile the application in THUMB mode.
ifeq ($(USE_THUMB),)
  USE_THUMB = yes
endif

# Verbose compile output deactivated if not explicitly set.
ifeq ($(USE_VERBOSE_COMPILE),)
  USE_VERBOSE_COMPILE = no
endif

# If enabled, this option makes the build process faster by not compiling
# modules not used in the current configuration.
ifeq ($(USE_SMART_BUILD),)
  USE_SMART_BUILD = yes
endif

#
# Build global options
##############################################################################

##############################################################################
# Architecture or project specific options
#

# Stack size to be allocated to the Cortex-M process stack. This stack is
# the stack used by the main() thread.
ifeq ($(USE_PROCESS_STACKSIZE),)
  USE_PROCESS_STACKSIZE = 2048
endif

# Stack size to the allocated to the Cortex-M main/exceptions stack. This
# stack is used for processing interrupts and exceptions.
ifeq ($(USE_EXCEPTIONS_STACKSIZE),)
  USE_EXCEPTIONS_STACKSIZE = 0x400
endif

# Enables the use of FPU (no, softfp, hard).
ifeq ($(USE_FPU),)
  USE_FPU = hard
endif

#
# Architecture or project specific options
##############################################################################

##############################################################################
# Project, sources and paths
#

# Define project name here
PROJECT = vesc_bms

# Imported source files and paths
CHIBIOS = ChibiOS_20.3.0

# Licensing files.
include $(CHIBIOS)/os/license/license.mk
# Startup files.
include $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk/startup_stm32l4xx.mk
# HAL-OSAL files (optional).
include $(CHIBIOS)/os/hal/hal.mk
include $(CHIBIOS)/os/hal/ports/STM32/STM32L4xx/platform.mk
include $(CHIBIOS)/os/hal/osal/rt-nil/osal.mk
# RTOS files (optional).
include $(CHIBIOS)/os/rt/rt.mk
include $(CHIBIOS)/os/common/ports/ARMCMx/compilers/GCC/mk/port_v7m.mk
# Other files (optional).
#include $(CHIBIOS)/test/lib/test.mk
#include $(CHIBIOS)/test/rt/rt_test.mk
#include $(CHIBIOS)/test/oslib/oslib_test.mk
include st_hal/st_hal.mk
include drivers/drivers.mk
include blackmagic/blackmagic.mk
include compression/compression.mk

# Define linker script file here
LDSCRIPT= STM32L476xG.ld

# C sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CSRC = $(ALLCSRC) \
       $(TESTSRC) \
       $(CHIBIOS)/os/various/syscalls.c \
       main.c \
       usbcfg.c \
       pwr.c \
       utils.c \
       board.c \
       comm_can.c \
       crc.c \
       hwconf/hw.c \
       buffer.c \
       comm_usb.c \
       commands.c \
       packet.c \
       bms_if.c \
       i2c_bb.c \
       config/confparser.c \
       config/confxml.c \
       mempools.c \
       terminal.c \
       flash_helper.c \
       conf_general.c \
       timeout.c \
       sleep.c \
       comm_uart.c \
       selftest.c

# C++ sources that can be compiled in ARM or THUMB mode depending on the global
# setting.
CPPSRC = $(ALLCPPSRC)

# C sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACSRC =

# C++ sources to be compiled in ARM mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
ACPPSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCSRC =

# C sources to be compiled in THUMB mode regardless of the global setting.
# NOTE: Mixing ARM and THUMB mode enables the -mthumb-interwork compiler
#       option that results in lower performance and larger code size.
TCPPSRC =

# List ASM source files here
ASMSRC = $(ALLASMSRC)
ASMXSRC = $(ALLXASMSRC)

INCDIR = $(ALLINC) $(TESTINC) hwconf config st_hal drivers

#
# Project, sources and paths
##############################################################################

##############################################################################
# Compiler settings
#

MCU  = cortex-m4

#TRGT = arm-elf-
TRGT = arm-none-eabi-
CC   = $(TRGT)gcc
CPPC = $(TRGT)g++
# Enable loading with g++ only if you need C++ runtime support.
# NOTE: You can use C++ even without C++ support if you are careful. C++
#       runtime support makes code size explode.
LD   = $(TRGT)gcc
#LD   = $(TRGT)g++
CP   = $(TRGT)objcopy
AS   = $(TRGT)gcc -x assembler-with-cpp
AR   = $(TRGT)ar
OD   = $(TRGT)objdump
SZ   = $(TRGT)size
HEX  = $(CP) -O ihex
BIN  = $(CP) -O binary

# ARM-specific options here
AOPT =

# THUMB-specific options here
TOPT = -mthumb -DTHUMB

# Define C warning options here
CWARN = -Wall -Wextra -Wundef -Wstrict-prototypes

# Define C++ warning options here
CPPWARN = -Wall -Wextra -Wundef

#
# Compiler settings
##############################################################################

##############################################################################
# Start of user section
#

# List all user C define here, like -D_DEBUG=1
UDEFS =

# Define ASM defines here
UADEFS =

# List all user directories here
UINCDIR =

# List the user directory to look for the libraries here
ULIBDIR =

# List all user libraries here
ULIBS = -lm --specs=nosys.specs

#
# End of user defines
##############################################################################

RULESPATH = $(CHIBIOS)/os/common/startup/ARMCMx/compilers/GCC/mk
include $(RULESPATH)/rules.mk

SRC_SENTINEL := $(BUILDDIR)/hw_src
HEADER_SENTINEL := $(BUILDDIR)/hw_header

# Update the tracker files if HW_SRC or HW_HEADER has changed to trigger a rebuild
ifeq ($(shell if [[ -d $(BUILDDIR) ]]; then printf 1; else printf ""; fi),1)
  ifneq ($(file < $(SRC_SENTINEL)),$(HW_SRC))
    $(info Updated $(SRC_SENTINEL))
    $(file > $(SRC_SENTINEL),$(HW_SRC))
    $(shell touch hwconf/hw.c conf_general.h)
  endif
  ifneq ($(file < $(HEADER_SENTINEL)),$(HW_HEADER))
    $(info Updated $(HEADER_SENTINEL))
    $(file > $(HEADER_SENTINEL),$(HW_HEADER))
    $(shell touch hwconf/hw.h conf_general.h)
  endif
endif

upload: build/$(PROJECT).bin
	openocd -f stm32l4_stlinkv2.cfg \
		-c "program build/$(PROJECT).elf verify reset exit"

upload_remote: build/$(PROJECT).bin
	./upload_remote build/$(PROJECT).bin benjamin 127.0.0.1 62122

mass_erase:
	@openocd -f stm32l4_stlinkv2.cfg \
		-c "init" \
		-c "reset halt" \
		-c "stm32l4x mass_erase 0" \
		-c "sleep 200" \
		-c "shutdown"

server:
	@openocd -f stm32l4_stlinkv2.cfg \
		-c "init" \
		-c "reset halt"
