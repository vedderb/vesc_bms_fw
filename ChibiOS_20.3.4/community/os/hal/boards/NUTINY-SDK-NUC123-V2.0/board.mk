# List of all the board related files.
BOARDSRC = $(CHIBIOS_CONTRIB)/os/hal/boards/NUTINY-SDK-NUC123-V2.0/board.c

# Required include directories
BOARDINC = $(CHIBIOS_CONTRIB)/os/hal/boards/NUTINY-SDK-NUC123-V2.0

ALLCSRC += $(BOARDSRC)
ALLINC += $(BOARDINC)
