# List of all the board related files.
BOARDSRC = ${CHIBIOS_CONTRIB}/os/hal/boards/FREESCALE_FREEDOM_KL25Z/board.c

# Required include directories
BOARDINC = ${CHIBIOS_CONTRIB}/os/hal/boards/FREESCALE_FREEDOM_KL25Z

# Shared variables
ALLCSRC += $(BOARDSRC)
ALLINC  += $(BOARDINC)
