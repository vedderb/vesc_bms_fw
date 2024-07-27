##
##  make targets for the Black Magic Probe
##    See: https://github.com/blacksphere/blackmagic (Black Magic)
##    See: https://github.com/blacksphere/blackmagic/wiki/Frequently-Asked-Questions (Black Magic udev rules)
##
##  Provides the following targets:
##
##  all:        Wrapper for the elf binary
##  flash:      Uses GDB to flash the target via Black Magic
##  debug:      Flashes the target and allows for debugging using gdb and Black Magic
##  debug-tui:  Flashes the target and allows for debugging using gdb's TUI and Black Magic
##

GDB               ?= arm-none-eabi-gdb           # Path to the arm-none-eabi-gdb binary
GDB_PROGRAM       ?= $(BUILDDIR)/$(PROJECT).elf  # Path to the elf binary
GDB_BREAK         ?= main                        # Function to break at when starting the debugger

BMP_GDB           ?= /dev/ttyBmpGdb              # Device file for the Black Magic GDB devic

all: $(GDB_PROGRAM)

flash: $(GDB_PROGRAM)
	$(GDB) $(GDB_PROGRAM) -nx --batch                  \
		-ex 'target extended-remote $(BMP_GDB)'    \
		-ex 'monitor swdp_scan'                    \
		-ex 'attach 1'                             \
		-ex 'load'                                 \
		-ex 'compare-sections'                     \
		-ex 'kill'

debug: $(GDB_PROGRAM)
	$(GDB) $(GDB_PROGRAM) -nx                          \
		-ex 'target extended-remote $(BMP_GDB)'    \
		-ex 'monitor swdp_scan'                    \
		-ex 'attach 1'                             \
		-ex 'load'                                 \
		-ex 'compare-sections'			   \
		-ex 'set mem inaccessible-by-default off'  \
		-ex 'break $(GDB_BREAK)'                   \
		-ex 'c'

debug-tui: $(GDB_PROGRAM)
	$(GDB) $(GDB_PROGRAM) -nx                         \
		-ex 'target extended-remote $(BMP_GDB)'   \
		-ex 'monitor swdp_scan'                   \
		-ex 'attach 1'                            \
		-ex 'load'                                \
		-ex 'compare-sections'                    \
		-ex 'set mem inaccessible-by-default off' \
		-ex 'set pagination off'                  \
		-ex 'focus cmd'	                          \
		-ex 'layout src'                          \
		-ex 'break $(GDB_BREAK)'                  \
		-ex 'c'					  \
		-ex 'set pagination on'                    

.PHONY: all flash debug debug-tui
