# FATFS files.
FATFSSRC = ${CHIBIOS_CONTRIB}/os/various/fatfs_bindings/fatfs_diskio.c \
           ${CHIBIOS}/os/various/fatfs_bindings/fatfs_syscall.c \
           ${CHIBIOS}/ext/fatfs/src/ff.c \
           $(CHIBIOS)/ext/fatfs/src/ffunicode.c

FATFSINC = ${CHIBIOS}/ext/fatfs/src ${CHIBIOS_CONTRIB}/os/various/fatfs_bindings

# Shared variables
ALLCSRC += $(FATFSSRC)
ALLINC  += $(FATFSINC)
