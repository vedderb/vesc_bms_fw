# FATFS files.
FATFSSRC = ${CHIBIOS_CONTRIB}/os/various/fatfs_bindings/fatfs_diskio.c \
           ${CHIBIOS}/os/various/fatfs_bindings/fatfs_syscall.c \
           ${CHIBIOS}/ext/fatfs/source/ff.c \
           $(CHIBIOS)/ext/fatfs/source/ffunicode.c

FATFSINC = ${CHIBIOS}/ext/fatfs/source ${CHIBIOS_CONTRIB}/os/various/fatfs_bindings

# Shared variables
ALLCSRC += $(FATFSSRC)
ALLINC  += $(FATFSINC)
