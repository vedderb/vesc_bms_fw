BLACKMAGICSRC =	blackmagic/bm_if.c \
				blackmagic/swdptap.c \
				blackmagic/timing.c \
				blackmagic/platform.c \
				blackmagic/exception.c \
				blackmagic/target/swdptap_generic.c \
				blackmagic/target/target.c \
				blackmagic/target/adiv5_swdp.c \
				blackmagic/target/adiv5.c \
				blackmagic/target/cortexm.c \
				blackmagic/target/nrf51.c \

BLACKMAGICINC =	blackmagic \
				blackmagic/target \

# Shared variables
ALLCSRC += $(BLACKMAGICSRC)
ALLINC  += $(BLACKMAGICINC)
				