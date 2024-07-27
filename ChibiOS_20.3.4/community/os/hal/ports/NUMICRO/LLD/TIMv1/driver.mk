ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_PWM TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/LLD/TIMv1/hal_pwm_lld.c
endif
else
PLATFORMSRC += $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/LLD/TIMv1/hal_pwm_lld.c
endif

PLATFORMSRC += $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/LLD/TIMv1/hal_st_lld.c
PLATFORMINC += $(CHIBIOS_CONTRIB)/os/hal/ports/NUMICRO/LLD/TIMv1
