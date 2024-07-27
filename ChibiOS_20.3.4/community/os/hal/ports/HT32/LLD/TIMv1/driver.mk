PLATFORMSRC += $(CHIBIOS_CONTRIB)/os/hal/ports/HT32/LLD/TIMv1/hal_st_lld.c

ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_GPT TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS_CONTRIB)/os/hal/ports/HT32/LLD/TIMv1/hal_gpt_lld.c
endif
ifneq ($(findstring HAL_USE_PWM TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS_CONTRIB)/os/hal/ports/HT32/LLD/TIMv1/hal_pwm_lld.c
endif
else
PLATFORMSRC += $(CHIBIOS_CONTRIB)/os/hal/ports/HT32/LLD/TIMv1/hal_gpt_lld.c
PLATFORMSRC += $(CHIBIOS_CONTRIB)/os/hal/ports/HT32/LLD/TIMv1/hal_pwm_lld.c
endif

PLATFORMINC += $(CHIBIOS_CONTRIB)/os/hal/ports/HT32/LLD/TIMv1
