PLATFORMSRC += ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32VF103/TIM/hal_st_lld.c

ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_GPT TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32VF103/TIM/hal_gpt_lld.c
endif
ifneq ($(findstring HAL_USE_ICU TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32VF103/TIM/hal_icu_lld.c
endif
ifneq ($(findstring HAL_USE_PWM TRUE,$(HALCONF)),)
PLATFORMSRC += ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32VF103/TIM/hal_pwm_lld.c
endif
else
PLATFORMSRC += ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32VF103/TIM/hal_gpt_lld.c
PLATFORMSRC += ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32VF103/TIM/hal_icu_lld.c
PLATFORMSRC += ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32VF103/TIM/hal_pwm_lld.c
endif

PLATFORMINC += ${CHIBIOS_CONTRIB}/os/hal/ports/GD/GD32VF103/TIM
