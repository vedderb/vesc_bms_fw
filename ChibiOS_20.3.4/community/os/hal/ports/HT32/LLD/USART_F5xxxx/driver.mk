ifeq ($(USE_SMART_BUILD),yes)
ifneq ($(findstring HAL_USE_SERIAL TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS_CONTRIB)/os/hal/ports/HT32/LLD/USART_F5xxxx/hal_serial_lld.c
endif
ifneq ($(findstring HAL_USE_UART TRUE,$(HALCONF)),)
PLATFORMSRC += $(CHIBIOS_CONTRIB)/os/hal/ports/HT32/LLD/USART_F5xxxx/hal_uart_lld.c
endif
else
PLATFORMSRC += $(CHIBIOS_CONTRIB)/os/hal/ports/HT32/LLD/USART_F5xxxx/hal_serial_lld.c
PLATFORMSRC += $(CHIBIOS_CONTRIB)/os/hal/ports/HT32/LLD/USART_F5xxxx/hal_uart_lld.c
endif

PLATFORMINC += $(CHIBIOS_CONTRIB)/os/hal/ports/HT32/LLD/USART_F5xxxx
