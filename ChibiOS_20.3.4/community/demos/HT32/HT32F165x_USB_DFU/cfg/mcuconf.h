#ifndef MCUCONF_H
#define MCUCONF_H

#define HT32F1654_MCUCONF

/*
 * HAL driver system settings.
 */

/*
 * Clock configuration.
 */

#define HT32_CK_HSE_FREQUENCY   8000000UL           // 8 MHz
#define HT32_CKCU_SW            CKCU_GCCR_SW_PLL
#define HT32_PLL_USE_HSE        TRUE
#define HT32_PLL_FBDIV          18                  // 8 MHz -> 144 MHz
#define HT32_PLL_OTDIV          0
#define HT32_AHB_PRESCALER      2                   // 144 MHz -> 72 MHz
#define HT32_USART_PRESCALER    1                   // 72 MHz
#define HT32_USB_PRESCALER      3                   // 144 MHz -> 48 MHz
// SysTick uses processor clock at 72MHz
#define HT32_ST_USE_HCLK        TRUE

#define HT32_GPT_USE_BFTM0                  FALSE
#define HT32_GPT_BFTM0_IRQ_PRIORITY         4
/*
 * USB driver settings
 */
#define HT32_USB_USE_USB0                   TRUE
#define HT32_USB_USB0_IRQ_PRIORITY          5

#define HT32_PWM_USE_GPTM1                  FALSE

#endif
