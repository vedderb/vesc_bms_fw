#ifndef HT32F165x_REG_H
#define HT32F165x_REG_H

#ifndef __IO
    #define __IO volatile
#endif

// Constants
// /////////////////////////////////////////////////////////////////////////////
#define AFIO_DEFAULT    0
#define AFIO_GPIO       1
#define AFIO_ADC        2
#define AFIO_CMP        3
#define AFIO_TM         4
#define AFIO_SPI        5
#define AFIO_USART      6
#define AFIO_I2C        7
#define AFIO_SMC        8
#define AFIO_EBI        9
#define AFIO_I2S        10
#define AFIO_OTHER      15

// Flash Memory Controller
// /////////////////////////////////////////////////////////////////////////////
typedef struct {
  __IO uint32_t TADR;           //!< 0x000          Flash Target Address Register
  __IO uint32_t WRDR;           //!< 0x004          Flash Write Data Register
       uint32_t RESERVED0[1];   //!< 0x008          Reserved
  __IO uint32_t OCMR;           //!< 0x00C          Flash Operation Command Register
  __IO uint32_t OPCR;           //!< 0x010          Flash Operation Control Register
  __IO uint32_t OIER;           //!< 0x014          Flash Operation Interrupt Enable Register
  __IO uint32_t OISR;           //!< 0x018          Flash Operation Interrupt and Status Register
       uint32_t RESERVED1[1];   //!< 0x01C          Reserved
  __IO uint32_t PPSR[4];        //!< 0x020 ~ 0x02C  Flash Page Erase/Program Protection Status Register
  __IO uint32_t CPSR;           //!< 0x030          Flash Security Protection Status Register
       uint32_t RESERVED2[51];  //!< 0x034 ~ 0x0FC  Reserved
  __IO uint32_t VMCR;           //!< 0x100          Flash Vector Mapping Control Register
       uint32_t RESERVED3[31];  //!< 0x104 ~ 0x17C  Reserved
  __IO uint32_t MDID;           //!< 0x180          Manufacturer and Device ID Register
  __IO uint32_t PNSR;           //!< 0x184          Flash Page Number Status Register
  __IO uint32_t PSSR;           //!< 0x188          Flash Page Size Status Register
#if defined(HT32F165x)
       uint32_t RESERVED4[29];  //!< 0x18C ~ 0x1FC  Reserved
#else
  __IO uint32_t DID;            //!< 0x18C          Device ID Register
       uint32_t RESERVED4[28];  //!< 0x190 ~ 0x1FC  Reserved
#endif
  __IO uint32_t CFCR;           //!< 0x200          Flash Cache and Pre-fetch Control Register
       uint32_t RESERVED5[63];  //!< 0x204 ~ 0x2FC  Reserved
  __IO uint32_t SBVT[4];        //!< 0x300 ~ 0x30C  SRAM Booting Vector (4x32Bit)
#if defined(HT32F165x)
#else
  __IO uint32_t CID[4];         //!< 0x310 ~ 0x31C  Custom ID Register
#endif
} FMC_TypeDef;

#define FMC_OCMR_CMD_MASK           (0xF << 0)
#define FMC_OCMR_CMD_IDLE           (0x0 << 0)
#define FMC_OCMR_CMD_WORD_PROGRAM   (0x4 << 0)
#define FMC_OCMR_CMD_PAGE_ERASE     (0x8 << 0)
#define FMC_OCMR_CMD_MASS_ERASE     (0xA << 0)
#define FMC_OPCR_OPM_MASK           (0xF << 1)
#define FMC_OPCR_OPM_IDLE           (0x6 << 1)
#define FMC_OPCR_OPM_COMMIT         (0xA << 1)
#define FMC_OPCR_OPM_FINISHED       (0xE << 1)
#define FMC_CFCR_CE                 (1U << 12)
#define FMC_CFCR_WAIT_MASK          (7U << 0)
#define FMC_CFCR_WAIT_0             (1U)
#define FMC_CFCR_WAIT_1             (2U)
#define FMC_CFCR_WAIT_2             (3U)

// Power Control Unit
// /////////////////////////////////////////////////////////////////////////////
typedef struct {
       uint32_t RESERVE0[64];
  __IO uint32_t BAKSR;          //!< 0x000          Status Register
  __IO uint32_t BAKCR;          //!< 0x004          Control Register
  __IO uint32_t BAKTEST;        //!< 0x008          Test Register
  __IO uint32_t HSIRCR;         //!< 0x00C          HSI Ready Counter Control Register
  __IO uint32_t LVDCSR;         //!< 0x010          Low Voltage/Brown Out Detect Control and Status Register
       uint32_t RESERVE1[59];   //!< 0x014 ~ 0x0FC  Reserved
  __IO uint32_t BAKREG[10];     //!< 0x100 ~ 0x124  Backup Register 0 ~ 9
} PWRCU_TypeDef;

// Clock Control Unit
// /////////////////////////////////////////////////////////////////////////////
typedef struct {
  __IO uint32_t GCFGR;          //!< 0x000          Global Clock Configuration Register
  __IO uint32_t GCCR;           //!< 0x004          Global Clock Control Register
  __IO uint32_t GCSR;           //!< 0x008          Global Clock Status Register
  __IO uint32_t GCIR;           //!< 0x00C          Global Clock Interrupt Register
       uint32_t RESERVED0[2];   //!< 0x010 ~ 0x014  Reserved
  __IO uint32_t PLLCFGR;        //!< 0x018          PLL Configuration Register
  __IO uint32_t PLLCR;          //!< 0x01C          PLL Control Register
  __IO uint32_t AHBCFGR;        //!< 0x020          AHB Configuration Register
  __IO uint32_t AHBCCR;         //!< 0x024          AHB Clock Control Register
  __IO uint32_t APBCFGR;        //!< 0x028          APB Configuration Register
  __IO uint32_t APBCCR0;        //!< 0x02C          APB Clock Control Register 0
  __IO uint32_t APBCCR1;        //!< 0x030          APB Clock Control Register 1
  __IO uint32_t CKST;           //!< 0x034          Clock source status Register
#if defined(HT32F1653_4)
  __IO uint32_t APBPCSR0;       //!< 0x038          APB Peripheral Clock Selection Register 0
  __IO uint32_t APBPCSR1;       //!< 0x03C          APB Peripheral Clock Selection Register 1
  __IO uint32_t HSICR;          //!< 0x040          HSI Control Register
  __IO uint32_t HSIATCR;        //!< 0x044          HSI Auto Trimming Counter Register
#else
       uint32_t RESERVED1[4];   //!< 0x038 ~ 0x044  Reserved
#endif
       uint32_t RESERVED2[174]; //!< 0x048 ~ 0x2FC  Reserved
  __IO uint32_t LPCR;           //!< 0x300          Low Power Control Register
  __IO uint32_t MCUDBGCR;       //!< 0x304          MCU Debug Control Register
} CKCU_TypeDef;

#define CKCU_GCFGR_LPMOD_MASK       (7U << 29)
#define CKCU_GCFGR_USBPRE_MASK      (3U << 22)
#define CKCU_GCFGR_URPRE_MASK       (3U << 20)
#define CKCU_GCFGR_PLLSRC           (1U << 8)
#define CKCU_GCFGR_CKOUTSRC_MASK    (7U << 0)
#define CKCU_GCFGR_CKOUTSRC_CK_REF  (0U << 0)
#define CKCU_GCFGR_CKOUTSRC_CK_AHB  (1U << 0)
#define CKCU_GCFGR_CKOUTSRC_CK_SYS  (2U << 0)
#define CKCU_GCFGR_CKOUTSRC_CK_HSE  (3U << 0)
#define CKCU_GCFGR_CKOUTSRC_CK_HSI  (4U << 0)
#define CKCU_GCFGR_CKOUTSRC_CK_LSE  (5U << 0)
#define CKCU_GCFGR_CKOUTSRC_CK_LSI  (6U << 0)
#define CKCU_GCCR_PSRCEN            (1U << 17)
#define CKCU_GCCR_CKMEN             (1U << 16)
#define CKCU_GCCR_HSIEN             (1U << 11)
#define CKCU_GCCR_HSEEN             (1U << 10)
#define CKCU_GCCR_PLLEN             (1U << 9)
#define CKCU_GCCR_SW_MASK           (3U << 0)
#define CKCU_GCCR_SW_PLL            (1U << 0)
#define CKCU_GCCR_SW_HSE            (2U << 0)
#define CKCU_GCCR_SW_HSI            (3U << 0)
#define CKCU_GCSR_LSIRDY            (1U << 5)
#define CKCU_GCSR_LSERDY            (1U << 4)
#define CKCU_GCSR_HSIRDY            (1U << 3)
#define CKCU_GCSR_HSERDY            (1U << 2)
#define CKCU_GCSR_PLLRDY            (1U << 1)
#define CKCU_PLLCFGR_PFBD_MASK      (0x3fU << 23)
#define CKCU_PLLCFGR_POTD_MASK      (3U << 21)
#define CKCU_PLLCR_PLLBPS           (1U << 31)
#define CKCU_AHBCFGR_AHBPRE_MASK    (3U << 0)
#define CKCU_AHBCCR_PAEN            (1U << 16)
#define CKCU_AHBCCR_CRCEN           (1U << 13)
#define CKCU_AHBCCR_EBIEN           (1U << 12)
#define CKCU_AHBCCR_CKREFEN         (1U << 11)
#define CKCU_AHBCCR_USBEN           (1U << 10)
#define CKCU_APBCFGR_ADCDIV_MASK    (7U << 16)
#define CKCU_APBCCR0_I2SEN          (1U << 25)
#define CKCU_APBCCR0_SCIEN          (1U << 24)
#define CKCU_APBCCR0_EXTIEN         (1U << 15)
#define CKCU_APBCCR0_AFIOEN         (1U << 14)
#define CKCU_APBCCR0_UR1EN          (1U << 11)
#define CKCU_APBCCR0_UR0EN          (1U << 10)
#define CKCU_APBCCR0_USR1EN         (1U << 9)
#define CKCU_APBCCR0_USR0EN         (1U << 8)
#define CKCU_APBCCR0_SPI1EN         (1U << 5)
#define CKCU_APBCCR0_SPI0EN         (1U << 4)
#define CKCU_APBCCR0_I2C1EN         (1U << 1)
#define CKCU_APBCCR0_I2C0EN         (1U << 0)
#define CKCU_APBCCR1_ADCEN          (1U << 24)
#define CKCU_APBCCR1_OPA1EN         (1U << 23)
#define CKCU_APBCCR1_OPA0EN         (1U << 22)
#define CKCU_APBCCR1_BFTM1EN        (1U << 17)
#define CKCU_APBCCR1_BFTM0EN        (1U << 16)
#define CKCU_APBCCR1_GPTM1EN        (1U << 9)
#define CKCU_APBCCR1_GPTM0EN        (1U << 8)
#define CKCU_APBCCR1_BKPREN         (1U << 6)
#define CKCU_APBCCR1_WDTREN         (1U << 4)
#define CKCU_APBCCR1_MCTM1EN        (1U << 1)
#define CKCU_APBCCR1_MCTM0EN        (1U << 0)
#define CKCU_CKST_CKSWST_MASK       (3U << 30)
#define CKCU_CKST_HSIST_MASK        (7U << 24)
#define CKCU_CKST_HSEST_MASK        (3U << 16)
#define CKCU_CKST_PLLST_MASK        (0xfU << 8)
#define CKCU_LPCR_USBSLEEP          (1U << 8)
#define CKCU_LPCR_BKISO             (1U << 0)

// Reset Control Unit
// /////////////////////////////////////////////////////////////////////////////
typedef struct {
  __IO uint32_t GRSR;           //!< 0x000          Global Reset Status Register
  __IO uint32_t AHBPRSTR;       //!< 0x004          AHB Peripheral Reset Register
  __IO uint32_t APBPRSTR0;      //!< 0x008          APB Peripheral Reset Register 0
  __IO uint32_t APBPRSTR1;      //!< 0x00C          APB Peripheral Reset Register 1
} RSTCU_TypeDef;

#define RSTCU_GRSR_PORSTF           (1U << 3)
#define RSTCU_GRSR_WDTRSTF          (1U << 2)
#define RSTCU_GRSR_EXTRSTF          (1U << 1)
#define RSTCU_GRSR_SYSRSTF          (1U << 0)
#define RSTCU_AHBPRSTR_PxRST(n)     ((1U << 8) << (n))
#define RSTCU_AHBPRSTR_CRCRST       (1U << 7)
#define RSTCU_AHBPRSTR_EBIRST       (1U << 6)
#define RSTCU_AHBPRSTR_USBRST       (1U << 5)
#define RSTCU_AHBPRSTR_DMARST       (1U << 0)
#define RSTCU_APBPRSTR0_I2SRST      (1U << 25)
#define RSTCU_APBPRSTR0_SCIRST      (1U << 24)
#define RSTCU_APBPRSTR0_EXTIRST     (1U << 15)
#define RSTCU_APBPRSTR0_AFIORST     (1U << 14)
#define RSTCU_APBPRSTR0_UR1RST      (1U << 11)
#define RSTCU_APBPRSTR0_UR0RST      (1U << 10)
#define RSTCU_APBPRSTR0_USR1RST     (1U << 9)
#define RSTCU_APBPRSTR0_USR0RST     (1U << 8)
#define RSTCU_APBPRSTR0_SPI1RST     (1U << 5)
#define RSTCU_APBPRSTR0_SPI0RST     (1U << 4)
#define RSTCU_APBPRSTR0_I2C1RST     (1U << 1)
#define RSTCU_APBPRSTR0_I2C0RST     (1U << 0)
#define RSTCU_APBPRSTR1_ADCRST      (1U << 24)
#define RSTCU_APBPRSTR1_OPA1RST     (1U << 23)
#define RSTCU_APBPRSTR1_OPA0RST     (1U << 22)
#define RSTCU_APBPRSTR1_BFTM1RST    (1U << 17)
#define RSTCU_APBPRSTR1_BFTM0RST    (1U << 16)
#define RSTCU_APBPRSTR1_GPTM1RST    (1U << 9)
#define RSTCU_APBPRSTR1_GPTM0RST    (1U << 8)
#define RSTCU_APBPRSTR1_WDTRST      (1U << 4)
#define RSTCU_APBPRSTR1_MCTM1RST    (1U << 1)
#define RSTCU_APBPRSTR1_MCTM0RST    (1U << 0)

// General Purpose I/O
// /////////////////////////////////////////////////////////////////////////////
typedef struct {
  __IO uint32_t DIRCR;          //!< 0x000          Data Direction Control Register
  __IO uint32_t INER;           //!< 0x004          Input function enable register
  __IO uint32_t PUR;            //!< 0x008          Pull-Up Selection Register
  __IO uint32_t PDR;            //!< 0x00C          Pull-Down Selection Register
  __IO uint32_t ODR;            //!< 0x010          Open Drain Selection Register
  __IO uint32_t DRVR;           //!< 0x014          Drive Current Selection Register
  __IO uint32_t LOCKR;          //!< 0x018          Lock Register
  __IO uint32_t DINR;           //!< 0x01c          Data Input Register
  __IO uint32_t DOUTR;          //!< 0x020          Data Output Register
  __IO uint32_t SRR;            //!< 0x024          Output Set and Reset Control Register
  __IO uint32_t RR;             //!< 0x028          Output Reset Control Register
} GPIO_TypeDef;

// Alternate Function Input/Output
// /////////////////////////////////////////////////////////////////////////////
typedef struct {
  __IO uint32_t ESSR[2];        //!< 0x000 ~ 0x004  EXTI Source Selection Register 0 ~ 1
       uint32_t RESERVE0[6];    //!< 0x008 ~ 0x01C  Reserved
  union {
    struct {
  __IO uint32_t GPACFGR[2];     //!< 0x020 ~ 0x024  GPIO Port A Configuration Register 0 ~ 1
  __IO uint32_t GPBCFGR[2];     //!< 0x028 ~ 0x02C  GPIO Port B Configuration Register 0 ~ 1
  __IO uint32_t GPCCFGR[2];     //!< 0x030 ~ 0x034  GPIO Port C Configuration Register 0 ~ 1
  __IO uint32_t GPDCFGR[2];     //!< 0x038 ~ 0x03C  GPIO Port D Configuration Register 0 ~ 1
#if defined(HT32F1655_6)
  __IO uint32_t GPECFGR[2];     //!< 0x040 ~ 0x044  GPIO Port E Configuration Register 0 ~ 1
#endif
    };
    // alternate mapping
    struct {
  __IO uint32_t GPxCFGR[0][2];  //!< 0x020 ~ 0x044  GPIO Port x Configuration Register 0 ~ 1
    };
  };
} AFIO_TypeDef;

// Nested Vectored Interrupt Controller
// /////////////////////////////////////////////////////////////////////////////
// Implemented in Cortex-M3 Headers

// External Interrupt/Event Controller
// /////////////////////////////////////////////////////////////////////////////
typedef struct {
  __IO uint32_t CFGR0;          //!< 0x000         EXTI Interrupt 0 Configuration Register
  __IO uint32_t CFGR1;          //!< 0x004         EXTI Interrupt 1 Configuration Register
  __IO uint32_t CFGR2;          //!< 0x008         EXTI Interrupt 2 Configuration Register
  __IO uint32_t CFGR3;          //!< 0x00C         EXTI Interrupt 3 Configuration Register
  __IO uint32_t CFGR4;          //!< 0x010         EXTI Interrupt 4 Configuration Register
  __IO uint32_t CFGR5;          //!< 0x014         EXTI Interrupt 5 Configuration Register
  __IO uint32_t CFGR6;          //!< 0x018         EXTI Interrupt 6 Configuration Register
  __IO uint32_t CFGR7;          //!< 0x01C         EXTI Interrupt 7 Configuration Register
  __IO uint32_t CFGR8;          //!< 0x020         EXTI Interrupt 8 Configuration Register
  __IO uint32_t CFGR9;          //!< 0x024         EXTI Interrupt 9 Configuration Register
  __IO uint32_t CFGR10;         //!< 0x028         EXTI Interrupt 10 Configuration Register
  __IO uint32_t CFGR11;         //!< 0x02C         EXTI Interrupt 11 Configuration Register
  __IO uint32_t CFGR12;         //!< 0x030         EXTI Interrupt 12 Configuration Register
  __IO uint32_t CFGR13;         //!< 0x034         EXTI Interrupt 13 Configuration Register
  __IO uint32_t CFGR14;         //!< 0x038         EXTI Interrupt 14 Configuration Register
  __IO uint32_t CFGR15;         //!< 0x03C         EXTI Interrupt 15 Configuration Register
  __IO uint32_t CR;             //!< 0x040         EXTI Interrupt Control Register
  __IO uint32_t EDGEFLGR;       //!< 0x044         EXTI Interrupt Edge Flag Register
  __IO uint32_t EDGESR;         //!< 0x048         EXTI Interrupt Edge Status Register
  __IO uint32_t SSCR;           //!< 0x04C         EXTI Interrupt Software Set Command Register
  __IO uint32_t WAKUPCR;        //!< 0x050         EXTI Interrupt Wakeup Control Register
  __IO uint32_t WAKUPPOLR;      //!< 0x054         EXTI Interrupt Wakeup Polarity Register
  __IO uint32_t WAKUPFLG;       //!< 0x058         EXTI Interrupt Wakeup Flag Register
} EXTI_TypeDef;

// Analog To Digital Converter
// /////////////////////////////////////////////////////////////////////////////

// Operational Amplifier / Comparator
// /////////////////////////////////////////////////////////////////////////////
#if defined(HT32F1653_4)
typedef struct {
  __IO uint32_t CR;             //!< 0x000          Comparator Control Register
  __IO uint32_t VALR;           //!< 0x004          Comparator Voltage Reference Register
  __IO uint32_t IER;            //!< 0x008          Comparator Interrupt Enable Register
  __IO uint32_t TFR;            //!< 0x00C          Comparator Transition Flag Register
} CMP_TypeDef;
#else
typedef struct {
  __IO uint32_t OPACR;          //!< 0x000          Operational Amplifier Control Register
  __IO uint32_t OFVCR;          //!< 0x004          Comparator Input Offset Voltage Cancellation Register
  __IO uint32_t CMPIER;         //!< 0x008          Comparator Interrupt Enable Register
  __IO uint32_t CMPRSR;         //!< 0x00C          Comparator Raw Status Register
  __IO uint32_t CMPISR;         //!< 0x010          Comparator Masked Interrupt Status Register
  __IO uint32_t CMPICLR;        //!< 0x014          Comparator Interrupt Clear Register
} OPACMP_TypeDef;
#endif

// Basic Function Timers
// /////////////////////////////////////////////////////////////////////////////
typedef struct {
  __IO uint32_t CR;             //!< 0x000          Control Register
  __IO uint32_t SR;             //!< 0x004          Status Register
  __IO uint32_t CNTR;           //!< 0x008          Counter Value Register
  __IO uint32_t CMP;            //!< 0x00C          Compare Value Register
} BFTM_TypeDef;

#define BFTM_CR_CEN                 (1U << 2)
#define BFTM_CR_OSM                 (1U << 1)
#define BFTM_CR_MIEN                (1U << 0)
#define BFTM_SR_MIF                 (1U << 0)

// General Purpose Timers
// Motor Control Timers
// /////////////////////////////////////////////////////////////////////////////
typedef struct {
  __IO uint32_t CNTCFR;         //!< 0x000          Timer Counter Configuaration Register
  __IO uint32_t MDCFR;          //!< 0x004          Timer Mode Configuration Register
  __IO uint32_t TRCFR;          //!< 0x008          Timer Trigger Configuration Register
       uint32_t RESERVED0[1];   //!< 0x00C          Reserved
  __IO uint32_t CTR;            //!< 0x010          Timer Counter Register
       uint32_t RESERVED1[3];   //!< 0x014 ~ 0x01C  Reserved
  __IO uint32_t CHnICFR[4];     //!< 0x020 ~ 0x02C  Channel n Input Configuration Register
       uint32_t RESERVED2[4];   //!< 0x030 ~ 0x03C  Reserved
  __IO uint32_t CHnOCFR[4];     //!< 0x040 ~ 0x04C  Channel n Output Configuration Register
  __IO uint32_t CHCTR;          //!< 0x050          Channel Control Register
  __IO uint32_t CHPOLR;         //!< 0x054          Channel Polarity Control Register
       uint32_t RESERVED3[5];   //!< 0x058 ~ 0x068  Reserved
  // note: only available as MCTM
  __IO uint32_t CHBRKCFR;       //!< 0x06C          Channel Break Configuration Register
  __IO uint32_t CHBRKCTR;       //!< 0x070          Channel Break Control Register
  // end note
  __IO uint32_t DICTR;          //!< 0x074          Timer PDMA/Interrupt Control Register
  __IO uint32_t EVGR;           //!< 0x078          Timer Event Generator Register
  __IO uint32_t INTSR;          //!< 0x07C          Timer Interrupt Status Register
  __IO uint32_t CNTR;           //!< 0x080          Timer Counter Register
  __IO uint32_t PSCR;           //!< 0x084          Timer Prescaler Register
  __IO uint32_t CRR;            //!< 0x088          Timer Counter Reload Register
  // note: only available as MCTM
  __IO uint32_t REPR;           //!< 0x08C          Timer Repetition Register
  // end note
  __IO uint32_t CHnCCR[4];      //!< 0x090 ~ 0x09C  Channel n Capture/Compare Register
  __IO uint32_t CHnACR[4];      //!< 0x0A0 ~ 0x0AC  Channel n Asymmentric Compare Register
} TM_TypeDef;

#define TM_CNTCFR_CMSEL_MASK        (3U << 16)
#define TM_CNTCFR_CMSEL_MODE_3      (3U << 16)
#define TM_CNTCFR_CMSEL_MODE_2      (2U << 16)
#define TM_CNTCFR_CMSEL_MODE_1      (1U << 16)
#define TM_CNTCFR_CMSEL_MODE_0      (0U << 16)
#define TM_CTR_CHCCDS               (1U << 16)
#define TM_CTR_COMUS                (1U << 9)
#define TM_CTR_COMPRE               (1U << 8)
#define TM_CTR_CRBE                 (1U << 1)
#define TM_CTR_TME                  (1U << 0)
#define TM_CHnOCFR_CHnPRE           (1U << 4)
#define TM_CHnOCFR_REFnCE           (1U << 3)
#define TM_CHnOCFR_CHnOM(n)         ((((n)>>0)&7)|((((n)>>3)&1)<<8))
#define TM_CHBRKCTR_CHMOE           (1U << 4)

// Real Time Clock
// /////////////////////////////////////////////////////////////////////////////

// Watchdog Timer
// /////////////////////////////////////////////////////////////////////////////

// I2C
// /////////////////////////////////////////////////////////////////////////////
typedef struct {
  __IO uint32_t CR;             //!< 0x000          Control Register
  __IO uint32_t IER;            //!< 0x004          Interrupt Enable Register
  __IO uint32_t ADDR;           //!< 0x008          Address Register
  __IO uint32_t SR;             //!< 0x00C          Status Register
  __IO uint32_t SHPGR;          //!< 0x010          SCL High Period Generation Register
  __IO uint32_t SLPGR;          //!< 0x014          SCL Low Period Generation Register
  __IO uint32_t DR;             //!< 0x018          Data Register
  __IO uint32_t TAR;            //!< 0x01C          Target Register
  __IO uint32_t ADDMR;          //!< 0x020          Address Mask Register
  __IO uint32_t ADDSR;          //!< 0x024          Address Snoop Register
  __IO uint32_t TOUT;           //!< 0x028          Timeout Register
} I2C_TypeDef;

#define I2C_CR_SEQ_FILTER_MASK      (3U << 14)
#define I2C_CR_SEQ_FILTER_2_PCLK    (2U << 14)
#define I2C_CR_SEQ_FILTER_1_PCLK    (1U << 14)
#define I2C_CR_SEQ_FILTER_DISABLE   (0U << 14)
#define I2C_CR_COMB_FILTER_En       (1U << 13)
#define I2C_CR_ENTOUT               (1U << 12)
#define I2C_CR_DMANACK              (1U << 10)
#define I2C_CR_RXDMAE               (1U << 9)
#define I2C_CR_TXDMAE               (1U << 8)
#define I2C_CR_ADRM                 (1U << 7)
#define I2C_CR_I2CEN                (1U << 3)
#define I2C_CR_GCEN                 (1U << 2)
#define I2C_CR_STOP                 (1U << 1)
#define I2C_CR_AA                   (1U << 0)
#define I2C_IER_RXBFIE              (1U << 18)
#define I2C_IER_TXDEIE              (1U << 17)
#define I2C_IER_RXDNEIE             (1U << 16)
#define I2C_IER_TOUTIE              (1U << 11)
#define I2C_IER_BUSERRIE            (1U << 10)
#define I2C_IER_RXNACKIE            (1U << 9)
#define I2C_IER_ARBLOSIE            (1U << 8)
#define I2C_IER_GCSIE               (1U << 3)
#define I2C_IER_ADRSIE              (1U << 2)
#define I2C_IER_STOIE               (1U << 1)
#define I2C_IER_STAIE               (1U << 0)
#define I2C_SR_TXNRX                (1U << 21)
#define I2C_SR_MASTER               (1U << 20)
#define I2C_SR_BUSBUSY              (1U << 19)
#define I2C_SR_RXBF                 (1U << 18)
#define I2C_SR_TXDE                 (1U << 17)
#define I2C_SR_RXDNE                (1U << 16)
#define I2C_SR_TOUTF                (1U << 11)
#define I2C_SR_BUSERR               (1U << 10)
#define I2C_SR_RXNACK               (1U << 9)
#define I2C_SR_ARBLOS               (1U << 8)
#define I2C_SR_GCS                  (1U << 3)
#define I2C_SR_ADRS                 (1U << 2)
#define I2C_SR_STO                  (1U << 1)
#define I2C_SR_STA                  (1U << 0)
#define I2C_TAR_RWD                 (1U << 10)

// SPI
// /////////////////////////////////////////////////////////////////////////////
typedef struct {
  __IO uint32_t CR0;            //!< 0x000          Control Register 0
  __IO uint32_t CR1;            //!< 0x004          Control Register 1
  __IO uint32_t IER;            //!< 0x008          Interrupt Enable Register
  __IO uint32_t CPR;            //!< 0x00C          Clock Prescaler Register
  __IO uint32_t DR;             //!< 0x010          Data Register
  __IO uint32_t SR;             //!< 0x014          Status Register
  __IO uint32_t FCR;            //!< 0x018          FIFO Control Register
  __IO uint32_t FSR;            //!< 0x01C          FIFO Status Register
  __IO uint32_t FTOCR;          //!< 0x020          FIFO Time Out Counter Register
} SPI_TypeDef;

#define SPI_CR0_GUADTEN             (1U << 7)
#define SPI_CR0_DUALEN              (1U << 6)
#define SPI_CR0_SSELC               (1U << 4)
#define SPI_CR0_SELOEN              (1U << 3)
#define SPI_CR0_SPIEN               (1U << 0)
#define SPI_CR1_MODE                (1U << 14)
#define SPI_CR1_SELM                (1U << 13)
#define SPI_CR1_FIRSTBIT            (1U << 12)
#define SPI_CR1_SELAP               (1U << 11)
#define SPI_CR1_FORMAT_MASK         (7U << 8)
#define SPI_CR1_FORMAT_MODE0        (0x1U << 8)
#define SPI_CR1_FORMAT_MODE1        (0x2U << 8)
#define SPI_CR1_FORMAT_MODE2        (0x6U << 8)
#define SPI_CR1_FORMAT_MODE3        (0x5U << 8)
#define SPI_IER_RXBNEIEN            (1U << 2)
#define SPI_IER_TXBEIEN             (1U << 0)
#define SPI_SR_RXBNE                (1U << 2)
#define SPI_SR_TXE                  (1U << 1)
#define SPI_SR_TXBE                 (1U << 0)
#define SPI_FCR_FIFOEN              (1U << 10)
#define SPI_FSR_TXFS_MASK           (0xfU << 0)
#define SPI_FSR_RXFS_MASK           (0xfU << 4)

// USART
// UART
// /////////////////////////////////////////////////////////////////////////////
typedef struct {
  union {
  __IO uint32_t RBR;            //!< 0x000          Receive Buffer Register
  __IO uint32_t TBR;            //!< 0x000          Transmit Holding Register
  __IO uint32_t DR;             //!< 0x000          Data Register
  };
  __IO uint32_t IER;            //!< 0x004          Interrupt Enable Register
  __IO uint32_t IIR;            //!< 0x008          Interrupt Identification Register/FIFO Control Register
  __IO uint32_t FCR;            //!< 0x00C          FIFO Control Register
  __IO uint32_t LCR;            //!< 0x010          Line Control Register
  // note: only available as USART
  __IO uint32_t MODCR;          //!< 0x014          Modem Control Register
  // end note
  __IO uint32_t LSR;            //!< 0x018          Line Status Register
  // note: only available as USART
  __IO uint32_t MODSR;          //!< 0x01C          Modem Status Register
  // end note
  __IO uint32_t TPR;            //!< 0x020          Timing Parameter Register
  __IO uint32_t MDR;            //!< 0x024          Mode Register
  // note: only available as USART
  __IO uint32_t IrDACR;         //!< 0x028          IrDA Control Register
  __IO uint32_t RS485CR;        //!< 0x02C          RS485 Control Register
  __IO uint32_t SYNCR;          //!< 0x030          Synchronous Control Register
  // end note
  __IO uint32_t FSR;            //!< 0x034          FIFO Status Register
  __IO uint32_t DLR;            //!< 0x038          Divisor Latch Register
       uint32_t RESERVED0;      //!< 0x03C          Reserved
  __IO uint32_t DEGTSTR;        //!< 0x040          Debug/Test Register
} USART_TypeDef;

#define USART_FCR_FME               (1 << 0)
#define USART_FCR_RFR               (1 << 1)
#define USART_FCR_TFR               (1 << 2)
#define USART_FCR_TFTL_MASK         (0x3 << 4)
#define USART_FCR_TFTL_0BYTE        (0x0 << 4)
#define USART_FCR_TFTL_2BYTE        (0x1 << 4)
#define USART_FCR_TFTL_4BYTE        (0x2 << 4)
#define USART_FCR_TFTL_8BYTE        (0x3 << 4)
#define USART_FCR_RFTL_MASK         (0x3 << 6)
#define USART_FCR_RFTL_1BYTE        (0x0 << 6)
#define USART_FCR_RFTL_4BYTE        (0x1 << 6)
#define USART_FCR_RFTL_8BYTE        (0x2 << 6)
#define USART_FCR_RFTL_14BYTE       (0x3 << 6)
#define USART_FCR_URTXEN            (1 << 8)
#define USART_FCR_URRXEN            (1 << 9)
#define USART_LCR_WLS_MASK          (0x3 << 0)
#define USART_LCR_WLS_7BIT          (0x0 << 0)
#define USART_LCR_WLS_8BIT          (0x1 << 0)
#define USART_LCR_WLS_9BIT          (0x2 << 0)
#define USART_LCR_NSB               (1 << 2)
#define USART_LCR_PBE               (1 << 3)
#define USART_LCR_EPE               (1 << 4)
#define USART_LCR_SPE               (1 << 5)
#define USART_LCR_BCB               (1 << 6)
#define USART_LSR_RFDR              (1 << 0)
#define USART_LSR_OEI               (1 << 1)
#define USART_LSR_PEI               (1 << 2)
#define USART_LSR_FEI               (1 << 3)
#define USART_LSR_BII               (1 << 4)
#define USART_LSR_TXFEMPT           (1 << 5)
#define USART_LSR_TXEMPT            (1 << 6)
#define USART_LSR_ERRRX             (1 << 7)
#define USART_LSR_RSADDEF           (1 << 8)
#define USART_MDR_MODE_MASK         (0x3 << 0)
#define USART_MDR_MODE_NORMAL       (0x0 << 0)
#define USART_MDR_MODE_IRDA         (0x1 << 0)
#define USART_MDR_MODE_RS485        (0x2 << 0)
#define USART_MDR_MODE_SYNCHRONOUS  (0x3 << 0)
#define USART_MDR_TRSM              (1 << 2)
#define USART_MDR_TXDMAEN           (1 << 4)
#define USART_MDR_RXDMAEN           (1 << 5)

// Smart Card Interface
// /////////////////////////////////////////////////////////////////////////////

// USB
// /////////////////////////////////////////////////////////////////////////////
typedef struct {
  __IO uint32_t CSR;            //!< 0x000          USB Control and Status Register
  __IO uint32_t IER;            //!< 0x004          USB Interrupt Enable Register
  __IO uint32_t ISR;            //!< 0x008          USB Interrupt Status Register
  __IO uint32_t FCR;            //!< 0x00C          USB Frame Count Register
  __IO uint32_t DEVAR;          //!< 0x010          USB Device Address Register
  struct {
    __IO uint32_t CSR;          //!< 0x014          USB Endpoint n Control and Status Register
    __IO uint32_t IER;          //!< 0x018          USB Endpoint n Interrupt Enable Register
    __IO uint32_t ISR;          //!< 0x01C          USB Endpoint n Interrupt Status Register
    __IO uint32_t TCR;          //!< 0x020          USB Endpoint n Transfer Count Register
    __IO uint32_t CFGR;         //!< 0x024          USB Endpoint n Configuration Register
  } EP[8];
} USB_TypeDef;

// USBCSR
#define USBCSR_FRES         (0x002)        // Force USB Reset Control
#define USBCSR_PDWN         (0x004)        // Power Down Mode Control
#define USBCSR_LPMODE       (0x008)        // Low-Power Mode Control
#define USBCSR_GENRSM       (0x020)        // Resume Request Generation Control
#define USBCSR_RXDP         (0x040)        // Received DP Line Status
#define USBCSR_RXDM         (0x080)        // Received DM Line Status
#define USBCSR_ADRSET       (0x100)        // Device Address Setting Control
#define USBCSR_SRAMRSTC     (0x200)        // USB SRAM Reset Condition
#define USBCSR_DPPUEN       (0x400)        // DP Pull Up Enable
#define USBCSR_DPWKEN       (0x800)        // DP Wake Up Enable

// USBIER
#define USBIER_UGIE         (0x0001)       // USB global Interrupt Enable
#define USBIER_SOFIE        (0x0002)       // Start Of Frame Interrupt Enable
#define USBIER_URSTIE       (0x0004)       // USB Reset Interrupt Enable
#define USBIER_RSMIE        (0x0008)       // Resume Interrupt Enable
#define USBIER_SUSPIE       (0x0010)       // Suspend Interrupt Enable
#define USBIER_ESOFIE       (0x0020)       // Expected Start Of Frame Enable
#define USBIER_EP0IE        (0x0100)       // Endpoint 0 Interrupt Enable
#define USBIER_EP1IE        (0x0200)       // Endpoint 1 Interrupt Enable
#define USBIER_EP2IE        (0x0400)       // Endpoint 2 Interrupt Enable
#define USBIER_EP3IE        (0x0800)       // Endpoint 3 Interrupt Enable
#define USBIER_EP4IE        (0x1000)       // Endpoint 4 Interrupt Enable
#define USBIER_EP5IE        (0x2000)       // Endpoint 5 Interrupt Enable
#define USBIER_EP6IE        (0x4000)       // Endpoint 6 Interrupt Enable
#define USBIER_EP7IE        (0x8000)       // Endpoint 7 Interrupt Enable

// USBISR
#define USBISR_SOFIF        (0x0002)       // Start Of Frame Interrupt Flag
#define USBISR_URSTIF       (0x0004)       // USB Reset Interrupt Flag
#define USBISR_RSMIF        (0x0008)       // Resume Interrupt Flag
#define USBISR_SUSPIF       (0x0010)       // Suspend Interrupt Flag
#define USBISR_ESOFIF       (0x0020)       // Expected Start Of Frame Interrupt
#define USBISR_EP0IF        (1U << 8)      // Endpoint 0 Interrupt Flag
#define USBISR_EP1IF        (1U << 9)      // Endpoint 1 Interrupt Flag
#define USBISR_EP2IF        (1U << 10)     // Endpoint 2 Interrupt Flag
#define USBISR_EP3IF        (1U << 11)     // Endpoint 3 Interrupt Flag
#define USBISR_EP4IF        (1U << 12)     // Endpoint 4 Interrupt Flag
#define USBISR_EP5IF        (1U << 13)     // Endpoint 5 Interrupt Flag
#define USBISR_EP6IF        (1U << 14)     // Endpoint 6 Interrupt Flag
#define USBISR_EP7IF        (1U << 15)     // Endpoint 7 Interrupt Flag
#define USBISR_EPnIF        (0xFF00)       // Endpoint Interrupt Mask

// USBFCR
#define USBFCR_FRNUM        (0x7FF)        // Frame Number
#define USBFCR_SOFLCK       (1U << 16)     // Start-of-Frame Lock Flag
#define USBFCR_LSOF         (0x3U << 17)   // Lost Start-of-Frame Number

// USBEPnCSR
#define USBEPnCSR_DTGTX     (0x01)         // Data Toggle Status, for IN transfer
#define USBEPnCSR_NAKTX     (0x02)         // NAK Status, for IN transfer
#define USBEPnCSR_STLTX     (0x04)         // STALL Status, for IN transfer
#define USBEPnCSR_DTGRX     (0x08)         // Data Toggle Status, for OUT transfer
#define USBEPnCSR_NAKRX     (0x10)         // NAK Status, for OUT transfer
#define USBEPnCSR_STLRX     (0x20)         // STALL Status, for OUT transfer

// USBEPnIER
#define USBEPnIER_OTRXIE    (0x001)        // OUT Token Received Interrupt Enable
#define USBEPnIER_ODRXIE    (0x002)        // OUT Data Received Interrupt Enable
#define USBEPnIER_ODOVIE    (0x004)        // OUT Data Buffer Overrun Interrupt Enable
#define USBEPnIER_ITRXIE    (0x008)        // IN Token Received Interrupt Enable
#define USBEPnIER_IDTXIE    (0x010)        // IN Data Transmitted Interrupt Enable
#define USBEPnIER_NAKIE     (0x020)        // NAK Transmitted Interrupt Enable
#define USBEPnIER_STLIE     (0x040)        // STALL Transmitted Interrupt Enable
#define USBEPnIER_UERIE     (0x080)        // USB Error Interrupt Enable
#define USBEPnIER_STRXIE    (0x100)        // SETUP Token Received Interrupt Enable
#define USBEPnIER_SDRXIE    (0x200)        // SETUP Data Received Interrupt Enable
#define USBEPnIER_SDERIE    (0x400)        // SETUP Data Error Interrupt Enable
#define USBEPnIER_ZLRXIE    (0x800)        // Zero Length Data Received Interrupt Enable

// USBEPnISR
#define USBEPnISR_OTRXIF    (0x001)        // OUT Token Received Interrupt Flag
#define USBEPnISR_ODRXIF    (0x002)        // OUT Data Received Interrupt Flag
#define USBEPnISR_ODOVIF    (0x004)        // OUT Data Buffer Overrun Interrupt Flag
#define USBEPnISR_ITRXIF    (0x008)        // IN Token Received Interrupt Flag
#define USBEPnISR_IDTXIF    (0x010)        // IN Data Transmitted Interrupt Flag
#define USBEPnISR_NAKIF     (0x020)        // NAK Transmitted Interrupt Flag
#define USBEPnISR_STLIF     (0x040)        // STALL Transmitted Interrupt Flag
#define USBEPnISR_UERIF     (0x080)        // USB Error Interrupt Flag
#define USBEPnISR_STRXIF    (0x100)        // SETUP Token Received Interrupt Flag
#define USBEPnISR_SDRXIF    (0x200)        // SETUP Data Received Interrupt Flag
#define USBEPnISR_SDERIF    (0x400)        // SETUP Data Error Interrupt Flag
#define USBEPnISR_ZLRXIF    (0x800)        // Zero Length Data Received Interrupt Flag

// USBEPnTCR
#define USBEPnTCR_TCNT      (0x1FF)        // Transfer Byte Count

// USBEPnCFGR
#define USBEPnCFGR_EPEN     (1U << 31)     // Endpoint Enable
#define USBEPnCFGR_EPTYPE   (1U << 29)     // Transfer Type
#define USBEPnCFGR_EPDIR    (1U << 28)     // Transfer Direction
#define USBEPnCFGR_EPADR    (0xFU << 24)   // Endpoint Address
#define USBEPnCFGR_EPLEN    (0x7FU << 10)  // Buffer Length
#define USBEPnCFGR_EPBUFA   (0x3FF)        // Endpoint Buffer Address

// Peripheral Direct Memory Access
// /////////////////////////////////////////////////////////////////////////////

// Extend Bus Interface
// /////////////////////////////////////////////////////////////////////////////

// Inter-IC Sound
// /////////////////////////////////////////////////////////////////////////////

// CRC
// /////////////////////////////////////////////////////////////////////////////

#endif // HT32F165x_REG_H
