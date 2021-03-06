#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

/*
 * STM32F407xx device specific header file
 */

#include <stdint.h>

#define __I     volatile const
#define __O     volatile
#define __IO    volatile

/************** Processor Specific **************/
/* NVIC Interrupt set-enable register addresses*/
#define NVIC_ISER_BASE          ((__IO uint32_t*)0xE000E100U)
#define NVIC_ISER_LEN           8

/* NVIC Interrupt clear-enable register addresses */
#define NVIC_ICER_BASE          ((__IO uint32_t*)0xE000E180U)
#define NVIC_ICER_LEN           8

/* NVIC Interrupt priority register addresses*/
#define NVIC_PR_BASE            ((__IO uint32_t*)0xE000E400U)
#define NVIC_PR_LEN             60

/**************  Microcontroller Specific **************/
/* Flash and SRAM base addresses */
#define FLASH_BASE_ADDRESS      0x08000000U
#define SRAM1_BASE_ADDRESS      0x20000000U
#define SRAM2_BASE_ADDRESS      0x2001C000U
#define SYSMEM_BASE_ADDRESS     0x1FFF0000U

/* APBx and AHBx Bus Domain base addresses */
#define APB1_BASE               0x40000000U
#define APB2_BASE               0x40010000U
#define AHB1_BASE               0x40020000U
#define AHB2_BASE               0x50000000U

/* APB1 Bus Peripheral Base addresses */
#define SPI2_BASE               (APB1_BASE + 0x3800U)
#define SPI3_BASE               (APB1_BASE + 0x3C00U)
#define USART2_BASE             (APB1_BASE + 0x4400U)
#define USART3_BASE             (APB1_BASE + 0x4800U)
#define UART4_BASE              (APB1_BASE + 0x4C00U)
#define UART5_BASE              (APB1_BASE + 0x5000U)
#define I2C1_BASE               (APB1_BASE + 0x5400U)
#define I2C2_BASE               (APB1_BASE + 0x5800U)
#define I2C3_BASE               (APB1_BASE + 0x5C00U)

/* APB2 Bus Peripheral Base addresses */
#define USART1_BASE             (APB2_BASE + 0x1000U)
#define USART6_BASE             (APB2_BASE + 0x1400U)
#define SPI1_BASE               (APB2_BASE + 0x3000U)
#define SYSCFG_BASE             (APB2_BASE + 0x3800U)
#define EXTI_BASE               (APB2_BASE + 0x3C00U)

/* AHB1 Bus Peripheral Base addresses */
#define GPIOA_BASE              (AHB1_BASE + 0x0000U)
#define GPIOB_BASE              (AHB1_BASE + 0x0400U)
#define GPIOC_BASE              (AHB1_BASE + 0x0800U)
#define GPIOD_BASE              (AHB1_BASE + 0x0C00U)
#define GPIOE_BASE              (AHB1_BASE + 0x1000U)
#define GPIOF_BASE              (AHB1_BASE + 0x1400U)
#define GPIOG_BASE              (AHB1_BASE + 0x1800U)
#define GPIOH_BASE              (AHB1_BASE + 0x1C00U)
#define GPIOI_BASE              (AHB1_BASE + 0x2000U)
#define RCC_BASE                (AHB1_BASE + 0x3800U)

/* GPIO Peripheral Register Structure */

typedef struct {
    __IO uint32_t MODER;         /* GPIO Mode Register */
    __IO uint32_t OTYPER;        /* GPIO Output Type Register */
    __IO uint32_t OSPEEDR;       /* GPIO Output Speed Register */
    __IO uint32_t PUPDR;         /* GPIO Pull-Up/Pull-Down Register*/
    __IO uint32_t IDR;           /* GPIO Input Data Register */
    __IO uint32_t ODR;           /* GPIO Output Data Register */
    __IO uint32_t BSSR;          /* GPIO Bit Set/Reset Register */
    __IO uint32_t LCKR;          /* GPIO Port Configuration Lock Register */
    __IO uint32_t AFRL;          /* GPIO Alternate Function Low Register */
    __IO uint32_t AFRH;          /* GPIO Alternate Function High Register */
} GPIO_t;

/* RCC Peripheral Register Structure */
typedef struct {
    __IO uint32_t CR;           /* RCC clock control register */
    __IO uint32_t PLLCFGR;      /* RCC PLL configuration register */
    __IO uint32_t CFGR;         /* RCC clock configuration register */
    __IO uint32_t CIR;          /* RCC clock interrupt register */
    __IO uint32_t AHB1RSTR;     /* RCC AHB1 peripheral reset register */
    __IO uint32_t AHB2RSTR;     /* RCC AHB2 peripheral reset register */
    __IO uint32_t AHB3RSTR;     /* RCC AHB3 peripheral reset register */
    uint32_t RESERVED0;
    __IO uint32_t APB1RSTR;     /* RCC APB1 peripheral reset register */
    __IO uint32_t APB2RSTR;     /* RCC APB2 peripheral reset register */
    uint32_t RESERVED1;
    uint32_t RESERVED2;
    __IO uint32_t AHB1ENR;      /* RCC AHB1 peripheral clock enable register */
    __IO uint32_t AHB2ENR;      /* RCC AHB2 peripheral clock enable register */
    __IO uint32_t AHB3ENR;      /* RCC AHB3 peripheral clock enable register */
    uint32_t RESERVED3;
    __IO uint32_t APB1ENR;      /* RCC APB1 peripheral clock enable register */
    __IO uint32_t APB2ENR;      /* RCC APB2 peripheral clock enable register */
    uint32_t RESERVED4;
    uint32_t RESERVED5;
    __IO uint32_t AHB1LPENR;    /* RCC AHB1 peripheral low power clock enable register */
    __IO uint32_t AHB2LPENR;    /* RCC AHB2 peripheral low power clock enable register */
    __IO uint32_t AHB3LPENR;    /* RCC AHB3 peripheral low power clock enable register */
    uint32_t RESERVED6;
    __IO uint32_t APB1LPENR;    /* RCC APB1 peripheral low power clock enable register */
    __IO uint32_t APB2LPENR;    /* RCC APB2 peripheral low power clock enable register */
    uint32_t RESERVED7;
    uint32_t RESERVED8;
    __IO uint32_t BDCR;         /* RCC Backup domain control register */
    __IO uint32_t CSR;          /* RCC clock control & status register */
    uint32_t RESERVED9;
    uint32_t RESERVED10;
    __IO uint32_t SSCGR;        /* RCC spread spectrum clock generation register */
    __IO uint32_t PLLI2SCFGR;   /* RCC PLLI2S configuration register */
    __IO uint32_t PLLSAICFGR;   /* RCC PLL configuration register */
    __IO uint32_t DCKCFGR;      /* RCC Dedicated Clock Configuration Register */
} RCC_t;

/* EXTI Peripheral Register Structure */
typedef struct {
    __IO uint32_t IMR;          /* EXTI Interrupt mask register */
    __IO uint32_t EMR;          /* EXTI Event mask register */
    __IO uint32_t RTSR;         /* EXTI Rising trigger selection register */
    __IO uint32_t FTSR;         /* EXTI Falling trigger selection register */
    __IO uint32_t SWIER;        /* EXTI Software interrupt event register */
    __IO uint32_t PR;           /* EXTI Pending register */
} EXTI_t;

/* SYSCFG Peripheral Register Structure */
typedef struct {
    __IO uint32_t MEMRMP;       /* SYSCFG Memory remap register */
    __IO uint32_t PMC;          /* SYSCFG Peripheral mode configuration register */
    __IO uint32_t EXTICR[4];    /* SYSCFG External interrupt configuration registers */
    uint32_t RESERVED[2];
    __IO uint32_t CMPCR;        /* SYSCFG Compensation control register */
} SYSCFG_t;

/* SPI Peripheral Register Structure */
typedef struct {
    __IO uint32_t CR1;          /* SPI Control Register 1 */
    __IO uint32_t CR2;          /* SPI Control Register 2 */
    __IO uint32_t SR;           /* SPI Status Register */
    __IO uint32_t DR;           /* SPI Data Register */
    __IO uint32_t CRCPR;        /* SPI CRC Polynomial Register */
    __IO uint32_t RXCRCR;       /* SPI RX CRC Register */
    __IO uint32_t TXCRCR;       /* SPI TX CRC Register */
    __IO uint32_t I2SCFGR;      /* SPI/I2S Configuration Register*/
    __IO uint32_t I2SPR;        /* SPI/I2S Prescaler Register */
} SPI_t;

/* I2C Peripheral Register Structure */
typedef struct {} I2C_t;

/* UART Peripheral Register Structure */
typedef struct {} UART_t;

/* USART Peripheral Register Structure */
typedef struct {} USART_t;

/* Peripheral Definitions */
#define GPIOA                   ((GPIO_t*)GPIOA_BASE)
#define GPIOB                   ((GPIO_t*)GPIOB_BASE)
#define GPIOC                   ((GPIO_t*)GPIOC_BASE)
#define GPIOD                   ((GPIO_t*)GPIOD_BASE)
#define GPIOE                   ((GPIO_t*)GPIOE_BASE)
#define GPIOF                   ((GPIO_t*)GPIOF_BASE)
#define GPIOG                   ((GPIO_t*)GPIOG_BASE)
#define GPIOH                   ((GPIO_t*)GPIOH_BASE)
#define GPIOI                   ((GPIO_t*)GPIOI_BASE)

#define SPI1                    ((SPI_t*)SPI1_BASE)
#define SPI2                    ((SPI_t*)SPI2_BASE)
#define SPI3                    ((SPI_t*)SPI3_BASE)

#define RCC                     ((RCC_t*)RCC_BASE)
#define EXTI                    ((EXTI_t*)EXTI_BASE)
#define SYSCFG                  ((SYSCFG_t*)SYSCFG_BASE)

/* RCC Register Bit Positions */
#define RCC_APB1RSTR_TIM2       0
#define RCC_APB1RSTR_TIM3       1
#define RCC_APB1RSTR_TIM4       2
#define RCC_APB1RSTR_TIM5       3
#define RCC_APB1RSTR_TIM6       4
#define RCC_APB1RSTR_TIM7       5
#define RCC_APB1RSTR_TIM12      6
#define RCC_APB1RSTR_TIM13      7
#define RCC_APB1RSTR_TIM14      8
#define RCC_APB1RSTR_WWDG       11
#define RCC_APB1RSTR_SPI2       14
#define RCC_APB1RSTR_SPI3       15
#define RCC_APB1RSTR_USART2     17
#define RCC_APB1RSTR_USART3     18
#define RCC_APB1RSTR_UART4      19
#define RCC_APB1RSTR_UART5      20
#define RCC_APB1RSTR_I2C1       21
#define RCC_APB1RSTR_I2C2       22
#define RCC_APB1RSTR_I2C3       23
#define RCC_APB1RSTR_CAN1       25
#define RCC_APB1RSTR_CAN2       26
#define RCC_APB1RSTR_PWR        28
#define RCC_APB1RSTR_DAC        29

#define RCC_APB2RSTR_TIM1       0
#define RCC_APB2RSTR_TIM8       1
#define RCC_APB2RSTR_USART1     4
#define RCC_APB2RSTR_USART6     5
#define RCC_APB2RSTR_ADC        8
#define RCC_APB2RSTR_SDIO       11
#define RCC_APB2RSTR_SPI1       12
#define RCC_APB2RSTR_SYSCFG     14
#define RCC_APB2RSTR_TIM9       16
#define RCC_APB2RSTR_TIM10      17
#define RCC_APB2RSTR_TIM11      18

/* Clock enable macros for GPIO Peripheral */
#define GPIO_CLK_ENABLE(port)   (RCC->AHB1ENR |= (1 << (port)))
#define GPIO_CLK_DISABLE(port)  (RCC->AHB1ENR &= ~(1 << (port)))

/* Clock enable macros for I2C Peripheral */
#define I2C1_CLK_ENABLE()       (RCC->APB1ENR |= (1 << 21))
#define I2C2_CLK_ENABLE()       (RCC->APB1ENR |= (1 << 22))
#define I2C3_CLK_ENABLE()       (RCC->APB1ENR |= (1 << 23))

#define I2C1_CLK_DISABLE()      (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_CLK_DISABLE()      (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_CLK_DISABLE()      (RCC->APB1ENR &= ~(1 << 23))

/* Clock enable macros for SPI Peripheral */
#define SPI1_CLK_ENABLE()       (RCC->APB2ENR |= (1 << 12))
#define SPI2_CLK_ENABLE()       (RCC->APB1ENR |= (1 << 14))
#define SPI3_CLK_ENABLE()       (RCC->APB1ENR |= (1 << 15))

#define SPI1_CLK_DISABLE()       (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_CLK_DISABLE()       (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_CLK_DISABLE()       (RCC->APB1ENR &= ~(1 << 15))

/* Clock enable macros for UART Peripheral */
#define UART4_CLK_ENABLE()      (RCC->APB1ENR |= (1 << 19))
#define UART5_CLK_ENABLE()      (RCC->APB1ENR |= (1 << 20))

#define UART4_CLK_DISABLE()     (RCC->APB1ENR &= ~(1 << 19))
#define UART5_CLK_DISABLE()     (RCC->APB1ENR &= ~(1 << 20))

/* Clock enable macros for USART Peripheral */
#define USART1_CLK_ENABLE()     (RCC->APB2ENR |= (1 << 4))
#define USART2_CLK_ENABLE()     (RCC->APB1ENR |= (1 << 17))
#define USART3_CLK_ENABLE()     (RCC->APB1ENR |= (1 << 18))
#define USART6_CLK_ENABLE()     (RCC->APB2ENR |= (1 << 5))

#define USART1_CLK_DISABLE()    (RCC->APB2ENR &= ~(1 << 4))
#define USART2_CLK_DISABLE()    (RCC->APB1ENR &= ~(1 << 17))
#define USART3_CLK_DISABLE()    (RCC->APB1ENR &= ~(1 << 18))
#define USART6_CLK_DISABLE()    (RCC->APB2ENR &= ~(1 << 5))

/* Clock enable macros for SYSCFG Peripheral */
#define SYSCFG_CLK_ENABLE()     (RCC->APB2ENR |= (1 << 14))
#define SYSCFG_CLK_DISABLE()    (RCC->APB2ENR &= ~(1 << 14))

/* IRQ Numbers */
#define EXTI0_IRQ               6
#define EXTI1_IRQ               7
#define EXTI2_IRQ               8
#define EXTI3_IRQ               9
#define EXTI4_IRQ               10
#define EXTI9_5_IRQ             23
#define SPI1_IRQ                35
#define SPI2_IRQ                36
#define EXTI15_10_IRQ           40
#define SPI3_IRQ                51
#define NUM_IRQS                82

/* Generic Macros */
#define ENABLE      1
#define DISABLE     0
#define SET         ENABLE
#define RESET       DISABLE

#ifndef NULL
#define NULL        0
#endif

/* SPI Register Bit Positions */
#define SPI_CR1_BIDIMODE        15
#define SPI_CR1_BIDIOE          14
#define SPI_CR1_CRCEN           13
#define SPI_CR1_CRCNEXT         12
#define SPI_CR1_DFF             11
#define SPI_CR1_RXONLY          10
#define SPI_CR1_SSM             9
#define SPI_CR1_SSI             8
#define SPI_CR1_LSBFIRST        7
#define SPI_CR1_SPE             6
#define SPI_CR1_BR              3
#define SPI_CR1_MSTR            2
#define SPI_CR1_CPOL            1
#define SPI_CR1_CPHA            0

#define SPI_CR2_TXEIE           7
#define SPI_CR2_RXNEIE          6
#define SPI_CR2_ERRIE           5
#define SPI_CR2_FRF             4
#define SPI_CR2_SSOE            2
#define SPI_CR2_TXDMAEN         1
#define SPI_CR2_RXDMAEN         0

#define SPI_SR_FRE              8
#define SPI_SR_BSY              7
#define SPI_SR_OVR              6
#define SPI_SR_MODF             5
#define SPI_SR_CRC_ERR          4
#define SPI_SR_TXE              1
#define SPI_SR_RXNE             0


#endif /* INC_STM32F407XX_H_ */
