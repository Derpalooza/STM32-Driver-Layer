#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

/*
 * STM32F407xx device specific header file
 */

#include <stdint.h>

#define __IO    volatile

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
    __IO uint32_t AHB1ENR;      /* RCC AHB1 peripheral clock register */
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

/* SPI Peripheral Register Structure */
typedef struct {} SPI_t;

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

#define RCC                     ((RCC_t*)RCC_BASE)

/* Clock enable macros for GPIO Peripheral */
#define GPIO_CLK_ENABLE(port)   (RCC->AHB1ENR |= (1 << (port)))
#define GPIO_CLK_DISABLE(port)  (RCC->AHB1ENR &= ~(1 << (port)))
#define GPIOA_CLK_ENABLE()      (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_CLK_ENABLE()      (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_CLK_ENABLE()      (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_CLK_ENABLE()      (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_CLK_ENABLE()      (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_CLK_ENABLE()      (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_CLK_ENABLE()      (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_CLK_ENABLE()      (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_CLK_ENABLE()      (RCC->AHB1ENR |= (1 << 8))

#define GPIOA_CLK_DISABLE()     (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_CLK_DISABLE()     (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_CLK_DISABLE()     (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_CLK_DISABLE()     (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_CLK_DISABLE()     (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_CLK_DISABLE()     (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_CLK_DISABLE()     (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_CLK_DISABLE()     (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_CLK_DISABLE()     (RCC->AHB1ENR &= ~(1 << 8))

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

/* Generic Macros */
#define ENABLE      1
#define DISABLE     0
#define SET         ENABLE
#define RESET       DISABLE

#endif /* INC_STM32F407XX_H_ */
