#ifndef INC_GPIO_H_
#define INC_GPIO_H_

/* GPIO Driver for the STM32F407xx MCU */

#include <stdint.h>
#include "gpio_defs.h"

typedef struct {
    GPIO_Port_t port;
    uint8_t pin;
} GPIO_Handle_t;

typedef struct {
    uint8_t PinMode;        /* Specifies IO direction mode */
    uint8_t PinOutputType;  /* Configure pin as open drain or push-pull */
    uint8_t PinSpeed;       /* Configure slew rate of pin */
    uint8_t PinResistor;    /* Configure the internal pull-up/pull-down resistors */
    uint8_t PinAltFunction; /* Selects alt fn. Ignored if PinMode is not set to alt fn*/
} GPIO_PinConfig_t;

/* Initializes the given pin with the provided settings */
void GPIO_Init(GPIO_Handle_t *handle, GPIO_PinConfig_t *config);

/* Resets the port registers to their default values */
void GPIO_Reset(GPIO_Port_t port);

/* Read data from input pin */
uint8_t GPIO_ReadPin(GPIO_Handle_t *handle);
uint16_t GPIO_ReadPort(GPIO_Port_t port);

/* Write data to output pin */
void GPIO_WritePin(GPIO_Handle_t *handle, uint8_t state);
void GPIO_WritePort(GPIO_Port_t port, uint16_t state);
void GPIO_TogglePin(GPIO_Handle_t *handle);

/* Interrupt configuration */
//void GPIO_ConfigureInterrupt(void);
//void GPIO_RegisterInterrupt(void);


#endif /* INC_GPIO_H_ */
