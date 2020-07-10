#ifndef INC_GPIO_H_
#define INC_GPIO_H_

/* GPIO Driver for the STM32F407xx MCU */

#include <stdint.h>
#include "gpio_defs.h"
#include "interrupt.h"

typedef struct {
    GPIO_Port_t port;
    uint8_t pin;
} GPIO_Handle_t;

typedef struct {
    GPIO_Mode_t PinMode;              /* Specifies IO direction mode */
    GPIO_OutputType_t PinOutputType;  /* Configure pin as open drain or push-pull */
    GPIO_OutputSpeed_t PinSpeed;      /* Configure slew rate of pin */
    GPIO_ResMode_t PinResistor;       /* Configure the internal pull-up/pull-down resistors */
    GPIO_AltFn_t PinAltFunction;      /* Selects altfn if PinMode is set to altfn mode*/
} GPIO_PinConfig_t;

typedef struct {
    InterruptEdge_t edge_trigger;
    InterruptPriority_t priority;
    InterruptType_t type;
} GPIO_InterruptSettings_t;

typedef void (*GPIO_Callback_t)(GPIO_Handle_t *handle, void *context);

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
void GPIO_RegisterInterrupt(GPIO_Handle_t *handle,
                            GPIO_InterruptSettings_t *it_settings,
                            GPIO_Callback_t callback,
                            void *context);

#endif /* INC_GPIO_H_ */
