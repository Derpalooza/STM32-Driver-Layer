#include "gpio.h"
#include "stm32f407xx.h"
#include "interrupt.h"

/* Mapping between port registers */
static GPIO_t* GPIO_PortMap[] = {
    GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI
};

static uint8_t IRQ_NumberMap[NUM_GPIO_PINS] = {
    EXTI0_IRQ,
    EXTI1_IRQ,
    EXTI2_IRQ,
    EXTI3_IRQ,
    EXTI4_IRQ,
    EXTI9_5_IRQ,
    EXTI9_5_IRQ,
    EXTI9_5_IRQ,
    EXTI9_5_IRQ,
    EXTI9_5_IRQ,
    EXTI15_10_IRQ,
    EXTI15_10_IRQ,
    EXTI15_10_IRQ,
    EXTI15_10_IRQ,
    EXTI15_10_IRQ,
    EXTI15_10_IRQ
};

/* GPIO Interrupt containers */
typedef struct {
    GPIO_Handle_t handle;
    GPIO_Callback_t callback;
    void *context;
} InterruptData;

static InterruptData interrupts[NUM_GPIO_PINS] = { 0 };

static void GPIO_ClockControl(GPIO_Port_t port, uint8_t status) {
    // TODO: Add check for invalid params
    if (status == ENABLE) {
        GPIO_CLK_ENABLE(port);
    } else {
        GPIO_CLK_DISABLE(port);
    }
}

void GPIO_Init(GPIO_Handle_t *handle, GPIO_PinConfig_t *config) {
    if (handle == NULL || config == NULL) {
        return;
    }

    uint8_t port = handle->port;
    uint8_t pin = handle->pin;

    // Enable port clock
    GPIO_ClockControl(port, ENABLE);

    // Obtain port pointer
    GPIO_t *GPIOx = GPIO_PortMap[port];

    // As some of the registers use multiple bits per data pin, a multiplier
    // must be applied to the bit shifts based on how many bits are used per pin

    // Set Mode Register
    uint8_t mode = config->PinMode & 0x03;
    GPIOx->MODER &= ~(0x3 << (2 * pin));
    GPIOx->MODER |= (mode << (2 * pin));

    // Set Alternate function Registers if pin is in alternate function mode
    if (config->PinMode == GPIO_MODE_ALTFN) {
        uint8_t altfn = config->PinAltFunction & 0x0F;

        if (pin > 7) {
            // Subtract pin number by 8 as an offset for the register
            uint8_t new_pin = pin - 8;

            GPIOx->AFRH &= ~(0xF << (4 * new_pin));
            GPIOx->AFRH |= (altfn << (4 * new_pin));
        } else {
            GPIOx->AFRL &= ~(0xF << (4 * pin));
            GPIOx->AFRL |= (altfn << (4 * pin));
        }
    }

    // Set Output type
    if (mode == GPIO_MODE_OUTPUT) {
        uint8_t output_type = config->PinOutputType & 0x03;
        GPIOx->OTYPER &= ~(1 << pin);
        GPIOx->OTYPER |= (output_type << pin);
    }

    // Set Speed register
    uint8_t speed = config->PinSpeed & 0x03;
    GPIOx->OSPEEDR &= ~(0x3 << (2 * pin));
    GPIOx->OSPEEDR |= (speed << (2 * pin));

    // Set resistor settings
    uint8_t res = config->PinResistor & 0x03;
    GPIOx->PUPDR &= ~(0x3 << (2 * pin));
    GPIOx->PUPDR |= (res << (2 * pin));
}

void GPIO_Reset(uint8_t port) {
    // Toggle Reset register
    RCC->AHB1RSTR |= (1 << port);
    RCC->AHB1RSTR &= ~(1 << port);

    for (uint8_t i = 0; i < NUM_GPIO_PINS; i++) {
        interrupts[i].callback = NULL;
        interrupts[i].context = NULL;
        interrupts[i].handle = (GPIO_Handle_t){ 0 };
    }

}

uint8_t GPIO_ReadPin(GPIO_Handle_t *handle) {
    if (handle == NULL) {
        return 0;
    }

    uint8_t port = handle->port;
    uint8_t pin = handle->pin;

    uint8_t port_data = (uint8_t)GPIO_ReadPort(port);

    // Shift desired bit to LSB position, and clear everything but LSB
    port_data = (port_data >> pin) & 0x1;
    return port_data;
}

uint16_t GPIO_ReadPort(GPIO_Port_t port) {
    GPIO_t *GPIOx = GPIO_PortMap[port];

    // Clear the upper 16 bits
    uint16_t port_data = (uint16_t)(GPIOx->IDR & 0xFFFF);

    return port_data;
}

void GPIO_WritePin(GPIO_Handle_t *handle, uint8_t state) {
    if (handle == NULL) {
        return;
    }

    // Since we are setting individual bits, we use the BSRR register for atomic writes
    uint8_t port = handle->port;
    uint8_t pin = handle->pin;

    GPIO_t *GPIOx = GPIO_PortMap[port];

    // In the BSRR register, the lower 16 bits set a specific pin on the port, while
    // the upper 16 bits reset a pin on the port. Writing 0 does nothing

    uint32_t bitmask = (1 << pin);

    if (!state) {
        bitmask <<= 16;
    }

    GPIOx->BSSR = bitmask;

    return;
}

void GPIO_WritePort(GPIO_Port_t port, uint16_t state) {
    GPIO_t *GPIOx = GPIO_PortMap[port];

    // There is no need to worry about atomicity here, as we are writing to the whole port
    GPIOx->ODR = state;
}

void GPIO_TogglePin(GPIO_Handle_t *handle) {
    if (handle == NULL) {
        return;
    }

    uint8_t port = handle->port;
    uint8_t pin = handle->pin;

    GPIO_t *GPIOx = GPIO_PortMap[port];

    GPIOx->ODR ^= (1 << pin);
}

void GPIO_RegisterInterrupt(GPIO_Handle_t *handle,
                            GPIO_InterruptSettings_t *it_settings,
                            GPIO_Callback_t callback,
                            void *context) {
    if (handle == NULL || it_settings == NULL || context == NULL || callback == NULL) {
        return;
    }

    uint8_t port = handle->port;
    uint8_t pin = handle->pin;

    // Configure SYSCFG registers to select the correct GPIO port
    uint8_t reg_idx = port / 4;
    uint8_t reg_pos = port % 4;

    // Clear previous value and set the EXTI selection bits
    SYSCFG->EXTICR[reg_idx] &= ~(0xF << (4 * reg_pos));
    SYSCFG->EXTICR[reg_idx] |= (port << (4 * reg_pos));

    // Record interrupt settings
    interrupts[pin].handle = (GPIO_Handle_t){ port, pin };
    interrupts[pin].callback = callback;
    interrupts[pin].context = context;

    // Configure EXTI settings
    Interrupt_EXTI_Config(pin, it_settings->edge_trigger, it_settings->type, ENABLE);

    // Configure NVIC settings
    Interrupt_NVIC_Config(IRQ_NumberMap[pin], ENABLE);
    Interrupt_NVIC_SetPriority(IRQ_NumberMap[pin], it_settings->priority);
}

/***************** Interrupt Handler code *****************/

void GPIO_IRQHandler(uint8_t pin) {

    // Check pending bit
    if (Interrupt_EXTI_GetPending(pin)) {

        // If an interrupt is pending for this line, start the callback
        GPIO_Handle_t *handle = (GPIO_Handle_t*)&interrupts[pin].handle;
        void *context = interrupts[pin].context;

        if (interrupts[pin].callback != NULL) {
            interrupts[pin].callback(handle, context);
        }

        // Clear the pending bit
        Interrupt_EXTI_ClearPending(pin);
    }
}

void EXTI0_IRQHandler() {
    GPIO_IRQHandler(0);
}

void EXTI1_IRQHandler() {
    GPIO_IRQHandler(1);
}

void EXTI2_IRQHandler() {
    GPIO_IRQHandler(2);
}

void EXTI3_IRQHandler() {
    GPIO_IRQHandler(3);
}

void EXTI4_IRQHandler() {
    GPIO_IRQHandler(4);
}

void EXTI9_5_IRQHandler() {
    GPIO_IRQHandler(5);
    GPIO_IRQHandler(6);
    GPIO_IRQHandler(7);
    GPIO_IRQHandler(8);
    GPIO_IRQHandler(9);
}

void EXTI15_10_IRQHandler() {
    GPIO_IRQHandler(10);
    GPIO_IRQHandler(11);
    GPIO_IRQHandler(12);
    GPIO_IRQHandler(13);
    GPIO_IRQHandler(14);
    GPIO_IRQHandler(15);
}

