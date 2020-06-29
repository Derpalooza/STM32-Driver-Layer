#include "stm32f407xx.h"
#include "gpio.h"

/* Mapping between port registers */
static GPIO_t* GPIO_PortMap[] = {
    GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG, GPIOH, GPIOI
};

static void GPIO_ClockControl(GPIO_Port_t port, uint8_t status) {
    // TODO: Add check for invalid params
    if (status == ENABLE) {
        GPIO_CLK_ENABLE(port);
    } else {
        GPIO_CLK_DISABLE(port);
    }
}

void GPIO_Init(GPIO_Handle_t *handle, GPIO_PinConfig_t *config) {
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
    uint8_t output_type = config->PinOutputType & 0x03;
    GPIOx->OTYPER &= ~(1 << pin);
    GPIOx->OTYPER |= (output_type << pin);

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
    RCC->AHB1RSTR |= (1 << port);
    RCC->AHB1RSTR &= ~(1 << port);

}

uint8_t GPIO_ReadPin(GPIO_Handle_t *handle) {
    uint8_t port = handle->port;
    uint8_t pin = handle->pin;

    uint8_t port_data = GPIO_ReadPort(port);

    // Shift desired bit to LSB position, and clear everything but LSB
    port_data = (port_data >> pin) & 0x1;
    return port_data;
}

uint16_t GPIO_ReadPort(GPIO_Port_t port) {
    GPIO_t *GPIOx = GPIO_PortMap[port];

    // Clear the upper 16 bits
    uint16_t port_data = (GPIOx->IDR & 0xFFFF);

    return port_data;
}

void GPIO_WritePin(GPIO_Handle_t *handle, uint8_t state) {
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

}
