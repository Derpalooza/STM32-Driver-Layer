#ifndef INC_I2C_H_
#define INC_I2C_H_

/* I2C Driver for the STM32F407xx MCU */
/* GPIO pins must be initialized before using this interface */

#include <stm32f407xx.h>
#include "gpio.h"

typedef enum {
    I2C_PORT_1 = 0,
    I2C_PORT_2,
    I2C_PORT_3,
    NUM_I2C_PORTS
} I2C_Port_t;

typedef enum {
    I2C_STANDARD_MODE = 0,
    I2C_FAST_MODE,
    NUM_I2C_MODES
} I2C_Mode_t;

typedef enum {
    I2C_FM_DUTY_2 = 0,
    I2C_FM_DUTY_16_9,
    NUM_I2C_FM_DUTY
} I2C_FMDutyCycle_t;

typedef struct {
    I2C_Mode_t mode;
    I2C_FMDutyCycle_t duty;
} I2C_Settings_t;

// Initialize I2C Port
void i2c_init(I2C_Port_t port, I2C_Settings_t *settings);

// Generate start condition on the I2C bus. This must be called before
// using i2c_read() or i2c_write()
void i2c_start(I2C_Port_t port);

// Read or write a specified number of bytes over the bus. Will generate
// a repeated start if called in succession. Returns 0 in the case of failure
int i2c_read(I2C_Port_t port, uint8_t addr, uint8_t *data, uint8_t len);
int i2c_write(I2C_Port_t port, uint8_t addr, uint8_t *data, uint8_t len);

// Generate stop condition on the I2C bus. This must be called after I2C communications
// have finished in order to release the bus
void i2c_stop(I2C_Port_t port);

#endif /* INC_I2C_H_ */
