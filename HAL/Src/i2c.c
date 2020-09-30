#include <i2c.h>
#include <stm32f407xx.h>

// Peripheral clock frequency in MHz
// NOTE: We are using the internal clock (HSI) is used with no prescalers, meaning
// that the following values were calculated assuming a clock frequency of 16MHz
#define APB1_CLOCK_FREQ         16

// CCR Macros calculated using information from Section 27.6.8 of the reference manual
#define CCR_STANDARD_MODE       0x50
#define CCR_FAST_DUTY_2         0x0D
#define CCR_FAST_DUTY_16_9      0x02

// TRISE values calculated based on information from section 27.6.9 of the reference manual
#define TRISE_STANDARD_MODE     0x11
#define TRISE_FAST_MODE         0x05

// I2C Transmit/Receive Macros
#define I2C_TRANSMIT_MODE       0x0
#define I2C_RECEIVE_MODE        0x1

// Register Bitmasks
#define I2C_ENABLE_MASK         (1 << I2C_CR1_ENABLE)
#define I2C_START_MASK          (1 << I2C_CR1_START)
#define I2C_STOP_MASK           (1 << I2C_CR1_STOP)
#define I2C_ACK_MASK            (1 << I2C_CR1_ACK)
#define I2C_FASTMODE_MASK       (1 << I2C_CCR_FASTMODE)
#define I2C_DUTY_MASK           (1 << I2C_CCR_DUTY)
#define I2C_ADDRMODE_MASK       (1 << I2C_OAR1_ADDRMODE)
#define I2C_BUSY_MASK           (1 << I2C_SR2_BSY)
#define I2C_SB_MASK             (1 << I2C_SR1_SB)
#define I2C_ADDR_MASK           (1 << I2C_SR1_ADDR)
#define I2C_TXE_MASK            (1 << I2C_SR1_TXE)
#define I2C_RXNE_MASK           (1 << I2C_SR1_RXNE)
#define I2C_BTF_MASK            (1 << I2C_SR1_BTF)
#define I2C_NACK_MASK           (1 << I2C_SR1_AF)

static I2C_t* I2C_PortMap[NUM_I2C_PORTS] = { I2C1, I2C2, I2C3 };

static void clock_control(I2C_Port_t port) {
    switch (port) {
    case I2C_PORT_1:
        I2C1_CLK_ENABLE();
        return;
    case I2C_PORT_2:
        I2C2_CLK_ENABLE();
        return;
    case I2C_PORT_3:
        I2C3_CLK_ENABLE();
        return;
    default:
        return;
    }
}

static uint8_t get_flag_status(I2C_t* I2Cx, uint32_t flag) {
    if (I2Cx->SR1 && flag) {
        return SET;
    }
    return RESET;
}

static uint8_t address_phase(I2C_t* I2Cx, uint8_t addr, bool read) {
    // The master sends the slave address in the first 7 bits
    uint8_t temp_reg = (addr << 1);

    // Set LSB based on R/W mode
    if (read == I2C_RECEIVE_MODE) {
        temp_reg |= 0x1;
    } else {
        temp_reg &= ~0x1;
    }

    // Write to DR register to send the address
    I2Cx->DR = temp_reg;

    // Wait until the address has been sent
    while (!get_flag_status(I2Cx, I2C_ADDR_MASK)) {
        // If a NACK is detected, return 0 so the function doesn't block
        if (get_flag_status(I2Cx, I2C_NACK_MASK)) {
            return 0;
        }
    }

    // Read the SR2 register to clear the ADDR flag
    temp_reg = I2Cx->SR2;

    return 1;
}

void i2c_init(I2C_Port_t port, I2C_Settings_t *settings) {
    // Check for valid parameters
    if (port >= NUM_I2C_PORTS || settings->mode >= NUM_I2C_MODES ||
        settings->duty >= NUM_I2C_FM_DUTY) {
        return;
    }

    // Enable port clock
    clock_control(port);

    // Obtain Peripheral pointer
    I2C_t* I2Cx = I2C_PortMap[port];

    // Program the peripheral input clock in CR2 Register to generate the correct timings
    // TODO: Make function to calculate this value
    I2Cx->CR2 &= ~(0x3F);
    I2Cx->CR2 |= APB1_CLOCK_FREQ;

    // Clear CCR bits
    I2Cx->CCR &= ~(0xFFF);

    // Clear TRISE bits
    I2Cx->TRISE &= ~(0x3F);

    // Configure clock control register
    if (settings->mode == I2C_STANDARD_MODE) {
        // Set as standard mode
        I2Cx->CCR &= ~I2C_FASTMODE_MASK;

        // Configure clock control value
        I2Cx->CCR |= CCR_STANDARD_MODE;

        // Configure TRISE register
        I2Cx->TRISE |= TRISE_STANDARD_MODE;
    } else {
        // Set as fast mode
        I2Cx->CCR |= I2C_FASTMODE_MASK;

        // Set clock control value based on duty cycle
        if (settings->duty == I2C_FM_DUTY_2) {
            I2Cx->CCR &= ~I2C_DUTY_MASK;
            I2Cx->CCR |= CCR_FAST_DUTY_2;
        } else {
            I2Cx->CCR |= I2C_DUTY_MASK;
            I2Cx->CCR |= CCR_FAST_DUTY_16_9;
        }

        // Configure TRISE register
        I2Cx->TRISE |= TRISE_FAST_MODE;
    }

    // Bit 14 of OAR1 should always be kept at 1 by software according to reference manual
    I2Cx->OAR1 |= (1 << 14);

    // Enable the peripheral
    I2Cx->CR1 |= I2C_ENABLE_MASK;

    // Enable ACKing
    I2Cx->CR1 |= I2C_ACK_MASK;
}

void i2c_start(I2C_Port_t port) {
    I2C_t* I2Cx = I2C_PortMap[port];

    // Wait until the BUSY flag is cleared
    while ((I2Cx->SR2 && I2C_BUSY_MASK)) {}

    // Set the START bit
    I2Cx->CR1 |= I2C_START_MASK;

    // Wait until the start condition has been sent
    while (!get_flag_status(I2Cx, I2C_SB_MASK)) {}
}

int i2c_read(I2C_Port_t port, uint8_t addr, uint8_t *data, uint8_t len) {
    I2C_t* I2Cx = I2C_PortMap[port];
    uint8_t ack_status;

    // Send slave address over the bus
    ack_status = address_phase(I2Cx, addr, I2C_RECEIVE_MODE);

    if (!ack_status) {
        return 0;
    }

    // Begin receiving data
    for (uint32_t i = 0; i < len; i++) {
        // To end the reception, a NACK must be sent for the final byte. Thus,
        // after reading the second-last byte, the ACK bit must be set to 0
        if (i == (len-1)) {
            I2Cx->CR1 &= ~I2C_ACK_MASK;
        }

        // Wait until the master has received a byte
        while (!get_flag_status(I2Cx, I2C_RXNE_MASK)) {}

        // Store data in rx buffer
        data[i] = I2Cx->DR;
    }

    // Restore ACK after NACK has been sent
    I2Cx->CR1 |= I2C_ACK_MASK;

    return 1;
}

int i2c_write(I2C_Port_t port, uint8_t addr, uint8_t *data, uint8_t len) {
    I2C_t* I2Cx = I2C_PortMap[port];
    uint8_t ack_status;

    // Send slave address over the bus
    ack_status = address_phase(I2Cx, addr, I2C_TRANSMIT_MODE);

    if (!ack_status) {
        return 0;
    }

    // Begin transmitting data
    for (uint32_t i = 0; i < len; i++) {
        // Wait until the data register is empty
        while (!get_flag_status(I2Cx, I2C_TXE_MASK)) {}

        // Write byte to the data register
        I2Cx->DR = data[i];
    }

    // Wait until the byte transfer is complete before a stop request can be made
    while (!get_flag_status(I2Cx, I2C_TXE_MASK)) {}
    while (!get_flag_status(I2Cx, I2C_BTF_MASK)) {}

    return 1;
}

void i2c_stop(I2C_Port_t port) {
    I2C_t* I2Cx = I2C_PortMap[port];

    // Generate stop condition by setting STOP bit
    I2Cx->CR1 |= I2C_STOP_MASK;
}
