#include "spi.h"
#include "stm32f407xx.h"

static void clock_control(SPI_Port_t port, uint8_t enable) {
    switch(port) {
    case SPI_PORT_1:
        if (enable == ENABLE) {
            SPI1_CLK_ENABLE();
        } else {
            SPI1_CLK_DISABLE();
        }
        break;
    case SPI_PORT_2:
        if (enable == ENABLE) {
            SPI2_CLK_ENABLE();
        } else {
            SPI2_CLK_DISABLE();
        }
        break;
    case SPI_PORT_3:
        if (enable == ENABLE) {
            SPI3_CLK_ENABLE();
        } else {
            SPI3_CLK_ENABLE();
        }
        break;
    default:
        break;
    }
}

int SPI_Init(SPI_Port_t port, SPI_Settings_t *settings, SPI_PinConfig_t *pins) {
    // Check parameters
    if (settings == NULL || pins == NULL || (port > NUM_SPI_PORTS)) {
        return 1;
    }

    // Enable the clock for the port
    clock_control(port, ENABLE);

    return 0;
}

int SPI_Reset(SPI_Port_t port) {
    if (port > NUM_SPI_PORTS) {
        return 1;
    }

    // Disable SPI Port clock
    clock_control(port, DISABLE);

    return 0;
}

int SPI_TransmitData(SPI_Port_t port, uint8_t *tx_data, uint32_t len) {
    return 1;
}

int SPI_ReceiveData(SPI_Port_t port, uint8_t *rx_data, uint32_t len) {
    return 1;
}

// Register interrupt
int SPI_RegisterCallback(SPI_Port_t port,
                          SPI_Callback_t tx_callback,
                          SPI_Callback_t rx_callback,
                          void *context) {
    return 1;
}

/************** SPI Interrupt Handlers **************/

void SPI1_IRQHandler() {}

void SPI2_IRQHandler() {}

void SPI3_IRQHandler() {}

