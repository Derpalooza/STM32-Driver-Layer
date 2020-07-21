#include "spi.h"

#include "stm32f407xx.h"

// Flag Bitmasks
#define SPI_TXE_FLAG        (1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG       (1 << SPI_SR_RXNE)
#define SPI_BSY_FLAG       (1 << SPI_SR_BSY)

// Register Bitmasks
#define SPI_ENABLE_MASK     (1 << SPI_CR1_SPE)
#define SPI_DFF_MASK        (1 << SPI_CR1_DFF)
#define SPI_MSTR_MASK       (1 << SPI_CR1_MSTR)
#define SPI_BIDIMODE_MASK   (1 << SPI_CR1_BIDIMODE)
#define SPI_RXONLY_MASK     (1 << SPI_CR1_RXONLY)
#define SPI_BIDIOE_MASK     (1 << SPI_CR1_BIDIOE)
#define SPI_DFF_MASK        (1 << SPI_CR1_DFF)
#define SPI_LSBFIRST_MASK   (1 << SPI_CR1_LSBFIRST)
#define SPI_CPOL_MASK       (1 << SPI_CR1_CPOL)
#define SPI_CPHA_MASK       (1 << SPI_CR1_CPHA)
#define SPI_SSM_MASK        (1 << SPI_CR1_SSM)
#define SPI_SSI_MASK        (1 << SPI_CR1_SSI)

#define SPI_SSOE_MASK       (1 << SPI_CR2_SSOE)

// RCC Reset masks
#define SPI1_RESET_MASK     (1 << RCC_APB2RSTR_SPI1)
#define SPI2_RESET_MASK     (1 << RCC_APB1RSTR_SPI2)
#define SPI3_RESET_MASK     (1 << RCC_APB1RSTR_SPI3)

static SPI_t* SPI_PortMap[] = {
    [SPI_PORT_1] = SPI1,
    [SPI_PORT_2] = SPI2,
    [SPI_PORT_3] = SPI3
};

// Check the state of the given flag for the given SPI port
static uint8_t SPI_GetFlagStatus(SPI_Port_t port, uint32_t flag) {
    SPI_t *SPIx = SPI_PortMap[port];

    if (SPIx->SR & flag) {
        return SET;
    }

    return RESET;
}

// Enables or disables the peripheral clock
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

int SPI_Init(SPI_Port_t port, SPI_Settings_t *settings) {
    // Check parameters
    if (settings == NULL || (port > NUM_SPI_PORTS)) {
        return 1;
    }

    // Get SPI Port
    SPI_t* SPIx = SPI_PortMap[port];

    // Enable the clock for the port
    clock_control(port, ENABLE);

    // Configure SPI Mode
    switch (settings->mode) {
    case SPI_MODE_MASTER:
        SPIx->CR1 |= SPI_MSTR_MASK;
        break;
    case SPI_MODE_SLAVE:
        SPIx->CR1 &= ~SPI_MSTR_MASK;
        break;
    default:
        return 1;
    }

    // Configure SPI Direction
    switch (settings->direction) {
    case SPI_FULL_DUPLEX_MODE:
        // Clear BIDIMODE bit
        SPIx->CR1 &= ~SPI_BIDIMODE_MASK;
        // Clear RXONLY bit
        SPIx->CR1 &= ~SPI_RXONLY_MASK;
        break;
    case SPI_SIMPLEX_RX_MODE:
        // Clear BIDIMODE bit
        SPIx->CR1 &= ~SPI_BIDIMODE_MASK;
        // Set RXONLY bit
        SPIx->CR1 |= SPI_RXONLY_MASK;
        break;
    case SPI_HALF_DUPLEX_TX_MODE:
        // Set BIDIMODE bit
        SPIx->CR1 |= SPI_BIDIMODE_MASK;
        // Set BIDIOE bit
        SPIx->CR1 |= SPI_BIDIOE_MASK;
        break;
    case SPI_HALF_DUPLEX_RX_MODE:
        // Set BIDIMODE bit
        SPIx->CR1 |= SPI_BIDIMODE_MASK;
        // Clear BIDIOE bit
        SPIx->CR1 &= ~SPI_BIDIOE_MASK;
        break;
    default:
        return 1;
    }

    // Configure SPI Data format
    switch (settings->data_format) {
    case SPI_DATA_FORMAT_8_BIT:
        // Clear DFF bit
        SPIx->CR1 &= ~SPI_DFF_MASK;
        break;
    case SPI_DATA_FORMAT_16_BIT:
        // Set DFF bit
        SPIx->CR1 |= SPI_DFF_MASK;
        break;
    default:
        return 1;
    }

    // Configure SPI Frame Format
    switch (settings->frame_format) {
    case SPI_FRAME_FORMAT_MSB_FIRST:
        // Clear LSBFIRST bit
        SPIx->CR1 &= ~SPI_LSBFIRST_MASK;
        break;
    case SPI_FRAME_FORMAT_LSB_FIRST:
        // Set LSBFIRST bit
        SPIx->CR1 |= SPI_LSBFIRST_MASK;
        break;
    default:
        return 1;
    }

    // Configure SPI Prescaler
    uint8_t baudrate = settings->prescaler & 0x7;
    SPIx->CR1 &= ~(0x7 << SPI_CR1_BR);
    SPIx->CR1 |= (baudrate << SPI_CR1_BR);

    // Configure Software Slave Management
    switch (settings->SSM) {
    case SPI_SSM_DISABLE:
        SPIx->CR1 &= ~SPI_SSM_MASK;

        // Enable output on NSS pin. NSS will automatically be pulled to low
        // whenever SPI port is enabled
        if (settings->mode == SPI_MODE_MASTER) {
            SPIx->CR2 |= SPI_SSOE_MASK;
        }
        break;
    case SPI_SSM_ENABLE:
        SPIx->CR1 |= SPI_SSM_MASK;

        // Drives NSS signal to high to avoid Master Fault Error
        if (settings->mode == SPI_MODE_MASTER) {
            SPIx->CR1 |= SPI_SSI_MASK;
        }
        break;
    default:
        return 1;
    }

    // Configure CPOL and CPHA
    if (!settings->CPOL) {
        SPIx->CR1 &= ~SPI_CPOL_MASK;
    } else {
        SPIx->CR1 |= SPI_CPOL_MASK;
    }

    if (!settings->CPHA) {
        SPIx->CR1 &= ~SPI_CPHA_MASK;
    } else {
        SPIx->CR1 |= SPI_CPHA_MASK;
    }

    return 0;
}

int SPI_Reset(SPI_Port_t port) {
    // Toggle Reset Register
    switch (port) {
    case SPI_PORT_1:
        RCC->APB2RSTR |= SPI1_RESET_MASK;
        RCC->APB2RSTR &= ~SPI1_RESET_MASK;
    case SPI_PORT_2:
        RCC->APB1RSTR |= SPI2_RESET_MASK;
        RCC->APB1RSTR &= !SPI1_RESET_MASK;
    case SPI_PORT_3:
        RCC->APB1RSTR |= SPI3_RESET_MASK;
        RCC->APB1RSTR &= ~SPI3_RESET_MASK;
    default:
        return 1;
    }

    return 0;
}

int SPI_Enable(SPI_Port_t port) {
    SPI_t *SPIx = SPI_PortMap[port];

    // Enable SPI Peripheral
    SPIx->CR1 |= SPI_ENABLE_MASK;
}

int SPI_Disable(SPI_Port_t port) {
    SPI_t *SPIx = SPI_PortMap[port];

    // Wait until SPI port is not busy
    while (SPI_GetFlagStatus(port, SPI_BSY_FLAG) == SET);

    // Disable SPI Peripheral
    SPIx->CR1 &= ~SPI_ENABLE_MASK;
}

int SPI_TransmitData(SPI_Port_t port, uint8_t *tx_data, uint32_t len) {
    if (tx_data == NULL || len <= 0 || port > NUM_SPI_PORTS) {
        // Invalid Parameters
        return 1;
    }
    // Get SPI Port pointer
    SPI_t *SPIx = SPI_PortMap[port];

    // Check if Data Format is 8 or 16 bit
    uint8_t dff;
    if (SPIx->CR1 & SPI_DFF_MASK) {
        dff = SPI_DATA_FORMAT_16_BIT;
    } else {
        dff = SPI_DATA_FORMAT_8_BIT;
    }

    for (uint32_t i = 0; i < len; i++) {
        // Wait until the TX buffer is empty and ready to be loaded with new data
        while (SPI_GetFlagStatus(port, SPI_TXE_FLAG) == RESET);

        // Write to the data register
        if (dff == SPI_DATA_FORMAT_8_BIT) {
            SPIx->DR = tx_data[i];

            // Ensure register is read to avoid overrun
            while (SPI_GetFlagStatus(port, SPI_RXNE_FLAG) == RESET);
            uint16_t temp = SPIx->DR;
        }
        else {
            // Account for little-endianness when recreating the 16-bit
            // value from tx_data
            SPIx->DR = (uint16_t)tx_data[i];

            // Make sure i increments by two per iteration to get the index
            // of the next 16 bit element
            i++;
        }
    }

    return 0;
}

int SPI_ReceiveData(SPI_Port_t port, uint8_t *rx_data, uint32_t len) {
    SPI_t *SPIx = SPI_PortMap[port];

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

