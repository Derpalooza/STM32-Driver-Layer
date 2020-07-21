#ifndef INC_SPI_H_
#define INC_SPI_H_

/* SPI Driver for the STM32F407xx MCU */
/* GPIO Pins must be configured before SPI port is initialized */

#include "stm32f407xx.h"
#include "gpio.h"

typedef enum {
    SPI_PORT_1 = 0,
    SPI_PORT_2,
    SPI_PORT_3,
    NUM_SPI_PORTS
} SPI_Port_t;

typedef enum {
    SPI_MODE_MASTER = 0,
    SPI_MODE_SLAVE,
    NUM_SPI_MODES
} SPI_Mode_t;

typedef enum {
    SPI_PRESCALER_2 = 0,
    SPI_PRESCALER_4,
    SPI_PRESCALER_8,
    SPI_PRESCALER_16,
    SPI_PRESCALER_32,
    SPI_PRESCALER_64,
    SPI_PRESCALER_128,
    SPI_PRESCALER_256,
    NUM_SPI_PRESCALERS
} SPI_Prescaler_t;

typedef enum {
    SPI_DATA_FORMAT_8_BIT = 0,
    SPI_DATA_FORMAT_16_BIT,
    NUM_DATA_FORMATS
} SPI_DataFormat_t;

typedef enum {
    SPI_FRAME_FORMAT_MSB_FIRST = 0,
    SPI_FRAME_FORMAT_LSB_FIRST,
    NUM_SPI_FORMATS
} SPI_FrameFormat_t;

typedef enum {
    SPI_FULL_DUPLEX_MODE = 0,
    SPI_SIMPLEX_RX_MODE,
    SPI_SIMPLEX_TX_MODE,
    SPI_HALF_DUPLEX_TX_MODE,
    SPI_HALF_DUPLEX_RX_MODE,
    NUM_SPI_DIRS,
} SPI_Direction_t;

typedef enum {
    SPI_SSM_DISABLE = 0,
    SPI_SSM_ENABLE,
    NUM_SPI_SSM
} SPI_SSM_t;

typedef struct {
    SPI_Mode_t mode;
    SPI_Direction_t direction;
    SPI_DataFormat_t data_format;
    SPI_FrameFormat_t frame_format;
    SPI_Prescaler_t prescaler;
    SPI_SSM_t SSM;
    uint8_t CPHA;
    uint8_t CPOL;
} SPI_Settings_t;

typedef void(*SPI_Callback_t)(SPI_Port_t port, void *context);

// Initializes SPI Port to given settings, and initializes given GPIO Pins
int SPI_Init(SPI_Port_t port, SPI_Settings_t *settings);

// Resets the SPI Port to its default setting
int SPI_Reset(SPI_Port_t port);

// Enable SPI Port. Call before starting SPI communication
int SPI_Enable(SPI_Port_t port);

// Disable SPI Port. Call before starting SPI communication
int SPI_Disable(SPI_Port_t port);

// Blocking Send and Receive calls. |len| refers to the number of data elements in
// the buffer, not the number of bytes.
int SPI_TransmitData(SPI_Port_t port, uint8_t *tx_data, uint32_t len);
int SPI_ReceiveData(SPI_Port_t port, uint8_t *rx_data, uint32_t len);

// Register interrupt
int SPI_RegisterCallback(SPI_Port_t port,
                         SPI_Callback_t tx_callback,
                         SPI_Callback_t rx_callback,
                         void *context);
#endif /* INC_SPI_H_ */
