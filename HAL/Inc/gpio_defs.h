#ifndef INC_GPIO_DEFS_H_
#define INC_GPIO_DEFS_H_

typedef enum {
    GPIO_PORT_A = 0,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D,
    GPIO_PORT_E,
    GPIO_PORT_F,
    GPIO_PORT_G,
    GPIO_PORT_H,
    GPIO_PORT_I,
    NUM_GPIO_PORTS
} GPIO_Port_t;

typedef enum {
    GPIO_PIN_0 = 0,
    GPIO_PIN_1,
    GPIO_PIN_2,
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_5,
    GPIO_PIN_6,
    GPIO_PIN_7,
    GPIO_PIN_8,
    GPIO_PIN_9,
    GPIO_PIN_10,
    GPIO_PIN_11,
    GPIO_PIN_12,
    GPIO_PIN_13,
    GPIO_PIN_14,
    GPIO_PIN_15,
    NUM_GPIO_PINS
} GPIO_Pin_t;

typedef enum {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT,
    GPIO_MODE_ALTFN,
    GPIO_MODE_ANALOG,
    NUM_GPIO_MODES
} GPIO_Mode_t;

typedef enum {
    GPIO_OUTPUT_PUSH_PULL = 0,
    GPIO_OUTPUT_OPEN_DRAIN,
    NUM_GPIO_OUTPUT_TYPES
} GPIO_OutputType_t;

typedef enum {
    GPIO_SPEED_LOW = 0,
    GPIO_SPEED_MEDIUM,
    GPIO_SPEED_HIGH,
    GPIO_SPEED_VERY_HIGH,
} GPIO_OutputSpeed_t;

typedef enum {
    GPIO_RES_NONE = 0,
    GPIO_RES_PULL_UP,
    GPIO_RES_PULL_DOWN,
    NUM_GPIO_RES
} GPIO_ResMode_t;

typedef enum {
    GPIO_ALTFN_0 = 0,
    GPIO_ALTFN_1,
    GPIO_ALTFN_2,
    GPIO_ALTFN_3,
    GPIO_ALTFN_4,
    GPIO_ALTFN_5,
    GPIO_ALTFN_6,
    GPIO_ALTFN_7,
    GPIO_ALTFN_8,
    GPIO_ALTFN_9,
    GPIO_ALTFN_10,
    GPIO_ALTFN_11,
    GPIO_ALTFN_12,
    GPIO_ALTFN_13,
    GPIO_ALTFN_14,
    GPIO_ALTFN_15,
    NUM_GPIO_ALTFNS
} GPIO_AltFn_t;

#endif /* INC_GPIO_DEFS_H_ */
