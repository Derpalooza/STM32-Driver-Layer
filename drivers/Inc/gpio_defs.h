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
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT,
    GPIO_MODE_ALTFN,
    GPIO_MODE_ANALOG,
} GPIO_Mode_t;

typedef enum {
    GPIO_OUTPUT_PUSH_PULL = 0,
    GPIO_OUTPUT_OPEN_DRAIN,
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
} GPIO_AltFn_t;

#endif /* INC_GPIO_DEFS_H_ */
