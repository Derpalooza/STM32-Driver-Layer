#ifndef INC_INTERRUPT_H_
#define INC_INTERRUPT_H_

/* Interrupt Driver for the STM32F407xx MCU */

#include <stdint.h>

typedef enum {
    INTERRUPT_TYPE_INTERRUPT = 0,
    INTERRUPT_TYPE_EVENT,
    NUM_INTERRUPT_TYPES
} InterruptType_t;

typedef enum {
    INTERRUPT_EDGE_RISING = 0,
    INTERRUPT_EDGE_FALLING,
    INTERRUPT_EDGE_RISING_FALLING,
    NUM_INTERRUPT_EDGES
} InterruptEdge_t;

typedef enum {
    INTERRUPT_NVIC_PRI0 = 0,
    INTERRUPT_NVIC_PRI1,
    INTERRUPT_NVIC_PRI2,
    INTERRUPT_NVIC_PRI3,
    INTERRUPT_NVIC_PRI4,
    INTERRUPT_NVIC_PRI5,
    INTERRUPT_NVIC_PRI6,
    INTERRUPT_NVIC_PRI7,
    INTERRUPT_NVIC_PRI8,
    INTERRUPT_NVIC_PRI9,
    INTERRUPT_NVIC_PRI10,
    INTERRUPT_NVIC_PRI11,
    INTERRUPT_NVIC_PRI12,
    INTERRUPT_NVIC_PRI13,
    INTERRUPT_NVIC_PRI14,
    INTERRUPT_NVIC_PRI15,
} InterruptPriority_t;

// Initialize interrupt
void Interrupt_Init(void);

// Enable or Disable an NVIC interrupt channel
void Interrupt_NVIC_Config(uint8_t irq_channel, uint8_t enable);

// Set the priority of an NVIC interrupt channel
void Interrupt_NVIC_SetPriority(uint8_t irq_channel, uint8_t irq_priority);

// Enable or disable an EXTI interrupt line
void Interrupt_EXTI_Config(uint8_t irq_line, InterruptEdge_t edge, InterruptType_t type,
                           uint8_t enable);

// Reads the pending bit of the interrupt line
uint8_t Interrupt_EXTI_GetPending(uint8_t irq_line);

// Clears the pending bit of the interrupt line
void Interrupt_EXTI_ClearPending(uint8_t irq_line);

// Triggers an interrupt in software
void Interrupt_EXTI_Trigger(uint8_t irq_line);


#endif /* INC_INTERRUPT_H_ */
