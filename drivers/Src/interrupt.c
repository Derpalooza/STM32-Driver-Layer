#include "interrupt.h"
#include "stm32f407xx.h"

#define NUM_EXTI_LINES  22

void Interrupt_Init(void) {
    // Enable SYSCFG clock
    SYSCFG_CLK_ENABLE();
}

void Interrupt_NVIC_Config(uint8_t irq_channel, uint8_t enable) {
    // Locate the proper register offset and bit position
    uint8_t iser_idx = irq_channel / 32;
    uint8_t iser_pos = irq_channel % 32;

    // Write to register
    if (enable == ENABLE) {
        NVIC_ISER_BASE[iser_idx] |= (1 << iser_pos);
    } else {
        NVIC_ICER_BASE[iser_idx] |= (1 << iser_pos);
        return;
    }
}

void Interrupt_NVIC_SetPriority(uint8_t irq_channel, uint8_t irq_priority) {
    // Locate the proper register offset and bit position
    uint8_t pr_idx = irq_channel / 4;
    uint8_t pr_pos = irq_channel % 4;

    // Processor only implements upper 4 bits of each priority field, so
    // we must shift up by four bits
    irq_priority <<= 4;

    // Write to the priority register
    NVIC_PR_BASE[pr_idx] &= ~(0xF << (8 * pr_pos));
    NVIC_PR_BASE[pr_idx] |= irq_priority << (8 * pr_pos);
}

void Interrupt_EXTI_Config(uint8_t irq_line, InterruptEdge_t edge, InterruptType_t type,
                           uint8_t enable) {
    // Check for invalid arguments
    if ((irq_line > NUM_EXTI_LINES) || (edge >= NUM_INTERRUPT_EDGES) ||
        (type > NUM_INTERRUPT_TYPES)) {
        return;
    }

    // Based on the type, configure the mask for the interrupt or event mask register
    if (type == INTERRUPT_TYPE_INTERRUPT) {
        if (enable == ENABLE) {
            EXTI->IMR |= (1 << irq_line);
        } else {
            EXTI->IMR &= ~(1 << irq_line);
        }
    } else if (INTERRUPT_TYPE_EVENT) {
        if (enable == ENABLE) {
            EXTI->EMR |= (1 << irq_line);
        } else {
            EXTI->EMR &= ~(1 << irq_line);
        }
    }

    // Configure the edge trigger
    if (edge == INTERRUPT_EDGE_RISING) {
        EXTI->RTSR |= (1 << irq_line);
        EXTI->FTSR &= ~(1 << irq_line);
    } else if (edge == INTERRUPT_EDGE_FALLING) {
        EXTI->FTSR |= (1 << irq_line);
        EXTI->RTSR &= ~(1 << irq_line);
    } else if (edge == INTERRUPT_EDGE_RISING_FALLING) {
        EXTI->RTSR |= (1 << irq_line);
        EXTI->FTSR |= (1 << irq_line);
    }
}

uint8_t Interrupt_EXTI_GetPending(uint8_t irq_line) {
    uint8_t pending_bit = (EXTI->PR >> irq_line) & 0x1;

    return pending_bit;
}

void Interrupt_EXTI_ClearPending(uint8_t irq_line) {
    if (irq_line > NUM_EXTI_LINES) {
        return;
    }

    // Pending bit is cleared by programming it to one
    EXTI->PR |= (1 << irq_line);
}

void Interrupt_EXTI_Trigger(uint8_t irq_line) {
    if (irq_line > NUM_EXTI_LINES) {
        return;
    }

    EXTI->SWIER |= (1 << irq_line);
}
