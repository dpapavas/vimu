#ifndef _UTIL_H_
#define _UTIL_H_

#include "usbserial.h"
#define assert(cond)                                            \
    if(!(cond)) {                                               \
        disable_all_interrupts();                               \
                                                                \
        if (usbserial_is_rts())  {                              \
            enable_interrupt(35);                               \
            prioritize_interrupt(35, 0);                        \
            usbserial_trace("assertion failed: %s.\n", #cond);  \
        }                                                       \
                                                                \
        while(1) {                                              \
            delay_ms(50);                                       \
            /* toggle_led(); */                                       \
        }                                                       \
    }

#define cycles_in_ms(t) ((t) * (F_CPU / 1000))
#define cycles_in_us(t) ((t) * (F_CPU / 1000000))

uint64_t cycles();
void delay_ms(uint64_t n);
void delay_us(uint64_t n);

#define set_led_color(c) {                      \
    if (c & (1 << 0)) {                         \
        GPIOD_PSOR = (1ul << 6);                \
    } else {                                    \
        GPIOD_PCOR = (1ul << 6);                \
    }                                           \
                                                \
    if (c & (1 << 1)) {                         \
        GPIOC_PSOR = (1ul << 2);                \
    } else {                                    \
        GPIOC_PCOR = (1ul << 2);                \
    }

#define toggle_led() GPIOC_PTOR = (1ul << 1)
#define turn_on_led() GPIOC_PCOR = (1ul << 1)
#define turn_off_led() GPIOC_PSOR = (1ul << 1)

#define LCG_RAND_MAX 2147483647
uint32_t lcg_rand();


#endif
