#ifndef _UTIL_H_
#define _UTIL_H_

#ifndef USBSERIAL_ASSERT
#define assert(cond)                                            \
    if(!(cond)) {                                               \
        while(1) {                                              \
            delay_ms(50);                                       \
            toggle_led();                                       \
        }                                                       \
    }
#else
#include "usbserial.h"
#define assert usbserial_assert
#endif

#define cycles_in_ms(t) ((t) * (F_CPU / 1000))
#define cycles_in_us(t) ((t) * (F_CPU / 1000000))

uint64_t cycles();
void delay_ms(uint64_t n);
void delay_us(uint64_t n);

#define toggle_led() GPIOC_PTOR = (1ul << 5)
#define turn_on_led() GPIOC_PSOR = (1ul << 5)
#define turn_off_led() GPIOC_PCOR = (1ul << 5)

#define LCG_RAND_MAX 2147483647
uint32_t lcg_rand();


#endif
