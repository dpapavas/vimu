#include "mk20dx128.h"
#include "util.h"

static volatile uint32_t ticks;
static uint32_t state = 0;

__attribute__((interrupt ("IRQ"))) void systick_isr()
{
    ticks += 1;
}

uint64_t cycles()
{
    uint32_t v, t, t_1;

    t = ticks;
    v = SYST_CVR;

    /* An overflow (tick) might have occured between the previous two
     * instructions. Check for it and if it did happen reload the
     * current systick value (since the last number of ticks read is
     * bound to still be accurate) . */
    
    t_1 = ticks;
    
    if (t_1 != t) {
        v = SYST_CVR;
        t = t_1;
    }
    
    return (((uint64_t)t) << 24) | (uint64_t)(SYST_RVR - v);
}

void delay_ms(uint64_t n)
{
    uint64_t t_0;
    
    t_0 = cycles();
    while (cycles() - t_0 < cycles_in_ms(n));
}

void delay_us(uint64_t n)
{
    uint64_t t_0;
    
    t_0 = cycles();
    while (cycles() - t_0 < cycles_in_us(n));
}

uint32_t lcg_rand()
{
    return (state = ((state * 1103515245) + 12345) & 0x7fffffff);
}
