#include <errno.h>
#include <limits.h>

#include "mk20dx128.h"
#include "usbcommon.h"
#include "usbserial.h"
#include "util.h"
#include "i2c.h"

extern void __libc_init_array (void);
extern void __startup(void);
extern int main (void);

static __attribute__((naked, used)) void diagnose_fault (unsigned long *sp)
{
    int i;

    if (usbserial_is_rts()) {
        prioritize_interrupt(35, 0);

        usbserial_printf("Hard fault\n"
                         "CFSR: \t%8x\n"
                         "HFSR: \t%8x\n",
                         SCB_CFSR, SCB_HFSR);

        if (SCB_CFSR & SCB_CFSR_MMARVALID) {
            usbserial_printf("MMAR: \t%8x\n", SCB_MMAR);
        }

        if (SCB_CFSR & SCB_CFSR_BFARVALID) {
            usbserial_printf("BFAR: \t%8x\n", SCB_BFAR);
        }

        usbserial_printf("r0: \t%8x\n"
                         "r1: \t%8x\n"
                         "r2: \t%8x\n"
                         "r3: \t%8x\n"
                         "r12: \t%8x\n"
                         "lr: \t%8x\n"
                         "pc: \t%8x\n"
                         "psr: \t%8x\n",
                         sp[0], sp[1], sp[2], sp[3],
                         sp[4], sp[5], sp[6], sp[7]);
    }

    for (i = 0 ; i < 20 ; i += 1) {
        delay_ms(150);
        toggle_led();
    }

    asm("bkpt");
}

static __attribute__((naked, noreturn, interrupt ("IRQ"))) void fault_isr(void)
{
    /* Determine if processor uses the process or main stack by
     * checking bit 2 of the LR register and pass the relevant stack
     * pointer to the diagnostic function. */

    __asm__ volatile(
        "movs r0, #4       \n\t"
        "mov r1, lr        \n\t"
        "tst r0, r1        \n\t"
        "beq 1f            \n\t"
        "mrs r0, psp       \n\t"
        "bl diagnose_fault \n\t"
        "1:                \n\t"
        "mrs r0, msp       \n\t"
        "bl diagnose_fault \n\t"
        );
}

static __attribute__((noreturn, interrupt ("IRQ"))) void unused_isr(void)
{
    assert(0);
    while (1);
}

void nmi_isr(void)              __attribute__ ((weak, alias("unused_isr")));
void hard_fault_isr(void)       __attribute__ ((weak, alias("fault_isr")));
void memmanage_fault_isr(void)  __attribute__ ((weak, alias("unused_isr")));
void bus_fault_isr(void)        __attribute__ ((weak, alias("unused_isr")));
void usage_fault_isr(void)      __attribute__ ((weak, alias("unused_isr")));
void svcall_isr(void)           __attribute__ ((weak, alias("unused_isr")));
void debugmonitor_isr(void)     __attribute__ ((weak, alias("unused_isr")));
void pendablesrvreq_isr(void)   __attribute__ ((weak, alias("unused_isr")));
void systick_isr(void)          __attribute__ ((weak, alias("unused_isr")));

void dma_ch0_isr(void)          __attribute__ ((weak, alias("unused_isr")));
void dma_ch1_isr(void)          __attribute__ ((weak, alias("unused_isr")));
void dma_ch2_isr(void)          __attribute__ ((weak, alias("unused_isr")));
void dma_ch3_isr(void)          __attribute__ ((weak, alias("unused_isr")));
void dma_error_isr(void)        __attribute__ ((weak, alias("unused_isr")));
void flash_cmd_isr(void)        __attribute__ ((weak, alias("unused_isr")));
void flash_error_isr(void)      __attribute__ ((weak, alias("unused_isr")));
void low_voltage_isr(void)      __attribute__ ((weak, alias("unused_isr")));
void wakeup_isr(void)           __attribute__ ((weak, alias("unused_isr")));
void watchdog_isr(void)         __attribute__ ((weak, alias("unused_isr")));
void i2c0_isr(void)             __attribute__ ((weak, alias("unused_isr")));
void spi0_isr(void)             __attribute__ ((weak, alias("unused_isr")));
void i2s0_tx_isr(void)          __attribute__ ((weak, alias("unused_isr")));
void i2s0_rx_isr(void)          __attribute__ ((weak, alias("unused_isr")));
void uart0_lon_isr(void)        __attribute__ ((weak, alias("unused_isr")));
void uart0_status_isr(void)     __attribute__ ((weak, alias("unused_isr")));
void uart0_error_isr(void)      __attribute__ ((weak, alias("unused_isr")));
void uart1_status_isr(void)     __attribute__ ((weak, alias("unused_isr")));
void uart1_error_isr(void)      __attribute__ ((weak, alias("unused_isr")));
void uart2_status_isr(void)     __attribute__ ((weak, alias("unused_isr")));
void uart2_error_isr(void)      __attribute__ ((weak, alias("unused_isr")));
void adc0_isr(void)             __attribute__ ((weak, alias("unused_isr")));
void cmp0_isr(void)             __attribute__ ((weak, alias("unused_isr")));
void cmp1_isr(void)             __attribute__ ((weak, alias("unused_isr")));
void ftm0_isr(void)             __attribute__ ((weak, alias("unused_isr")));
void ftm1_isr(void)             __attribute__ ((weak, alias("unused_isr")));
void cmt_isr(void)              __attribute__ ((weak, alias("unused_isr")));
void rtc_alarm_isr(void)        __attribute__ ((weak, alias("unused_isr")));
void rtc_seconds_isr(void)      __attribute__ ((weak, alias("unused_isr")));
void pit0_isr(void)             __attribute__ ((weak, alias("unused_isr")));
void pit1_isr(void)             __attribute__ ((weak, alias("unused_isr")));
void pit2_isr(void)             __attribute__ ((weak, alias("unused_isr")));
void pit3_isr(void)             __attribute__ ((weak, alias("unused_isr")));
void pdb_isr(void)              __attribute__ ((weak, alias("unused_isr")));
void usb_isr(void)              __attribute__ ((weak, alias("unused_isr")));
void usb_charge_isr(void)       __attribute__ ((weak, alias("unused_isr")));
void tsi0_isr(void)             __attribute__ ((weak, alias("unused_isr")));
void mcg_isr(void)              __attribute__ ((weak, alias("unused_isr")));
void lptmr_isr(void)            __attribute__ ((weak, alias("unused_isr")));
void porta_isr(void)            __attribute__ ((weak, alias("unused_isr")));
void portb_isr(void)            __attribute__ ((weak, alias("unused_isr")));
void portc_isr(void)            __attribute__ ((weak, alias("unused_isr")));
void portd_isr(void)            __attribute__ ((weak, alias("unused_isr")));
void porte_isr(void)            __attribute__ ((weak, alias("unused_isr")));
void software_isr(void)         __attribute__ ((weak, alias("unused_isr")));

__attribute__ ((section(".vectors"), used))
void (* const vectors[])(void) =
{
    __startup,               /* 1 ARM: Initial Program Counter */
    nmi_isr,                 /* 2 ARM: Non-maskable Interrupt (NMI) */
    hard_fault_isr,          /* 3 ARM: Hard Fault */
    memmanage_fault_isr,     /* 4 ARM: MemManage Fault */
    bus_fault_isr,           /* 5 ARM: Bus Fault */
    usage_fault_isr,         /* 6 ARM: Usage Fault */
    fault_isr,               /* 7 -- */
    fault_isr,               /* 8 -- */
    fault_isr,               /* 9 -- */
    fault_isr,               /* 10 -- */
    svcall_isr,              /* 11 ARM: Supervisor call (SVCall) */
    debugmonitor_isr,        /* 12 ARM: Debug Monitor */
    fault_isr,               /* 13 -- */
    pendablesrvreq_isr,      /* 14 ARM: Pendable req serv(PendableSrvReq) */
    systick_isr,             /* 15 ARM: System tick timer (SysTick) */
    dma_ch0_isr,             /* 16 DMA channel 0 transfer complete */
    dma_ch1_isr,             /* 17 DMA channel 1 transfer complete */
    dma_ch2_isr,             /* 18 DMA channel 2 transfer complete */
    dma_ch3_isr,             /* 19 DMA channel 3 transfer complete */
    dma_error_isr,           /* 20 DMA error interrupt channel */
    unused_isr,              /* 21 DMA -- */
    flash_cmd_isr,           /* 22 Flash Memory Command complete */
    flash_error_isr,         /* 23 Flash Read collision */
    low_voltage_isr,         /* 24 Low-voltage detect/warning */
    wakeup_isr,              /* 25 Low Leakage Wakeup */
    watchdog_isr,            /* 26 Both EWM and WDOG interrupt */
    i2c0_isr,                /* 27 I2C0 */
    spi0_isr,                /* 28 SPI0 */
    i2s0_tx_isr,             /* 29 I2S0 Transmit */
    i2s0_rx_isr,             /* 30 I2S0 Receive */
    uart0_lon_isr,           /* 31 UART0 CEA709.1-B (LON) status */
    uart0_status_isr,        /* 32 UART0 status */
    uart0_error_isr,         /* 33 UART0 error */
    uart1_status_isr,        /* 34 UART1 status */
    uart1_error_isr,         /* 35 UART1 error */
    uart2_status_isr,        /* 36 UART2 status */
    uart2_error_isr,         /* 37 UART2 error */
    adc0_isr,                /* 38 ADC0 */
    cmp0_isr,                /* 39 CMP0 */
    cmp1_isr,                /* 40 CMP1 */
    ftm0_isr,                /* 41 FTM0 */
    ftm1_isr,                /* 42 FTM1 */
    cmt_isr,                 /* 43 CMT */
    rtc_alarm_isr,           /* 44 RTC Alarm interrupt */
    rtc_seconds_isr,         /* 45 RTC Seconds interrupt */
    pit0_isr,                /* 46 PIT Channel 0 */
    pit1_isr,                /* 47 PIT Channel 1 */
    pit2_isr,                /* 48 PIT Channel 2 */
    pit3_isr,                /* 49 PIT Channel 3 */
    pdb_isr,                 /* 50 PDB Programmable Delay Block */
    usb_isr,                 /* 51 USB OTG */
    usb_charge_isr,          /* 52 USB Charger Detect */
    tsi0_isr,                /* 53 TSI0 */
    mcg_isr,                 /* 54 MCG */
    lptmr_isr,               /* 55 Low Power Timer */
    porta_isr,               /* 56 Pin detect (Port A) */
    portb_isr,               /* 57 Pin detect (Port B) */
    portc_isr,               /* 58 Pin detect (Port C) */
    portd_isr,               /* 59 Pin detect (Port D) */
    porte_isr,               /* 60 Pin detect (Port E) */
    software_isr,            /* 61 Software interrupt */
};

/* Set a bogus backdoor key, and disable all protection. */

__attribute__ ((section(".flashconfig")))
const uint8_t flashconfig[16] = {
    0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00,
    0xff, 0xff, 0xff, 0xff, 0x02, 0xff, 0xff, 0xff
};

void reset(void)
{
    extern char __bss_start, __bss_end;
    extern char __data_start, __data_end, __data_load;
    char *c, *d;

    /* Unlock the watchdog in order to disable it.  There are strict
     * timing requirements here so it's best to disable interrupts
     * during the process. */

    disable_interrupts();
    WDOG_UNLOCK = (uint16_t)0xc520;
    WDOG_UNLOCK = (uint16_t)0xd928;
    enable_interrupts();

    WDOG_STCTRLH &= ~WDOG_STCTRLH_WDOGEN;

    /* Copy the data section to RAM and clear the bss section. */

    for (c = &__bss_start;
         c < &__bss_end;
         *c = 0, c += 1);

    for (c = &__data_start, d = &__data_load;
         c < &__data_end;
         *c = *d, c += 1, d += 1);

    /* Bring up the system clock.  We start out in FEI mode. */

    OSC0_CR = OSC_CR_SC8P | OSC_CR_SC2P; /* Configure crystal load
                                          * capacitors. */

    MCG_C2 = MCG_C2_RANGE0(2) | MCG_C2_EREFS0; /* Enable external crystal,
                                                  very high frequency range. */

    /* Switch to crystal as clock source, FLL input = 16 MHz / 512 */
    MCG_C1 =  MCG_C1_CLKS(2) | MCG_C1_FRDIV(4);

    /* Wait for crystal oscillator to stabilize. */

    while ((MCG_S & MCG_S_OSCINIT0) == 0);
    while ((MCG_S & MCG_S_IREFST) != 0);
    while ((MCG_S & MCG_S_CLKST_MASK) != MCG_S_CLKST(2));

    /* Now we're in FBE mode, config PLL input for
     * 16 MHz Crystal / 4 = 4 MHz */

    MCG_C5 = MCG_C5_PRDIV0(3);
    MCG_C6 = MCG_C6_PLLS | MCG_C6_VDIV0(0); /* Config PLL for 96 MHz output. */

    /* Wait for PLL lock and we're in PBE mode. */

    while (!(MCG_S & MCG_S_PLLST));
    while (!(MCG_S & MCG_S_LOCK0));

    /* Configure the clock divisors: 48 MHz core, 48 MHz bus and 24 MHz
     * flash. */

    SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV2(1) |
                  SIM_CLKDIV1_OUTDIV4(3);

    /* Switch to PLL as clock source, FLL input = 16 MHz / 512,
       Wait for PLL clock to be selected and we're in PEE mode. */

    MCG_C1 = MCG_C1_CLKS(0) | MCG_C1_FRDIV(4);
    while ((MCG_S & MCG_S_CLKST_MASK) != MCG_S_CLKST(3));

    /* Set up the LED GPIO outputs. */

    SIM_SCGC5 |= SIM_SCGC5_PORTD | SIM_SCGC5_PORTC;
    PORTD_PCR6 = PORT_PCR_MUX(1) | PORT_PCR_DSE;
    GPIOD_PDDR |= ((uint32_t)1 << 6);
    GPIOD_PSOR = ((uint32_t)1 << 6);

    PORTC_PCR1 = PORT_PCR_MUX(1) | PORT_PCR_DSE;
    GPIOC_PDDR |= ((uint32_t)1 << 1);
    GPIOC_PSOR = ((uint32_t)1 << 1);

    PORTC_PCR2 = PORT_PCR_MUX(1) | PORT_PCR_DSE;
    GPIOC_PDDR |= ((uint32_t)1 << 2);
    GPIOC_PCOR = ((uint32_t)1 << 2);

    /* Configure the SysTick timer. */

    SYST_RVR = 0xffffff - 1;
    SYST_CSR = SYST_CSR_ENABLE | SYST_CSR_CLKSOURCE | SYST_CSR_TICKINT;
    SYST_CVR = 0;

    /* Enable the PIT module. */

    SIM_SCGC6 |= SIM_SCGC6_PIT;
    PIT_MCR &= ~PIT_MCR_MDIS;

    __libc_init_array();

    main();

    sleep_while(1);
}
