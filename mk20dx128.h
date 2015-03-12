#ifndef __MK20DX128__
#define __MK20DX128__

#include <stdint.h>

#define PORTB_PCR0 (*((volatile uint32_t *)0x4004a000))
#define PORTB_PCR1 (*((volatile uint32_t *)0x4004a004))
#define PORTB_PCR2 (*((volatile uint32_t *)0x4004a008))
#define PORTB_PCR3 (*((volatile uint32_t *)0x4004a00c))
#define PORTB_PCR16 (*((volatile uint32_t *)0x4004a040))
#define PORTB_PCR17 (*((volatile uint32_t *)0x4004a044))

#define PORTC_PCR0 (*((volatile uint32_t *)0x4004b000))
#define PORTC_PCR1 (*((volatile uint32_t *)0x4004b004))
#define PORTC_PCR2 (*((volatile uint32_t *)0x4004b008))
#define PORTC_PCR3 (*((volatile uint32_t *)0x4004b00c))
#define PORTC_PCR4 (*((volatile uint32_t *)0x4004b010))
#define PORTC_PCR5 (*((volatile uint32_t *)0x4004b014))
#define PORTC_PCR6 (*((volatile uint32_t *)0x4004b018))
#define PORTC_PCR7 (*((volatile uint32_t *)0x4004b01c))

#define PORTD_PCR1 (*((volatile uint32_t *)0x4004c004))
#define PORTD_PCR3 (*((volatile uint32_t *)0x4004c00c))
#define PORTD_PCR5 (*((volatile uint32_t *)0x4004c014))
#define PORTD_PCR6 (*((volatile uint32_t *)0x4004c018))

#define PORT_PCR_MUX(n) (((uint32_t)(n) & 0b111) << 8)
#define PORT_PCR_IRQC(n) (((uint32_t)(n) & 0b1111) << 16)
#define PORT_PCR_ISF ((uint32_t)1 << 24)
#define PORT_PCR_DSE ((uint32_t)1 << 6)
#define PORT_PCR_ODE ((uint32_t)1 << 5)
#define PORT_PCR_PFE ((uint32_t)1 << 4)
#define PORT_PCR_SRE ((uint32_t)1 << 2)
#define PORT_PCR_PE ((uint32_t)1 << 1)
#define PORT_PCR_PS ((uint32_t)1 << 0)

#define GPIOB_PDOR (*((volatile uint32_t *)0x400ff040))
#define GPIOB_PSOR (*((volatile uint32_t *)0x400ff044))
#define GPIOB_PCOR (*((volatile uint32_t *)0x400ff048))
#define GPIOB_PTOR (*((volatile uint32_t *)0x400ff04c))
#define GPIOB_PDIR (*((volatile uint32_t *)0x400ff050))
#define GPIOB_PDDR (*((volatile uint32_t *)0x400ff054))

#define GPIOC_PDOR (*((volatile uint32_t *)0x400ff080))
#define GPIOC_PSOR (*((volatile uint32_t *)0x400ff084))
#define GPIOC_PCOR (*((volatile uint32_t *)0x400ff088))
#define GPIOC_PTOR (*((volatile uint32_t *)0x400ff08c))
#define GPIOC_PDIR (*((volatile uint32_t *)0x400ff090))
#define GPIOC_PDDR (*((volatile uint32_t *)0x400ff094))

#define GPIOD_PDOR (*((volatile uint32_t *)0x400ff0c0))
#define GPIOD_PSOR (*((volatile uint32_t *)0x400ff0c4))
#define GPIOD_PCOR (*((volatile uint32_t *)0x400ff0c8))
#define GPIOD_PTOR (*((volatile uint32_t *)0x400ff0cc))
#define GPIOD_PDIR (*((volatile uint32_t *)0x400ff0d0))
#define GPIOD_PDDR (*((volatile uint32_t *)0x400ff0d4))

#define OSC0_CR (*((volatile uint8_t *)0x40065000))
#define OSC_CR_SC8P ((uint8_t)1 << 1)
#define OSC_CR_SC2P ((uint8_t)1 << 3)

#define MCG_C1 (*((volatile uint8_t *)0x40064000))
#define MCG_C2 (*((volatile uint8_t *)0x40064001))
#define MCG_C5 (*((volatile uint8_t *)0x40064004))
#define MCG_C6 (*((volatile uint8_t *)0x40064005))
#define MCG_S (*((volatile uint8_t *)0x40064006))
#define MCG_C1_CLKS(n) (((uint8_t)(n) & 0b11) << 6)
#define MCG_C1_FRDIV(n) (((uint8_t)(n) & 0b111) << 3)
#define MCG_C1_IRCLKEN ((uint8_t)1 << 1)
#define MCG_C2_RANGE0(n) (((uint8_t)(n) & 0b11) << 4)
#define MCG_C2_EREFS0 ((uint8_t)1 << 2)
#define MCG_C5_PRDIV0(n)(uint8_t) ((n) & 0b11111)
#define MCG_C6_PLLS ((uint8_t)1 << 6)
#define MCG_C6_VDIV0(n)(uint8_t) ((n) & 0b11111)
#define MCG_S_OSCINIT0 ((uint8_t)1 << 1)
#define MCG_S_IREFST ((uint8_t)1 << 4)
#define MCG_S_CLKST_MASK (uint8_t)(0b11UL << 2)
#define MCG_S_CLKST(n) (((uint8_t)(n) & 0b11) << 2)
#define MCG_S_PLLST ((uint8_t)1 << 5)
#define MCG_S_LOCK0 ((uint8_t)1 << 6)

#define SIM_SCGC4 (*((volatile uint32_t *)0x40048034))
#define SIM_SCGC5 (*((volatile uint32_t *)0x40048038))
#define SIM_SCGC6 (*((volatile uint32_t *)0x4004803C))
#define SIM_CLKDIV1 (*((volatile uint32_t *)0x40048044))
#define SIM_CLKDIV2 (*((volatile uint32_t *)0x40048048))
#define SIM_SOPT1 (*((volatile uint32_t *)0x40047000))
#define SIM_SOPT2 (*((volatile uint32_t *)0x40048004))
#define SIM_SCGC4_USBOTG ((uint32_t)1 << 18)
#define SIM_SCGC4_I2C0 ((uint32_t)1 << 6)
#define SIM_SCGC4_UART0 ((uint32_t)1 << 10)
#define SIM_SCGC5_LPTIMER ((uint32_t)1 << 0)
#define SIM_SCGC5_PORTB ((uint32_t)1 << 10)
#define SIM_SCGC5_PORTC ((uint32_t)1 << 11)
#define SIM_SCGC5_PORTD ((uint32_t)1 << 12)
#define SIM_SCGC6_SPI0 ((uint32_t)1 << 12)
#define SIM_SCGC6_PIT ((uint32_t)1 << 23)
#define SIM_SCGC6_CRC ((uint32_t)1 << 18)
#define SIM_CLKDIV1_OUTDIV1(n) (((uint32_t)(n) & 0b1111) << 28)
#define SIM_CLKDIV1_OUTDIV2(n) (((uint32_t)(n) & 0b1111) << 24)
#define SIM_CLKDIV1_OUTDIV4(n) (((uint32_t)(n) & 0b1111) << 16)
#define SIM_CLKDIV2_USBDIV(n) (((uint32_t)(n) & 0b111) << 1)
#define SIM_CLKDIV2_USBFRAC ((uint32_t)1 << 0)
#define SIM_SOPT2_USBSRC ((uint32_t)1 << 18)
#define SIM_SOPT2_PLLFLLSEL ((uint32_t)1 << 16)
#define SIM_SOPT2_TRACECLKSEL ((uint32_t)1 << 12)
#define SIM_SOPT2_CLKOUTSEL(n) (((uint32_t)(n) & 0b111) << 5)
#define SIM_SOPT1_OSC32KSEL(n) (((uint32_t)(n) & 0b11) << 18)

#define WDOG_STCTRLH (*(volatile uint16_t *)0x40052000)
#define WDOG_UNLOCK (*(volatile uint16_t *)0x4005200e)
#define WDOG_STCTRLH_WDOGEN ((uint16_t)1 << 0)
#define WDOG_STCTRLH_ALLOWUPDATE ((uint16_t)1 << 4)

#define USB0_STAT (*((volatile uint8_t *)0x40072090))
#define USB0_INTEN (*((volatile uint8_t *)0x40072084))
#define USB0_ERREN (*((volatile uint8_t *)0x4007208c))
#define USB0_OTGISTAT (*((volatile uint8_t *)0x40072010))
#define USB0_ISTAT (*((volatile uint8_t *)0x40072080))
#define USB0_ERRSTAT (*((volatile uint8_t *)0x40072088))
#define USB0_CONTROL (*((volatile uint8_t *)0x40072108))
#define USB0_CTL (*((volatile uint8_t *)0x40072094))
#define USB0_USBCTRL (*((volatile uint8_t *)0x40072100))
#define USB0_BDTPAGE1 (*((volatile uint8_t *)0x4007209c))
#define USB0_BDTPAGE2 (*((volatile uint8_t *)0x400720b0))
#define USB0_BDTPAGE3 (*((volatile uint8_t *)0x400720b4))
#define USB0_USBTRC0 (*((volatile uint8_t *)0x4007210c))
#define USB0_ENDPT(n) (*((volatile uint8_t *)(0x400720c0 + 4 * (n))))
#define USB0_ADDR (*((volatile uint8_t *)0x40072098))
#define USB_USBTRC0_USBRESET ((uint8_t)1 << 7)
#define USB_CTL_TXSUSPENDTOKENBUSY ((uint8_t)1 << 5)
#define USB_CTL_USBENSOFEN ((uint8_t)1 << 0)
#define USB_CTL_ODDRST ((uint8_t)1 << 1)
#define USB_USBCTRL_SUSP ((uint8_t)1 << 7)
#define USB_USBCTRL_PDE ((uint8_t)1 << 6)
#define USB_CONTROL_DPPULLUPNONOTG ((uint8_t)1 << 4)
#define USB_ISTAT_STALL ((uint8_t)1 << 7)
#define USB_ISTAT_SLEEP ((uint8_t)1 << 4)
#define USB_ISTAT_TOKDNE ((uint8_t)1 << 3)
#define USB_ISTAT_SOFTOK ((uint8_t)1 << 2)
#define USB_ISTAT_ERROR ((uint8_t)1 << 1)
#define USB_ISTAT_USBRST ((uint8_t)1 << 0)
#define USB_ERRSTAT_BTOERR ((uint8_t)1 << 4)
#define USB_ENDPT_EPRXEN ((uint8_t)1 << 3)
#define USB_ENDPT_EPTXEN ((uint8_t)1 << 2)
#define USB_ENDPT_EPSTALL ((uint8_t)1 << 1)
#define USB_ENDPT_EPHSHK ((uint8_t)1 << 0)
#define USB_INTEN_STALLEN ((uint8_t)1 << 7)
#define USB_INTEN_SLEEPEN ((uint8_t)1 << 4)
#define USB_INTEN_TOKDNEEN ((uint8_t)1 << 3)
#define USB_INTEN_SOFTOKEN ((uint8_t)1 << 2)
#define USB_INTEN_ERROREN ((uint8_t)1 << 1)
#define USB_INTEN_USBRSTEN ((uint8_t)1 << 0)
#define USB_STAT_TX ((uint8_t)1 << 3)
#define USB_STAT_ODD ((uint8_t)1 << 2)
#define USB_STAT_ENDP(n) (uint8_t)((n) >> 4)

#define DMA_ES (*((volatile uint32_t *)0x40008004))
#define DMA_ES_VLD ((uint32_t)1 << 31)

#define SYST_CSR *(volatile uint32_t *)0xe000e010
#define SYST_RVR *(volatile uint32_t *)0xe000e014
#define SYST_CVR *(volatile uint32_t *)0xe000e018
#define SYST_CALIB *(const uint32_t *)0xe000e01c
#define SYST_CSR_COUNTFLAG ((uint32_t)1 << 16)
#define SYST_CSR_CLKSOURCE ((uint32_t)1 << 2)
#define SYST_CSR_TICKINT ((uint32_t)1 << 1)
#define SYST_CSR_ENABLE ((uint32_t)1 << 0)

#define I2C0_A1 *(volatile uint8_t *)0x40066000
#define I2C0_F *(volatile uint8_t *)0x40066001
#define I2C0_C1 *(volatile uint8_t *)0x40066002
#define I2C0_S *(volatile uint8_t *)0x40066003
#define I2C0_D *(volatile uint8_t *)0x40066004
#define I2C0_C2 *(volatile uint8_t *)0x40066005
#define I2C0_FLT *(volatile uint8_t *)0x40066006
#define I2C_F_MULT(n) (((uint8_t)(n) & 0b11) << 6)
#define I2C_F_ICR(n) (((uint8_t)(n) & 0b111111) << 0)
#define I2C_C1_IICEN ((uint8_t)1 << 7)
#define I2C_C1_IICIE ((uint8_t)1 << 6)
#define I2C_C1_MST ((uint8_t)1 << 5)
#define I2C_C1_TX ((uint8_t)1 << 4)
#define I2C_C1_TXAK ((uint8_t)1 << 3)
#define I2C_C1_RSTA ((uint8_t)1 << 2)
#define I2C_C1_DMAEN ((uint8_t)1 << 0)
#define I2C_S_TCF ((uint8_t)1 << 7)
#define I2C_S_BUSY ((uint8_t)1 << 5)
#define I2C_S_ARBL ((uint8_t)1 << 4)
#define I2C_S_IICIF ((uint8_t)1 << 1)
#define I2C_S_RXAK ((uint8_t)1 << 0)
#define I2C_C2_HDRS ((uint8_t)1 << 5)
#define I2C_FLT_FLT(n) (uint8_t)((n) & 0b1111)

#define SPI0_MCR (*((volatile uint32_t *)0x4002c000))
#define SPI_MCR_HALT ((uint32_t)1 << 0)
#define SPI_MCR_CLR_RXF ((uint32_t)1 << 10)
#define SPI_MCR_CLR_TXF ((uint32_t)1 << 11)
#define SPI_MCR_DIS_RXF ((uint32_t)1 << 12)
#define SPI_MCR_DIS_TXF ((uint32_t)1 << 13)
#define SPI_MCR_MDIS ((uint32_t)1 << 14)
#define SPI_MCR_ROOE ((uint32_t)1 << 24)
#define SPI_MCR_DCONF(n) (((uint32_t)(n) & 0b11) << 28)
#define SPI_MCR_MSTR ((uint32_t)1 << 31)
#define SPI0_TCR (*((volatile uint32_t *)0x4002c008))
#define SPI_TCR_TCNT(n) (uint32_t)((n) << 16)
#define SPI0_CTAR0 (*((volatile uint32_t *)0x4002c00c))
#define SPI0_CTAR1 (*((volatile uint32_t *)0x4002c010))
#define SPI_CTAR_BR(n) (((uint32_t)(n) & 0b1111) << 0)
#define SPI_CTAR_DT(n) (((uint32_t)(n) & 0b1111) << 4)
#define SPI_CTAR_ASC(n) (((uint32_t)(n) & 0b1111) << 8)
#define SPI_CTAR_CSSCK(n) (((uint32_t)(n) & 0b1111) << 12)
#define SPI_CTAR_PBR(n) (((uint32_t)(n) & 0b11) << 16)
#define SPI_CTAR_PDT(n) (((uint32_t)(n) & 0b11) << 18)
#define SPI_CTAR_PASC(n) (((uint32_t)(n) & 0b11) << 20)
#define SPI_CTAR_PCSSCK(n) (((uint32_t)(n) & 0b11) << 22)
#define SPI_CTAR_LSBFE ((uint32_t)1 << 24)
#define SPI_CTAR_CPHA ((uint32_t)1 << 25)
#define SPI_CTAR_CPOL ((uint32_t)1 << 26)
#define SPI_CTAR_FMSZ(n) (((uint32_t)(n) & 0b1111) << 27)
#define SPI_CTAR_DBR ((uint32_t)1 << 31)
#define SPI0_SR (*((volatile uint32_t *)0x4002c02c))
#define SPI_SR_RFDF ((uint32_t)1 << 17)
#define SPI_SR_TFFF ((uint32_t)1 << 25)
#define SPI_SR_TCF ((uint32_t)1 << 31)
#define SPI0_PUSHR (*((volatile uint32_t *)0x4002c034))
#define SPI_PUSHR_PCS_MASK ((uint32_t)0b111111 << 16)
#define SPI_PUSHR_PCS(n) ((~((uint32_t)1 << (16 + n))) & SPI_PUSHR_PCS_MASK)
#define SPI_PUSHR_CTAS(n) (((uint32_t)(n) & 0b111) << 28)
#define SPI0_POPR (*((volatile uint32_t *)0x4002c038))
#define SPI0_RSER (*((volatile uint32_t *)0x4002c030))
#define SPI_RSER_TCF_RE ((uint32_t)1 << 31)
#define SPI_RSER_TFFF_RE ((uint32_t)1 << 25)
#define SPI_RSER_RFDF_RE ((uint32_t)1 << 17)

#define PIT_MCR *(volatile uint32_t *)0x40037000
#define PIT_LDVAL(n) (*((volatile uint32_t *)(0x40037100 + 0x10 * (n))))
#define PIT_CVAL(n) (*((volatile uint32_t *)(0x40037104 + 0x10 * (n))))
#define PIT_TCTRL(n) (*((volatile uint32_t *)(0x40037108 + 0x10 * (n))))
#define PIT_TFLG(n) (*((volatile uint32_t *)(0x4003710C + 0x10 * (n))))
#define PIT_MCR_MDIS ((uint32_t)1 << 1)
#define PIT_TCTRL_TEN ((uint32_t)1 << 0)
#define PIT_TCTRL_TIE ((uint32_t)1 << 1)
#define PIT_TFLG_TIF ((uint32_t)1 << 0)

#define NVIC_ISER(n) (*((volatile uint32_t *)(0xe000e100 + 4 * (n))))
#define NVIC_ICER(n) (*((volatile uint32_t *)(0xe000e180 + 4 * (n))))
#define NVIC_ISPR(n) (*((volatile uint32_t *)(0xe000e200 + 4 * (n))))
#define NVIC_ICPR(n) (*((volatile uint32_t *)(0xe000e280 + 4 * (n))))
#define NVIC_IPR(n)  (*((volatile uint32_t *)(0xe000e400 + 4 * (n))))

#define enable_interrupt(n) (NVIC_ISER(n / 32) = ((uint32_t)1 << (n % 32)))
#define disable_interrupt(n) (NVIC_ICER(n / 32) = ((uint32_t)1 << (n % 32)))
#define pend_interrupt(n) (NVIC_ISPR(n / 32) = ((uint32_t)1 << (n % 32)))
#define unpend_interrupt(n) (NVIC_ICPR(n / 32) = ((uint32_t)1 << (n % 32)))
#define prioritize_interrupt(n, p)                                      \
    {                                                                   \
        int i = n / 4, j = 8 * (n % 4) + 4;                             \
                                                                        \
        NVIC_IPR(i) = ((NVIC_IPR(i) & ~((uint32_t)0xf << j)) |          \
                       ((p & 0xf) << j));                               \
    }

#define disable_all_interrupts()                                \
    {                                                           \
        NVIC_ICER(0) = ~(uint32_t)1;                            \
        NVIC_ICER(1) = ~(uint32_t)1;                            \
    }

typedef void (*SoftwareIsrCallback)(void *userdata);
void set_software_isr_callback(SoftwareIsrCallback new_callback,
                               void *new_userdata);

#define pend_software_interrupt(n) pend_interrupt(45)
#define unpend_software_interrupt() unpend_interrupt(45)
#define prioritize_software_interrupt(p) prioritize_interrupt(45, p)

#define sleep() asm("wfi")
#define sleep_if(cond) asm volatile("cmp %0, #0\n\t"                    \
                                    "it ne\n\t"                         \
                                    "wfine" : : "r" (cond) : "cc")
#define disable_interrupts() asm("cpsid i")
#define enable_interrupts() asm("cpsie i")
#define sleep_while(cond)                               \
    {                                                   \
        do {                                            \
            disable_interrupts();                       \
            sleep_if (cond);                            \
            enable_interrupts();                        \
        } while(cond);                                  \
    }

#define SCB_CFSR (*((volatile uint32_t *)(0xe000ed28)))
#define SCB_HFSR (*((volatile uint32_t *)(0xe000ed2c)))
#define SCB_MMAR (*((volatile uint32_t *)(0xe000ed34)))
#define SCB_BFAR (*((volatile uint32_t *)(0xe000ed38)))
#define SCB_CFSR_MMARVALID ((uint32_t)1 << 7)
#define SCB_CFSR_BFARVALID ((uint32_t)1 << 15)
#define SCB_AIRCR (*((volatile uint32_t *)(0xe000ed0c)))

#define request_reset() SCB_AIRCR = 0x5fa0004; while(1)
#define request_reboot() asm("bkpt")

#define CRC_CRC32 *(volatile uint32_t *)0x40032000
#define CRC_CRC16 *(volatile uint16_t *)0x40032000
#define CRC_CRC8 *(volatile uint8_t *)0x40032000
#define CRC_GPOLY *(volatile uint32_t *)0x40032004
#define CRC_CTRL *(volatile uint32_t *)0x40032008
#define CRC_CTRL_TCRC ((uint32_t)1 << 24)
#define CRC_CTRL_WAS ((uint32_t)1 << 25)

#define LPTMR0_CSR (*((volatile uint32_t *)(0x40040000)))
#define LPTMR0_PSR (*((volatile uint32_t *)(0x40040004)))
#define LPTMR0_CMR (*((volatile uint32_t *)(0x40040008)))
#define LPTMR0_CNR (*((volatile uint32_t *)(0x4004000c)))

#define LPTMR_CSR_TEN ((uint32_t)1 << 0)
#define LPTMR_CSR_TMS ((uint32_t)1 << 1)
#define LPTMR_CSR_TFC ((uint32_t)1 << 2)
#define LPTMR_CSR_TPP ((uint32_t)1 << 3)
#define LPTMR_CSR_TPS(n) (((uint32_t)(n) & 0b11) << 4)
#define LPTMR_CSR_TIE ((uint32_t)1 << 6)
#define LPTMR_CSR_TCF ((uint32_t)1 << 7)

#define LPTMR_PSR_PCS(n) (((uint32_t)(n) & 0b11) << 0)
#define LPTMR_PSR_PBYP ((uint32_t)1 << 2)
#define LPTMR_PSR_PRESCALE(n) (((uint32_t)(n) & 0b1111) << 3)

#define UART0_BDH (*((volatile uint8_t *)0x4006a000))
#define UART0_BDL (*((volatile uint8_t *)0x4006a001))
#define UART0_C1 (*((volatile uint8_t *)0x4006a002))
#define UART0_C2 (*((volatile uint8_t *)0x4006a003))
#define UART0_S1 (*((volatile uint8_t *)0x4006a004))
#define UART0_S2 (*((volatile uint8_t *)0x4006a005))
#define UART0_C3 (*((volatile uint8_t *)0x4006a006))
#define UART0_D (*((volatile uint8_t *)0x4006a007))
#define UART0_C4 (*((volatile uint8_t *)0x4006a00a))
#define UART0_C5 (*((volatile uint8_t *)0x4006a00b))
#define UART0_ED (*((volatile uint8_t *)0x4006a00c))
#define UART0_PFIFO (*((volatile uint8_t *)0x4006a010))
#define UART0_CFIFO (*((volatile uint8_t *)0x4006a011))
#define UART0_SFIFO (*((volatile uint8_t *)0x4006a012))
#define UART0_TWFIFO (*((volatile uint8_t *)0x4006a013))
#define UART0_TCFIFO (*((volatile uint8_t *)0x4006a014))
#define UART0_RWFIFO (*((volatile uint8_t *)0x4006a015))
#define UART0_RCFIFO (*((volatile uint8_t *)0x4006a016))

#define UART_BDH_SBR(n) (((uint8_t)(n) & 0b11111) << 0)
#define UART_BDH_RXEDGIE ((uint8_t)1 << 7)
#define UART_C1_LOOPS ((uint8_t)1 << 7)
#define UART_C1_M ((uint8_t)1 << 4)
#define UART_C1_PE ((uint8_t)1 << 1)
#define UART_C1_PT ((uint8_t)1 << 0)

#define UART_C2_TIE ((uint8_t)1 << 7)
#define UART_C2_RIE ((uint8_t)1 << 5)
#define UART_C2_TE ((uint8_t)1 << 3)
#define UART_C2_RE ((uint8_t)1 << 2)

#define UART_S1_TDRE ((uint8_t)1 << 7)
#define UART_S1_RDRF ((uint8_t)1 << 5)
#define UART_S1_OR ((uint8_t)1 << 3)
#define UART_S1_NF ((uint8_t)1 << 2)
#define UART_S1_FE ((uint8_t)1 << 1)
#define UART_S1_PF ((uint8_t)1 << 0)

#define UART_S2_RXEDGIF ((uint8_t)1 << 6)
#define UART_S2_MSBF ((uint8_t)1 << 5)
#define UART_S2_RAF ((uint8_t)1 << 0)

#define UART_C3_ORIE ((uint8_t)1 << 3)
#define UART_C3_NEIE ((uint8_t)1 << 2)
#define UART_C3_FEIE ((uint8_t)1 << 1)
#define UART_C3_PEIE ((uint8_t)1 << 0)

#define UART_C4_BRFA(n) (((uint8_t)(n) & 0b11111) << 0)

#define UART_ED_NOISY ((uint8_t)1 << 6)
#define UART_ED_PARITYE ((uint8_t)1 << 7)

#define UART_PFIFO_TXFE ((uint8_t)1 << 7)
#define UART_PFIFO_TXFIFOSIZE(n) ((n >> 4) & 0b111)
#define UART_PFIFO_RXFE ((uint8_t)1 << 3)
#define UART_PFIFO_RXFIFOSIZE(n) ((n >> 0) & 0b111)

#define UART_CFIFO_TXFLUSH ((uint8_t)1 << 7)
#define UART_CFIFO_RXFLUSH ((uint8_t)1 << 6)
#define UART_CFIFO_RXOFE ((uint8_t)1 << 2)
#define UART_CFIFO_TXOFE ((uint8_t)1 << 1)
#define UART_CFIFO_RXUFE ((uint8_t)1 << 0)

#define UART_CFIFO_TXEMPT ((uint8_t)1 << 7)
#define UART_CFIFO_RXEMPT ((uint8_t)1 << 6)
#define UART_CFIFO_RXOF ((uint8_t)1 << 2)
#define UART_CFIFO_TXOF ((uint8_t)1 << 1)
#define UART_CFIFO_RXUF ((uint8_t)1 << 0)

#define set_uart_baud_rate(n, rate)                           \
    UART##n##_BDH = UART_BDH_SBR(F_CPU / 4096 / rate);        \
    UART##n##_BDL = (uint8_t)(F_CPU / 16 / rate);             \
    UART##n##_C4 = UART_C4_BRFA(F_CPU * 2 / rate);

#endif
