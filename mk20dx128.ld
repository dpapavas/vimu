MEMORY
{
        ROM (rx) : ORIGIN = 0x00000000, LENGTH = 128K
        RAM (rwx) : ORIGIN = 0x1FFFE000, LENGTH = 16K
}


SECTIONS
{
        .text : {
                . = 0;
                LONG(ORIGIN(RAM) + LENGTH(RAM)); /* The initial SP. */
                KEEP(reset.o (.vectors))
                . = 0x400;
                KEEP(*(.flashconfig*))
                crt0.o (.startup)
                __text_start = .;
                * (.text*)
                * (.rodata*)

                KEEP(*(.init))
                KEEP(*(.fini))

                __preinit_array_start = .;
                KEEP (*(.preinit_array))
                __preinit_array_end = .;

                __init_array_start = .;
                KEEP (*(SORT(.init_array.*)))
                KEEP (*(.init_array))
                __init_array_end = .;
        } > ROM = 0xFF

        .ARM.exidx : {
                __exidx_start = .;
                *(.ARM.exidx* .gnu.linkonce.armexidx.*)
                __exidx_end = .;
        } > ROM
        __text_end = .;

        .usbdata (NOLOAD) : {
                /* . = ORIGIN(RAM); */
                . = ALIGN(512);
                usb.o (.usbdata.bdt)
        } > RAM

        __data_start = .;
        .data : AT (__text_end) {
              . = ALIGN(4);
              *(.data*)
              . = ALIGN(4);
        } > RAM
        __data_end = .;
        __data_load = LOADADDR(.data);

        .noinit (NOLOAD) : {
                *(.noinit*)
        } > RAM

        __bss_start = .;
        .bss : {
                *(.bss*)
                *(COMMON)
                . = ALIGN(4);
        } > RAM
        __bss_end = .;
        __stack_start = .;

        __stack_end = ORIGIN(RAM) + LENGTH(RAM);
}
