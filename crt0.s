  	.section .startup
        .align   2
        .thumb_func
        .global   __startup
        .type __startup STT_FUNC
        
__startup:
        @@ Initialize the general-purpose registers.

        mov     r0, #0
	mov     r1, #0
	mov     r2, #0
	mov     r3, #0
	mov     r4, #0
	mov     r5, #0
	mov     r6, #0
	mov     r7, #0
        mov     r8, r7
        mov     r9, r7
        mov     r10, r7
        mov     r11, r7
        mov     r12, r7

        @@ Enable interrupts.
        
	cpsie   i

        @@ ldr     r0, =__stack_end
        @@ cmp     r0, sp
        @@ beq     .

@@         @@ Copy the .data section to RAM.

@@         .extern __data_load
@@         .extern __data_start
@@         .extern __data_end

@@         ldr     r0, =__data_start
@@         ldr     r1, =__data_end
@@         ldr     r2, =__data_load

@@ 1:      cmp     r0, r1
@@         beq     2f
        
@@         ldmia   r2!, {r3}
@@         stmia   r0!, {r3}
@@         b       1b

@@         @@ Clear the .bss section.

@@         .extern __bss_start
@@         .extern __bss_end

@@ 2:      ldr     r0, =__bss_start
@@         ldr     r1, =__bss_end
@@         mov     r2, #0

@@ 3:      cmp     r0, r1
@@         beq     4f
        
@@         stmia   r0!, {r2}
@@         b       3b

        @@ Pass control to C.
        
        .extern reset
4:      bl      reset
        
        b       .
