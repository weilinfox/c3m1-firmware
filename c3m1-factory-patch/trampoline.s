/*
 * C3M1 Factory Firmware SDIO Polling IRQ Guard Trampoline
 * Loaded at Flash address: 0x0800E900 (past end of original 0x0800E8C4 image)
 *
 * Wraps HAL_SD_ReadBlocks (0x080042F4) and HAL_SD_WriteBlocks (0x08004548):
 *   - Preserves r0..r3 and 5th stack argument (Timeout at caller [sp, #0])
 *   - Saves NVIC->ISER[0..1] (0xE000E100/04) and disables all enabled external
 *     peripheral IRQs via NVIC->ICER[0..1] (0xE000E180/84) so VSYNC/EXTI/TIM6
 *     cannot cause SDIO TXUNDERR/RXOVERR during polling transfer
 *   - Keeps Cortex-M3 SysTick running so HAL_GetTick() timeouts still work
 *   - Restores exact original NVIC->ISER[0..1] before returning HAL status in r0
 */

    .syntax unified
    .cpu cortex-m3
    .thumb

    .section .text
    .global sd_read_trampoline
    .global sd_write_trampoline

    .equ HAL_SD_READ_BLOCKS,   0x080042F5   /* Thumb bit set */
    .equ HAL_SD_WRITE_BLOCKS,  0x08004549   /* Thumb bit set */
    .equ NVIC_ISER0,           0xE000E100

    .align 2
    .thumb_func
sd_read_trampoline:
    ldr     r12, =HAL_SD_READ_BLOCKS
    b.n     sd_common_guard

    .align 2
    .thumb_func
sd_write_trampoline:
    ldr     r12, =HAL_SD_WRITE_BLOCKS

sd_common_guard:
    push    {r4, r5, r6, lr}        /* 16 bytes pushed; caller Timeout now at [sp, #16] */
    sub     sp, #8                  /* Maintain 8-byte SP alignment; 5th arg slot at [sp, #0] */
    ldr     r4, [sp, #24]           /* Load original 5th arg (Timeout) */
    str     r4, [sp, #0]            /* Pass Timeout to HAL_SD_*Blocks */

    ldr     r6, =NVIC_ISER0
    ldr     r4, [r6, #0x00]         /* Save enabled external IRQs 0..31 */
    ldr     r5, [r6, #0x04]         /* Save enabled external IRQs 32..63 */
    str     r4, [r6, #0x80]         /* NVIC->ICER[0] = r4 (mask IRQs 0..31) */
    str     r5, [r6, #0x84]         /* NVIC->ICER[1] = r5 (mask IRQs 32..63) */
    dsb
    isb

    blx     r12                     /* Call HAL_SD_ReadBlocks / HAL_SD_WriteBlocks */

    str     r4, [r6, #0x00]         /* Restore NVIC->ISER[0] */
    str     r5, [r6, #0x04]         /* Restore NVIC->ISER[1] */
    add     sp, #8
    pop     {r4, r5, r6, pc}        /* Return HAL status in r0 */

    .pool
