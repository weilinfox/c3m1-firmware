/*
 * C3M1 Factory Firmware DNG Tear Fix (Frame-Latch-First Trampoline)
 * Loaded at Flash address: 0x0800E900 (past end of original 0x0800E8C4 image)
 *
 * Replaces the single call `bl 0x08000B98` (camera_save_dng) at 0x080016B6:
 *   1. Waits until VSYNC EXTI9_5 ISR finishes latching one complete 644x480
 *      raw frame into AL422B (ov_sta == 2 @ 0x20001691, where FIFO_WEN PB3 is
 *      driven LOW and kept locked).
 *   2. Tail-calls original camera_save_dng() (0x08000B98) with original LR/SP
 *      intact, so f_open(), f_write(592B header), and STMFLASH_Write() (20-40ms
 *      Flash page erase CPU/IRQ stall) ALL run AFTER AL422B FIFO writes are
 *      locked off.
 *   3. Zero SDIO/FatFs modifications — strictly isolates the DNG tear race.
 */

    .syntax unified
    .cpu cortex-m3
    .thumb

    .section .text
    .global dng_lock_trampoline

    .equ OV_STA_ADDR,     0x20001691
    .equ CAMERA_SAVE_DNG, 0x08000B99   /* Thumb bit set */

    .align 2
    .thumb_func
dng_lock_trampoline:
    ldr     r3, =OV_STA_ADDR
1:
    ldrb    r2, [r3, #0]
    cmp     r2, #2
    bne     1b
    ldr     r3, =CAMERA_SAVE_DNG
    bx      r3

    .pool
