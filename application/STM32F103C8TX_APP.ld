/**
 * @file    STM32F103C8TX_APP.ld
 * @brief   Linker script for the application.
 *
 * Flash origin is 0x08002000 (after the 8KB bootloader).
 * RAM is shared with the bootloader (only one runs at a time).
 */

ENTRY(Reset_Handler)

_estack = ORIGIN(RAM) + LENGTH(RAM);

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08002000, LENGTH = 54K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

SECTIONS
{
    /* ---- Vector table at 0x08002000 ---- */
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } > FLASH

    /* ---- Code ---- */
    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        *(.glue_7)
        *(.glue_7t)
        *(.eh_frame)

        KEEP(*(.init))
        KEEP(*(.fini))

        . = ALIGN(4);
        _etext = .;
    } > FLASH

    /* ---- Constants ---- */
    .rodata :
    {
        . = ALIGN(4);
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    /* ---- ARM unwinding tables ---- */
    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > FLASH

    .ARM.exidx :
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > FLASH

    /* ---- Init arrays ---- */
    .preinit_array :
    {
        PROVIDE_HIDDEN(__preinit_array_start = .);
        KEEP(*(.preinit_array*))
        PROVIDE_HIDDEN(__preinit_array_end = .);
    } > FLASH

    .init_array :
    {
        PROVIDE_HIDDEN(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array*))
        PROVIDE_HIDDEN(__init_array_end = .);
    } > FLASH

    .fini_array :
    {
        PROVIDE_HIDDEN(__fini_array_start = .);
        KEEP(*(SORT(.fini_array.*)))
        KEEP(*(.fini_array*))
        PROVIDE_HIDDEN(__fini_array_end = .);
    } > FLASH

    _sidata = LOADADDR(.data);

    /* ---- Initialised data in RAM ---- */
    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } > RAM AT > FLASH

    /* ---- BSS ---- */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        __bss_start__ = _sbss;
        *(.bss)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        _ebss = .;
        __bss_end__ = _ebss;
    } > RAM

    /* Flash-writing wrappers are copied to RAM by ramfunc_init() in main. */
    .ramfunc :
    {
        . = ALIGN(4);
        _sramfunc = .;
        *(.ramfunc)
        *(.ramfunc*)
        . = ALIGN(4);
        _eramfunc = .;
    } > RAM AT > FLASH

    _siramfunc = LOADADDR(.ramfunc);

    /* ---- Heap for FreeRTOS ---- */
    ._user_heap_stack :
    {
        . = ALIGN(8);
        PROVIDE(end = .);
        PROVIDE(_end = .);
        /* libc is /DISCARD/ed below and the application never calls malloc:
         * FreeRTOS heap_4 owns its own static array (configTOTAL_HEAP_SIZE).
         * So `end`/`__heap_limit` only need to exist as symbols, and the RAM
         * previously reserved here is pure waste on a 20KB part.
         * 128 B is kept purely as a link-time sanity guard. */
        . = . + 128;
        . = ALIGN(8);
        __heap_limit = .;
    } > RAM

    /* ---- Remove unused libc/libm to save flash ---- */
    /DISCARD/ :
    {
        libc.a ( * )
        libm.a ( * )
        libgcc.a ( * )
    }
}
