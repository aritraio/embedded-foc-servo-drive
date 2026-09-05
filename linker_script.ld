/* STM32G474RETx linker script — 512 KB Flash, 128 KB SRAM.
 * Simplified production layout: ISR vector in Flash, DMA buffers in SRAM1. */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
  SRAM  (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

_estack = ORIGIN(SRAM) + LENGTH(SRAM);

SECTIONS
{
  .isr_vector :
  {
    KEEP(*(.isr_vector))
  } > FLASH

  .text :
  {
    *(.text*) *(.rodata*)
    _etext = .;
  } > FLASH

  .ARM.exidx : { *(.ARM.exidx*) } > FLASH

  .data : AT (_etext)
  {
    _sdata = .;
    *(.data*)
    _edata = .;
  } > SRAM

  .bss (NOLOAD) :
  {
    _sbss = .;
    *(.bss*) *(COMMON)
    _ebss = .;
  } > SRAM

  /* 1 KB DMA descriptor + ADC double buffer, 4-byte aligned. */
  .dma_buffer (NOLOAD) :
  {
    . = ALIGN(4);
    _sdma = .;
    KEEP(*(.dma_buffer))
    . = ALIGN(4);
    _edma = .;
  } > SRAM

  /DISCARD/ : { *(.note*) *(.comment*) }
}
