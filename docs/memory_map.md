# Memory Map (STM32G474RETx: 512 KB Flash, 128 KB SRAM)

- `0x08000000` Flash: `.isr_vector` + `.text`/`.rodata` (firmware ≈ 60–90 KB).
- `0x20000000` SRAM: `.data`/`.bss` (control structs ≈ 2 KB, all static).
- `.dma_buffer` (NOLOAD, 4-byte aligned): ADC double buffer 64×2×int16 = 256 B,
  never memset by startup code.
- Telemetry queue: 16×32 B = 512 B static.
- Stack: 8 KB at top of SRAM (`_estack`); no heap (`_sbrk` returns ENOMEM).
- Hot-loop guarantee: zero malloc/new (verified by grep in CI).
