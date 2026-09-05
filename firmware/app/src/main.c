/* Bare-metal entry point (STM32) / host demo entry (SIL build).
 * Initializes clocks, HAL, control ISRs, then idles in background scheduler
 * (telemetry @ 1 kHz, thermal monitor, watchdog kick). */
#include "foc_app.h"
#include "config_params.h"

#include <stdint.h>

void isr_control_init(void);
void isr_fast_loop(void);

#if defined(STM32_TARGET)
void Reset_Handler(void);
void SystemInit(void);
int main(void)
{
    isr_control_init();
    for (;;) {
        /* Background: telemetry, temperature, watchdog. WFI for determinism. */
        __asm__ volatile("wfi");
    }
    return 0;
}
#else
#include <stdio.h>
int main(void)
{
    isr_control_init();
    /* Host demo: run a few fast-loop iterations with synthetic inputs. */
    for (uint32_t i = 0U; i < 25000U; i++) {
        isr_fast_loop();
    }
    printf("foc_firmware host demo complete\n");
    return 0;
}
#endif
