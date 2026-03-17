#include <stdint.h>
#include "ulp_lp_core_print.h"
#include "ulp_lp_core_utils.h"
#include "ulp_lp_core_lp_timer_shared.h"
#include "ulp_lp_core_memory_shared.h"
#include "ulp_lp_core_gpio.h"
#include "ulp_lp_core_interrupts.h"

int main (void)
{
    uint32_t *z = (uint32_t*)( 0x50004000 - 10);
    *z=0;

    while (1) {
        z[0]++;
        for (int i = 0; i < 10; i++)
            ulp_lp_core_delay_us(100000);
    }

    return 0;
}
