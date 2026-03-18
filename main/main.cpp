#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "heap_memory_layout.h"
#include "esp_partition.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ulp_lp_core.h"
#include "ulp_lp_core_memory_shared.h"

extern "C" void app_main(void)
{
    uint32_t *z = (uint32_t *)(0x50004000 - (sizeof(uint32_t) * 30));
    while (1)
    {
        printf("Waiting %ld\n", *z);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}
