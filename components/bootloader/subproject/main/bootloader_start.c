/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include "sdkconfig.h"
#include "esp_partition.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_rom_caps.h"
#include "bootloader_init.h"
#include "bootloader_utility.h"
#include "bootloader_common.h"
#include "bootloader_hooks.h"

#include "soc/rtc.h"

#include "hal/lp_timer_ll.h"
#include "hal/clk_tree_ll.h"
#include "hal/misc.h"
#include "hal/cpu_ll.h"
#include "hal/lp_core_ll.h"

#include "hal/spi_ll.h"
#include "hal/spi_types.h"
#include "soc/spi_periph.h"
#include "hal/clk_gate_ll.h"
#include "hal/gpio_hal.h"
#include "esp_rom_gpio.h"
#include "hal/lp_timer_ll.h"
#include "hal/gpio_ll.h"
#include "esp_rom_gpio.h"
#include "soc/rtc.h"
#include "hal/spi_types.h"
#include "hal/spi_ll.h"
#include "soc/clk_tree_defs.h"
#include "hal/clk_tree_ll.h"
#include "hal/lp_core_ll.h"

#include "bootloader_flash_priv.h"

#include "esp_rom_crc.h"

#if CONFIG_IDF_TARGET_ESP32P4 || CONFIG_IDF_TARGET_ESP32C5
#define LP_CORE_RCC_ATOMIC() PERIPH_RCC_ATOMIC()
#else
#define LP_CORE_RCC_ATOMIC()
#endif

#define TIMER_ID 1
#define RTC_SLOW_MEM ((uint32_t*) SOC_RTC_DATA_LOW)
#define CONFIG_ULP_SHARED_MEM 0x10
#define SOC_ULP_LP_UART_SUPPORTED       1
#define SOC_RTC_MEM_SUPPORT_SPEED_MODE_SWITCH           1
#define SOC_LP_TIMER_SUPPORTED          1
#define CONFIG_ULP_NORESET_UNDER_DEBUG 1
#define WAKEUP_SOURCE_MAX_NUMBER 6
#define CONFIG_ULP_COPROC_RESERVE_MEM 16348
#define ALIGN_DOWN(SIZE, AL)   (SIZE & ~(AL - 1))

#define ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU    BIT(0) // Started by HP core (1 single wakeup)
#define ULP_LP_CORE_WAKEUP_SOURCE_LP_UART   BIT(1) // Enable wake-up by a certain number of LP UART RX pulses
#define ULP_LP_CORE_WAKEUP_SOURCE_LP_IO     BIT(2) // Enable wake-up by LP IO interrupt
#define ULP_LP_CORE_WAKEUP_SOURCE_ETM       BIT(3) // Enable wake-up by ETM event
#define ULP_LP_CORE_WAKEUP_SOURCE_LP_TIMER  BIT(4) // Enable wake-up by LP timer
#define ULP_LP_CORE_WAKEUP_SOURCE_LP_VAD    BIT(5) // Enable wake-up by LP VAD

static const char *TAG = "boot";

static struct {
    lp_timer_dev_t *dev;
} lp_timer_context = { .dev = &LP_TIMER };

typedef struct lp_fw_info
{
    uint32_t lpfwtag;       // Should set to 0x4C504657 to indicate that the LP firmware is present
    uint16_t lpfwtype;
    uint16_t lpfwver;       // Version of the LP firmware
    uint16_t lpfwstatus;
    uint16_t lpfwcounter;   // Incremented periodically by the LP Core. Can be used by the HP core to determine that the LP core is still running.
    uint32_t hpfwcounter;   // Incremented periodically by the HP Core. Can be used by the LP core to determine that the HP core is still running.
} LP_Firmware_Info;

typedef struct {
    uint64_t sleep_duration_us;     /* Configured sleep duration for periodic wakeup, if set the ulp will automatically schedule the next wakeup */
    uint64_t sleep_duration_ticks;  /* Configured sleep duration, in LP-timer clock ticks, if set it allows us to skip doing integer division when configuring the timer  */
} ulp_lp_core_memory_shared_cfg_t;

typedef struct {
    uint32_t wakeup_source;                  /*!< Wakeup source flags */
    uint32_t lp_timer_sleep_duration_us;     /*!< Sleep duration when ULP_LP_CORE_WAKEUP_SOURCE_LP_TIMER is specified. Measurement unit: us */
#if ESP_ROM_HAS_LP_ROM
    bool    skip_lp_rom_boot;               /*!< Skips the LP rom code and boots directly into the app code placed in LP RAM,
                                                 this gives faster boot time for time sensitive use-cases at the cost of skipping
                                                 setup e.g. of UART */
#endif //ESP_ROM_HAS_LP_ROM
} ulp_lp_core_cfg_t;

static int select_partition_number(bootloader_state_t *bs);
static int selected_boot_partition(const bootloader_state_t *bs);

/* Maps the flags defined in ulp_lp_core.h e.g. ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU to their actual HW values */
static uint32_t wakeup_src_sw_to_hw_flag_lookup[WAKEUP_SOURCE_MAX_NUMBER] = {
    LP_CORE_LL_WAKEUP_SOURCE_HP_CPU,
    LP_CORE_LL_WAKEUP_SOURCE_LP_UART,
    LP_CORE_LL_WAKEUP_SOURCE_LP_IO,
    LP_CORE_LL_WAKEUP_SOURCE_ETM,
    LP_CORE_LL_WAKEUP_SOURCE_LP_TIMER,
#if SOC_LP_VAD_SUPPORTED
    LP_CORE_LL_WAKEUP_SOURCE_LP_VAD,
#endif
};

static void lp_timer_hal_set_alarm_target(uint64_t value)
{
    lp_timer_ll_clear_lp_alarm_intr_status(lp_timer_context.dev);
    lp_timer_ll_set_alarm_target(lp_timer_context.dev, TIMER_ID, value);
    lp_timer_ll_set_target_enable(lp_timer_context.dev, TIMER_ID, true);
}

uint64_t ulp_lp_core_lp_timer_get_cycle_count(void)
{
    lp_timer_ll_counter_snapshot(lp_timer_context.dev);

    uint32_t lo = lp_timer_ll_get_counter_value_low(lp_timer_context.dev, 0);
    uint32_t hi = lp_timer_ll_get_counter_value_high(lp_timer_context.dev, 0);

    lp_timer_counter_value_t result = {
        .lo = lo,
        .hi = hi
    };

    return result.val;
}

uint64_t ulp_lp_core_lp_timer_calculate_sleep_ticks(uint64_t sleep_duration_us)
{
    return (sleep_duration_us * (1 << RTC_CLK_CAL_FRACT) / clk_ll_rtc_slow_load_cal());
}

void ulp_lp_core_lp_timer_set_wakeup_time(uint64_t sleep_duration_us)
{
    uint64_t cycle_cnt = ulp_lp_core_lp_timer_get_cycle_count();
    uint64_t alarm_target = cycle_cnt + ulp_lp_core_lp_timer_calculate_sleep_ticks(sleep_duration_us);

    lp_timer_hal_set_alarm_target(alarm_target);
}

/* Convert the wake-up sources defined in ulp_lp_core.h to the actual HW wake-up source values */
static uint32_t lp_core_get_wakeup_source_hw_flags(uint32_t flags)
{
    uint32_t hw_flags = 0;
    for (int i = 0; i < WAKEUP_SOURCE_MAX_NUMBER; i++) {
        if (flags & (1 << i)) {
            hw_flags |= wakeup_src_sw_to_hw_flag_lookup[i];
        }
    }
    return hw_flags;
}

ulp_lp_core_memory_shared_cfg_t* ulp_lp_core_memory_shared_cfg_get(void)
{
#if IS_ULP_COCPU
    return &s_shared_mem;
#else
#if ESP_ROM_HAS_LP_ROM
    extern uint32_t _rtc_ulp_memory_start;
    uint32_t ulp_base_addr = (uint32_t)&_rtc_ulp_memory_start;
#else
    uint32_t ulp_base_addr = SOC_RTC_DRAM_LOW;
#endif
    /* Ensure the end where the shared memory starts is aligned to 8 bytes
    if updating this also update the same in ulp_lp_core_riscv.ld
    */
    return (ulp_lp_core_memory_shared_cfg_t *)(ulp_base_addr + ALIGN_DOWN(CONFIG_ULP_COPROC_RESERVE_MEM, 0x8)  - CONFIG_ULP_SHARED_MEM);
#endif
}

void ulp_lp_core_stop(void)
{
    if (esp_cpu_dbgr_is_attached()) {
        /* upon SW reset debugger puts LP core into the infinite loop at reset vector,
           so configure it to stall when going to sleep */
        lp_core_ll_stall_at_sleep_request(true);
        /* Avoid resetting chip in sleep mode when debugger is attached,
        otherwise configured HW breakpoints and dcsr.ebreak* bits will be missed */
        lp_core_ll_rst_at_sleep_enable(!CONFIG_ULP_NORESET_UNDER_DEBUG);
        lp_core_ll_debug_module_enable(true);
    }
    /* Disable wake-up source and put lp core to sleep */
    lp_core_ll_set_wakeup_source(0);
    lp_core_ll_request_sleep();
}

esp_err_t ulp_lp_core_run(ulp_lp_core_cfg_t* cfg)
{
    if (!cfg->wakeup_source) {
        ESP_LOGE(TAG, "No valid wakeup source specified");
        return ESP_ERR_INVALID_ARG;
    }

#if ESP_ROM_HAS_LP_ROM
    /* If we have a LP ROM we boot from it, before jumping to the app code */
    intptr_t boot_addr;
    if (cfg->skip_lp_rom_boot) {
        boot_addr = RESET_HANDLER_ADDR;
    } else {
        boot_addr = SOC_LP_ROM_LOW;
        /* Disable UART init in ROM, it defaults to XTAL clk src
         * which is not powered on during deep sleep
         * This will cause the ROM code to get stuck during UART output
         * if used
         */
        REG_SET_BIT(LP_UART_INIT_CTRL_REG, 1 << 0);
    }

    lp_core_ll_set_boot_address(boot_addr);
    lp_core_ll_set_app_boot_address(RESET_HANDLER_ADDR);

#endif //ESP_ROM_HAS_LP_ROM

    LP_CORE_RCC_ATOMIC() {
#if CONFIG_ULP_NORESET_UNDER_DEBUG
        /* lp_core module reset causes loss of configured HW breakpoints and dcsr.ebreak* */
        if (! esp_cpu_dbgr_is_attached()) {
            lp_core_ll_reset_register();
        }
#else
        lp_core_ll_reset_register();
#endif
        lp_core_ll_enable_bus_clock(true);
    }

#if SOC_RTC_MEM_SUPPORT_SPEED_MODE_SWITCH
    /* Disable fast LP mem access to allow LP core to access LP memory during sleep */
    lp_core_ll_fast_lp_mem_enable(false);
#endif

    /* Enable stall at sleep request*/
    lp_core_ll_stall_at_sleep_request(true);

    /* Enable reset CPU when going to sleep */
    /* Avoid resetting chip in sleep mode when debugger is attached,
       otherwise configured HW breakpoints and dcsr.ebreak* bits will be missed */
    lp_core_ll_rst_at_sleep_enable(!(CONFIG_ULP_NORESET_UNDER_DEBUG && esp_cpu_dbgr_is_attached()));

    /* Set wake-up sources */
    lp_core_ll_set_wakeup_source(lp_core_get_wakeup_source_hw_flags(cfg->wakeup_source));

    /* Enable JTAG debugging */
    lp_core_ll_debug_module_enable(true);

    if (cfg->wakeup_source & ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU) {
        lp_core_ll_hp_wake_lp();
    }

#if SOC_ULP_LP_UART_SUPPORTED
    if (cfg->wakeup_source & ULP_LP_CORE_WAKEUP_SOURCE_LP_UART) {
        lp_core_ll_enable_lp_uart_wakeup(true);
    }
#endif

#if SOC_LP_TIMER_SUPPORTED
    ulp_lp_core_memory_shared_cfg_t* shared_mem = ulp_lp_core_memory_shared_cfg_get();

    if (cfg->wakeup_source & ULP_LP_CORE_WAKEUP_SOURCE_LP_TIMER) {
        if (!cfg->lp_timer_sleep_duration_us) {
            ESP_LOGI(TAG, "LP timer specified as wakeup source, but no sleep duration set. ULP will only wake-up once unless it calls ulp_lp_core_lp_timer_set_wakeup_time()");
        }
        shared_mem->sleep_duration_us = cfg->lp_timer_sleep_duration_us;
        shared_mem->sleep_duration_ticks = ulp_lp_core_lp_timer_calculate_sleep_ticks(cfg->lp_timer_sleep_duration_us);

        // Set first wakeup alarm 
        ulp_lp_core_lp_timer_set_wakeup_time(cfg->lp_timer_sleep_duration_us);
    }
#endif

    return ESP_OK;
}

esp_err_t ulp_lp_core_load_binary()
{
    /* Turn off LP CPU before loading binary */
    ulp_lp_core_stop();
#if ESP_ROM_HAS_LP_ROM
    uint32_t* base = (uint32_t*)&_rtc_ulp_memory_start;
#else
    uint32_t* base = RTC_SLOW_MEM;
#endif

    hal_memset(base, 0, CONFIG_ULP_COPROC_RESERVE_MEM);
    esp_err_t err = bootloader_flash_read(0x3C0000, base, CONFIG_ULP_COPROC_RESERVE_MEM, false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to load\n");
    } else {
        ESP_LOGE(TAG, "Loaded\n");
    }

    // 1. Initial CRC value (usually 0xFFFFFFFF or 0x00000000)
    //uint32_t init_crc = 0xFFFFFFFF;

    // 2. Calculate CRC32 (Little Endian)
    // The ROM function calculates it directly
    //uint32_t crc32 = esp_rom_crc32_le(init_crc, (uint8_t*)base, 16344);
    //ESP_LOGE(TAG, "CRC: 0x%06x\n", crc32);

    return ESP_OK;
}

/*
 * We arrive here after the ROM bootloader finished loading this second stage bootloader from flash.
 * The hardware is mostly uninitialized, flash cache is down and the app CPU is in reset.
 * We do have a stack, so we can do the initialization in C.
 */
void __attribute__((noreturn)) call_start_cpu0(void)
{
    // (0. Call the before-init hook, if available)
    if (bootloader_before_init) {
        bootloader_before_init();
    }

    // 1. Hardware initialization
    if (bootloader_init() != ESP_OK) {
        bootloader_reset();
    }

    // (1.1 Call the after-init hook, if available)
    if (bootloader_after_init) {
        bootloader_after_init();
    }

#ifdef CONFIG_BOOTLOADER_SKIP_VALIDATE_IN_DEEP_SLEEP
    // If this boot is a wake up from the deep sleep then go to the short way,
    // try to load the application which worked before deep sleep.
    // It skips a lot of checks due to it was done before (while first boot).
    bootloader_utility_load_boot_image_from_deep_sleep();
    // If it is not successful try to load an application as usual.
#endif

    // 2. Select the number of boot partition
    bootloader_state_t bs = {0};
    int boot_index = select_partition_number(&bs);
    if (boot_index == INVALID_INDEX) {
        bootloader_reset();
    }

    int ReturnCode = ulp_lp_core_load_binary();
    if(ReturnCode != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to load LP FW");
    }

    ulp_lp_core_cfg_t cfg = {
        .wakeup_source = ULP_LP_CORE_WAKEUP_SOURCE_HP_CPU,
        .lp_timer_sleep_duration_us = 0,
    };

    ReturnCode = ulp_lp_core_run(&cfg);
    if(ReturnCode != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start LP FW: %x", ReturnCode);
    } else {
        ESP_LOGE(TAG, "Started lp", ReturnCode);
    }

    // 2.1 Load the TEE image
#if CONFIG_SECURE_ENABLE_TEE
    bootloader_utility_load_tee_image(&bs);
#endif

    // 3. Load the app image for booting
    bootloader_utility_load_boot_image(&bs, boot_index);
}

// Select the number of boot partition
static int select_partition_number(bootloader_state_t *bs)
{
    // 1. Load partition table
    if (!bootloader_utility_load_partition_table(bs)) {
        ESP_LOGE(TAG, "load partition table error!");
        return INVALID_INDEX;
    }

    // 2. Select the number of boot partition
    return selected_boot_partition(bs);
}

/*
 * Selects a boot partition.
 * The conditions for switching to another firmware are checked.
 */
static int selected_boot_partition(const bootloader_state_t *bs)
{
    int boot_index = bootloader_utility_get_selected_boot_partition(bs);
    if (boot_index == INVALID_INDEX) {
        return boot_index; // Unrecoverable failure (not due to corrupt ota data or bad partition contents)
    }
    if (esp_rom_get_reset_reason(0) != RESET_REASON_CORE_DEEP_SLEEP) {
        // Factory firmware.
#ifdef CONFIG_BOOTLOADER_FACTORY_RESET
        bool reset_level = false;
#if CONFIG_BOOTLOADER_FACTORY_RESET_PIN_HIGH
        reset_level = true;
#endif
        if (bootloader_common_check_long_hold_gpio_level(CONFIG_BOOTLOADER_NUM_PIN_FACTORY_RESET, CONFIG_BOOTLOADER_HOLD_TIME_GPIO, reset_level) == GPIO_LONG_HOLD) {
            ESP_LOGI(TAG, "Detect a condition of the factory reset");
            bool ota_data_erase = false;
#ifdef CONFIG_BOOTLOADER_OTA_DATA_ERASE
            ota_data_erase = true;
#endif
            const char *list_erase = CONFIG_BOOTLOADER_DATA_FACTORY_RESET;
            ESP_LOGI(TAG, "Data partitions to erase: %s", list_erase);
            if (bootloader_common_erase_part_type_data(list_erase, ota_data_erase) == false) {
                ESP_LOGE(TAG, "Not all partitions were erased");
            }
#ifdef CONFIG_BOOTLOADER_RESERVE_RTC_MEM
            bootloader_common_set_rtc_retain_mem_factory_reset_state();
#endif
            return bootloader_utility_get_selected_boot_partition(bs);
        }
#endif // CONFIG_BOOTLOADER_FACTORY_RESET
        // TEST firmware.
#ifdef CONFIG_BOOTLOADER_APP_TEST
        bool app_test_level = false;
#if CONFIG_BOOTLOADER_APP_TEST_PIN_HIGH
        app_test_level = true;
#endif
        if (bootloader_common_check_long_hold_gpio_level(CONFIG_BOOTLOADER_NUM_PIN_APP_TEST, CONFIG_BOOTLOADER_HOLD_TIME_GPIO, app_test_level) == GPIO_LONG_HOLD) {
            ESP_LOGI(TAG, "Detect a boot condition of the test firmware");
            if (bs->test.offset != 0) {
                boot_index = TEST_APP_INDEX;
                return boot_index;
            } else {
                ESP_LOGE(TAG, "Test firmware is not found in partition table");
                return INVALID_INDEX;
            }
        }
#endif // CONFIG_BOOTLOADER_APP_TEST
        // Customer implementation.
        // if (gpio_pin_1 == true && ...){
        //     boot_index = required_boot_partition;
        // } ...
    }
    return boot_index;
}

#if CONFIG_LIBC_NEWLIB
// Return global reent struct if any newlib functions are linked to bootloader
struct _reent *__getreent(void)
{
    return _GLOBAL_REENT;
}
#endif
