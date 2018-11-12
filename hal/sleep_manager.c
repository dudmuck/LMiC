/* mbed Microcontroller Library
 * Copyright (c) 2017 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <limits.h>
#include <stdbool.h>
#include "power_mgmt.h"
#include "mbed_critical.h"
#include "ticker_api.h"
#include "sleep_api.h"

// deep sleep locking counter. A target is allowed to deep sleep if counter == 0
static uint16_t deep_sleep_lock = 0U;
static us_timestamp_t sleep_time = 0;
static us_timestamp_t deep_sleep_time = 0;

void sleep_manager_lock_deep_sleep_internal(void)
{   
    core_util_critical_section_enter();
    if (deep_sleep_lock == USHRT_MAX) {
        core_util_critical_section_exit();
        //MBED_ERROR1(MBED_MAKE_ERROR(MBED_MODULE_HAL, MBED_ERROR_CODE_OVERFLOW), "DeepSleepLock overflow (> USHRT_MAX)", deep_sleep_lock);
        for (;;) asm("nop");
    }
    core_util_atomic_incr_u16(&deep_sleep_lock, 1);
    core_util_critical_section_exit();
}

void sleep_manager_unlock_deep_sleep_internal(void)
{
    core_util_critical_section_enter();
    if (deep_sleep_lock == 0) {
        core_util_critical_section_exit();
        //MBED_ERROR1(MBED_MAKE_ERROR(MBED_MODULE_HAL, MBED_ERROR_CODE_UNDERFLOW), "DeepSleepLock underflow (< 0)", deep_sleep_lock);
        for (;;) asm("nop");
    }
    core_util_atomic_decr_u16(&deep_sleep_lock, 1);
    core_util_critical_section_exit();
}

bool sleep_manager_can_deep_sleep(void)
{
    return deep_sleep_lock == 0 ? true : false;
}

static inline us_timestamp_t read_us(void)
{
#if defined(MBED_CPU_STATS_ENABLED) && defined(DEVICE_LPTICKER)
    if (NULL == sleep_ticker) {
        sleep_ticker = (ticker_data_t *)get_lp_ticker_data();
    }
    return ticker_read_us(sleep_ticker);
#else
    return 0;
#endif
}

volatile uint8_t foo;
void sleep_manager_sleep_auto(void)
{
#ifdef MBED_SLEEP_TRACING_ENABLED
    sleep_tracker_print_stats();
#endif
    core_util_critical_section_enter();
    us_timestamp_t start = read_us();
    bool deep = false;

    foo = 1;
// debug profile should keep debuggers attached, no deep sleep allowed
#ifdef MBED_DEBUG
    hal_sleep();
#else
    if (sleep_manager_can_deep_sleep()) {
        deep = true;
        hal_deepsleep();
    } else {
        hal_sleep();
    }
#endif

    us_timestamp_t end = read_us();
    if (true == deep) {
        deep_sleep_time += end - start;
    } else {
        sleep_time += end - start;
    }
    core_util_critical_section_exit();
    foo = 9;
}

