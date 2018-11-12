/** \addtogroup platform */
/** @{*/
/**
 * \defgroup platform_power_mgmt Power management functions
 * @{
 */

/* mbed Microcontroller Library
 * Copyright (c) 2006-2018 ARM Limited
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
#include <stdbool.h>

#define sleep_manager_lock_deep_sleep() \
    sleep_manager_lock_deep_sleep_internal()

#define sleep_manager_unlock_deep_sleep() \
    sleep_manager_unlock_deep_sleep_internal()

/** Lock the deep sleep mode
 *
 * This locks the automatic deep mode selection.
 * sleep_manager_sleep_auto() will ignore deepsleep mode if
 * this function is invoked at least once (the internal counter is non-zero)
 *
 * Use this locking mechanism for interrupt driven API that are
 * running in the background and deepsleep could affect their functionality
 *
 * The lock is a counter, can be locked up to USHRT_MAX
 * This function is IRQ and thread safe
 */
void sleep_manager_lock_deep_sleep_internal(void);

/** Unlock the deep sleep mode
 *
 * Use unlocking in pair with sleep_manager_lock_deep_sleep().
 *
 * The lock is a counter, should be equally unlocked as locked
 * This function is IRQ and thread safe
 */
void sleep_manager_unlock_deep_sleep_internal(void);

/** Enter auto selected sleep mode. It chooses the sleep or deeepsleep modes based
 *  on the deepsleep locking counter
 *
 * This function is IRQ and thread safe
 *
 * @note
 * If MBED_DEBUG is defined, only hal_sleep is allowed. This ensures the debugger
 * to be active for debug modes.
 *
 */
void sleep_manager_sleep_auto(void);

/** Get the status of deep sleep allowance for a target
 *
 * @return true if a target can go to deepsleep, false otherwise
 */
bool sleep_manager_can_deep_sleep(void);

