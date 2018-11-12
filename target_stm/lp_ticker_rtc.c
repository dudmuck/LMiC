/* mbed Microcontroller Library
 *******************************************************************************
 * Copyright (c) 2018, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */

#include "rtc_api_hal.h"

const ticker_info_t *lp_ticker_get_info()
{
    static const ticker_info_t info = {
        RTC_CLOCK / 4, // RTC_WAKEUPCLOCK_RTCCLK_DIV4
        32
    };
    return &info;
}

void lp_ticker_init(void)
{
    rtc_init();
    lp_ticker_disable_interrupt();
}

uint32_t lp_ticker_read(void)
{
    return rtc_read_lp();
}

void lp_ticker_set_interrupt(timestamp_t timestamp)
{
    lp_ticker_disable_interrupt();
    rtc_set_wake_up_timer(timestamp);
}

void lp_ticker_fire_interrupt(void)
{
    rtc_fire_interrupt();
}

void lp_ticker_disable_interrupt(void)
{
    rtc_deactivate_wake_up_timer();
}

void lp_ticker_clear_interrupt(void)
{
    lp_ticker_disable_interrupt();
}

void lp_ticker_free(void)
{
    lp_ticker_disable_interrupt();
}

