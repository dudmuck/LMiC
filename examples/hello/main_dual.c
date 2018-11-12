/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "lmic.h"
#include "debug.h"
#include <stdio.h>
#include <stdlib.h>

// LMIC application callbacks not used in his example
void os_getArtEui (u1_t* buf) {
}

void os_getDevEui (u1_t* buf) {
}

void onEvent (ev_t ev) {
}

#if 0
extern volatile uint16_t cnt_at_cmp;
extern volatile bool cnt_at_cmp_new;

#define FAST_MS     167
#define SLOW_MS     500
#define LEWAY       4

static void fastFunc (osjob_t* job) {
    unsigned meas_ms;
    static ostime_t prev_now = -1;
    u4_t was_set_at = job->cmp_set_at;
    job->cmp_set_at = os_getTime();
    os_setTimedCallback(job, job->cmp_set_at + ms2osticks(FAST_MS), fastFunc);

    meas_ms = osticks2ms(job->cmp_set_at - prev_now);
    if (cnt_at_cmp_new) {
        cnt_at_cmp_new = false;
        printf("*");
    } else
        printf(" ");
    printf("   %08x from prev:%u   (%04x %04x :%d)\r\n", job->cmp_set_at, meas_ms, cnt_at_cmp, job->cmp_set_to, job->nearness);
    if (prev_now != -1 && (meas_ms < (FAST_MS-LEWAY) || meas_ms > (FAST_MS+LEWAY))) {
        printf("xxx   (%04x)cmp %04x was_set_at:%04x\r\n", cnt_at_cmp, job->cmp_set_to, was_set_at);

        for (;;) asm("nop");
    }
    prev_now = job->cmp_set_at;
}


static void slowFunc (osjob_t* job) {
    unsigned meas_ms;
    static ostime_t prev_now = -1;
    u4_t was_set_at = job->cmp_set_at;
    job->cmp_set_at = os_getTime();
    os_setTimedCallback(job, job->cmp_set_at + ms2osticks(SLOW_MS), slowFunc);

    meas_ms = osticks2ms(job->cmp_set_at - prev_now);
    if (cnt_at_cmp_new) {
        cnt_at_cmp_new = false;
        printf("*");
    } else
        printf(" ");
    printf("Slow %08x from prev:%u   (%04x %04x :%d)\r\n", job->cmp_set_at, meas_ms, cnt_at_cmp, job->cmp_set_to, job->nearness);
    if (prev_now != -1 && (meas_ms < (SLOW_MS-LEWAY) || meas_ms > (SLOW_MS+LEWAY))) {
        printf("XXX   (%04x)cmp %04x was_set_at:%04x\r\n", cnt_at_cmp, job->cmp_set_to, was_set_at);
        for (;;) asm("nop");
    }
    prev_now = job->cmp_set_at;
}

// application entry point
int main () {
    osjob_t slowJob;
    osjob_t fastJob;

    // initialize runtime env
    os_init();
    // initialize debug library
    debug_init();
    // setup initial job
    os_setCallback(&slowJob, slowFunc);
    os_setCallback(&fastJob, fastFunc);
    // execute scheduled jobs and events
    os_runloop();
    // (not reached)
    return 0;
}
#endif /* if 0 */

int randr(int lower, int upper)
{
    return (rand() % (upper - lower + 1)) + lower; 
}

//#define FAST_US     166667
#define FAST_US_MIN     100000
#define FAST_US_MAX     200000
#define SLOW_US     500000
ticker_t fast;
ticker_t slow;

us_timestamp_t fast_prev = (us_timestamp_t)-1;
unsigned fast_us;

void slow_cb()
{
    int diff;
    static us_timestamp_t prev = (us_timestamp_t)-1;
    us_timestamp_t now = LowPowerTimeout_read_us(&slow);
    LowPowerTimeout_attach_us(&slow, slow_cb, SLOW_US);
    diff = now - prev;
    printf("\tslow %d    %d    %u\r\n", diff, diff - SLOW_US, (unsigned)(now - fast_prev));
    prev = now;
}

void fast_cb()
{
    unsigned saved_fast_us;
    us_timestamp_t now = LowPowerTimeout_read_us(&fast);
    int diff;
    saved_fast_us = fast_us;
    fast_us = randr(FAST_US_MIN, FAST_US_MAX);
    LowPowerTimeout_attach_us(&fast, fast_cb, fast_us);
    diff = now - fast_prev;
    printf("%u ", fast_us);
    printf("fast %d    %d\r\n", diff, diff - saved_fast_us);
    fast_prev = now;
}

int main ()
{
    _hal_init();
    debug_init();

    //sleep_manager_lock_deep_sleep();    // prevent deep sleep
 
    printf("can deep:%d\r\n", sleep_manager_can_deep_sleep());

    LowPowerTimeout_attach_us(&slow, slow_cb, SLOW_US);
    fast_us = randr(FAST_US_MIN, FAST_US_MAX);
    LowPowerTimeout_attach_us(&fast, fast_cb, fast_us);
    for (;;) {
        asm("nop");
        sleep_manager_sleep_auto();
    }
}

