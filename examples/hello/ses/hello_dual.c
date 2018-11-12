#include "main.h"
#include "lmic.h"
#include "debug.h"
#include <stdio.h>
#include <stdlib.h>

#define FAST_US_MIN     100000
#define FAST_US_MAX     200000
#define SLOW_US     500000
ticker_t fast;
ticker_t slow;

us_timestamp_t fast_prev = (us_timestamp_t)-1;
unsigned fast_us;

int randr(int lower, int upper)
{
    return (rand() % (upper - lower + 1)) + lower; 
}

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

void lorawan_app_init(void)
{
    // initialize runtime env
    _hal_init();
    // initialize debug library
    debug_init();
    // setup initial job

    printf("can deep:%d\r\n", sleep_manager_can_deep_sleep());

    LowPowerTimeout_attach_us(&slow, slow_cb, SLOW_US);
    fast_us = randr(FAST_US_MIN, FAST_US_MAX);
    LowPowerTimeout_attach_us(&fast, fast_cb, fast_us);
}

void lorawan_app_mainloop(void)
{
    sleep_manager_sleep_auto();
}


