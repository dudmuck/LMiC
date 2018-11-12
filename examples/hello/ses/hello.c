#include "main.h"
#include "lmic.h"
#include "debug.h"
#include <stdio.h>

void mycb()
{
    printf("mycb\r\n");
}

ticker_t test;

void lorawan_app_init(void)
{
    // initialize runtime env
    _hal_init();
    // initialize debug library
    debug_init();
    // setup initial job
    LowPowerTimeout_attach_us(&test, mycb, 1000000);

}

void lorawan_app_mainloop(void)
{
    sleep_manager_sleep_auto();
}

