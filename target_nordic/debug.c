#include "debug.h"
#include "lmic.h"

#include "app_uart.h"

void debug_init()
{
    /* initialized in application layer */
}

void debug_uint (u4_t v) {
    for(s1_t n=24; n>=0; n-=8) {
        debug_hex(v>>n);
    }
}

void debug_hex (u1_t b) {
    debug_char("0123456789ABCDEF"[b>>4]);
    debug_char("0123456789ABCDEF"[b&0xF]);
}

void debug_char(char c)
{
    app_uart_put(c);
}

void debug_str (const char* str) {
    while(*str) {
        debug_char(*str++);
    }
}

void debug_buf (const char* const label, const u1_t* buf, int len)
{
    debug_str(label);
    debug_char(':');
    debug_char(' ');

    while(len--) {
        debug_hex(*buf++);
        debug_char(' ');
    }
    debug_char('\r');
    debug_char('\n');
}

void debug_val (const char* label, u4_t val) {
    debug_str(label);
    debug_uint(val);
    debug_char('\r');
    debug_char('\n');
}

void debug_event (int ev)
{
    static const char* const evnames[] = {
        [EV_SCAN_TIMEOUT]   = "SCAN_TIMEOUT",
        [EV_BEACON_FOUND]   = "BEACON_FOUND",
        [EV_BEACON_MISSED]  = "BEACON_MISSED",
        [EV_BEACON_TRACKED] = "BEACON_TRACKED",
#ifdef JOINEUI
        [EV_JOINING]        = "JOINING",
        [EV_JOINED]         = "JOINED",
        [EV_JOIN_FAILED]    = "JOIN_FAILED",
        [EV_REJOIN_FAILED]  = "REJOIN_FAILED",
#endif /* JOINEUI */
        [EV_RFU1]           = "RFU1",
        [EV_TXCOMPLETE]     = "TXCOMPLETE",
        [EV_LOST_TSYNC]     = "LOST_TSYNC",
        [EV_RESET]          = "RESET",
        [EV_RXCOMPLETE]     = "RXCOMPLETE",
        [EV_LINK_DEAD]      = "LINK_DEAD",
        [EV_LINK_ALIVE]     = "LINK_ALIVE",
        [EV_SCAN_FOUND]     = "SCAN_FOUND",
        [EV_TXSTART]        = "EV_TXSTART",
        [EV_LINKCHECK_ANS]  = "EV_LINKCHECK_ANS ",
        [EV_DEVICE_TIME_ANS]= "EV_DEVICE_TIME_ANS",
    };
    debug_str((ev < sizeof(evnames)/sizeof(evnames[0])) ? evnames[ev] : "EV_UNKNOWN" );
    debug_char('\r');
    debug_char('\n');
}
