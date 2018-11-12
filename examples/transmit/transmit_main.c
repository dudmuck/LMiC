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

#ifdef MANUAL_UPLINK
void console()
{
    debug_char('\r');
    debug_char('\n');

    if (_pcbuf_len == 1) {
        if (pcbuf[0] == 'L') {
            printf("LinkCheckReq\r\n");
            LMIC.flags.LinkCheckReq = 1;
            LMIC_setTxData();
        } else if (pcbuf[0] == '?') {
            printf("L                 link check\r\n");
            printf("up<payload>       send uplink\r\n");
        } else if (pcbuf[0] == '.') {
            printf("opmode:%04x\r\n", LMIC.opmode);
            printf("txrxFlags:%02x\r\n", LMIC.txrxFlags);
        }
    } else if (_pcbuf_len >= 2) {
        if (pcbuf[0] == 'u' && pcbuf[1] == 'p') {
            printf("LMIC_setTxData2\r\n");
            LMIC_setTxData2(20, pcbuf+2, _pcbuf_len-2, false);
        }
#if defined(JOINEUI) && defined(OPTNEG)
        else if (pcbuf[0] == 'r' && (pcbuf[1] == '0' || pcbuf[1] == '1' )) {
            printf("LMIC_tryRejoin\r\n");
            LMIC_tryRejoin(pcbuf[1] - '0');
        }
#endif /* JOINEUI && OPTNEG */
    }


    debug_char('>');
    debug_char(' ');
}
#endif /* MANUAL_UPLINK */


//////////////////////////////////////////////////
// MAIN - INITIALIZATION AND STARTUP
//////////////////////////////////////////////////

// application entry point
int main () {
    bit_t ok;
    // initialize runtime env
    LMIC_init();

    // reset MAC state
    LMIC_reset();
    // start joining
#if defined(JOINEUI) && defined(ROOT_NWKKEY)
    /* OTA */
    ok = LMIC_startJoining();
    printf("joining: %u\r\n", ok);
#elif defined(DEVADDR)
    /* ABP */
    LMIC_setSession();

    #ifndef MANUAL_UPLINK
    /* start first uplink */
    LMIC.frame[0] = 0;
    LMIC_setTxData2(1, LMIC.frame, 1, false);
    #endif /* !MANUAL_UPLINK */
#endif

    #ifdef MANUAL_UPLINK
    sleep_manager_lock_deep_sleep();    // prevent deep sleep, for UART-rx
    #endif

    for (;;) {
        LMIC_mainloop();
#ifdef MANUAL_UPLINK
        if (_pcbuf_len != 0) {
            console();
            _pcbuf_len = 0;
        }
#endif /* MANUAL_UPLINK */
    }

    return 0;
}

//////////////////////////////////////////////////
// LMIC EVENT CALLBACK
//////////////////////////////////////////////////

u1_t onEvent (ev_t ev, const void* args)
{
#ifdef JOINEUI
    u1_t devEui[8];
#endif /* JOINEUI */
    u1_t stop = 0;
#ifndef MANUAL_UPLINK
    static u1_t cnt;
#endif /* !MANUAL_UPLINK */
    debug_event(ev);

    switch(ev) {
#ifdef JOINEUI
        // network joined, session established
        case EV_JOINED:
            debug_val("netid = ", LMIC.netid);
            printf("devAddr:%08x\r\n", LMIC.devaddr);
#ifndef MANUAL_UPLINK
            cnt = 0;
#endif /* !MANUAL_UPLINK */
            goto tx;
        case EV_REJOIN_FAILED:
        case EV_JOIN_FAILED:
            stop = 1;
            break;
#endif /* JOINEUI */

            // scheduled data sent (optionally data received)
        case EV_TXCOMPLETE:
            printf("datalen:%u ",LMIC.dataLen);
            if(LMIC.dataLen) { // data received in rx slot after tx
                debug_buf("frame", LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
            }
            tx:
#ifndef MANUAL_UPLINK
            {
            bit_t conf = (cnt++ & 3) == 3;
            // immediately prepare next transmission
            LMIC.frame[0] = LMIC.snr;
            // schedule transmission (port 1, datalen 1, no ack requested)
            printf("cnt%u ", cnt);
            if (conf)
                printf("Conf ");
            else
                printf("Unconf ");
            LMIC_setTxData2(1, LMIC.frame, 1, conf);
            // (will be sent as soon as duty cycle permits)
            }
#endif /* !MANUAL_UPLINK */
            break;
        case EV_SCAN_TIMEOUT:
        case EV_BEACON_FOUND:
        case EV_BEACON_MISSED:
        case EV_BEACON_TRACKED:
        case EV_RFU1:
        case EV_LOST_TSYNC:
        case EV_RESET:
        case EV_RXCOMPLETE:
        case EV_LINK_DEAD:
        case EV_LINK_ALIVE:
        case EV_SCAN_FOUND:
            break;
        case EV_TXSTART:
            printf("tx %uhz ch%u dr%u\r\n", LMIC.freq, LMIC.txChnl, LMIC.datarate);
            break;
#ifdef JOINEUI
        case EV_JOINING:
            os_getDevEui(devEui);
            for (unsigned n = 0; n < 8; n++)
                printf("%02x ", devEui[n]);
            printf("\r\n");
            break;
#endif /* JOINEUI */
        case EV_LINKCHECK_ANS: {
            const u1_t* ans = args;
            printf("margin:%u, gwCnt:%u\r\n", ans[0], ans[1]);
            }
            break;
        case EV_DEVICE_TIME_ANS: {
            const u1_t* ans = args;
            u4_t secs = os_rlsbf4(ans);
            u4_t subsecs = ans[4];
            printf("%u.%u\r\n", secs, subsecs);
            }
            break;
    }

    return stop;
}

