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

//////////////////////////////////////////////////
// CONFIGURATION (FOR APPLICATION CALLBACKS BELOW)
//////////////////////////////////////////////////

// application router ID (LSBF)
static const u1_t APPEUI[8]  = { 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34 };

// unique device ID (LSBF)
//static const u1_t DEVEUI[8]  = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

// device-specific AES key (derived from device EUI)
static const u1_t DEVKEY[16] = { 0xAB, 0x89, 0xEF, 0xCD, 0x23, 0x01, 0x67, 0x45, 0x54, 0x76, 0x10, 0x32, 0xDC, 0xFE, 0x98, 0xBA };
// AB89EFCD2301674554761032DCFE98BA 


//////////////////////////////////////////////////
// APPLICATION CALLBACKS
//////////////////////////////////////////////////

// provide application router ID (8 bytes, LSBF)
void os_getArtEui (u1_t* buf) {
    unsigned i;
    printf("JOINEUI:");
    for (i = 0; i < 8; i++)
        printf("%02x ", APPEUI[i]);
    printf("\r\n");

    memcpyr(buf, APPEUI, 8);
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui (u1_t* buf) {
    unsigned i;
#ifdef DEVEUI
    u1_t devEui[8] = DEVEUI;
#else
    u1_t devEui[8];
    getHWDevEui(devEui);
#endif
    printf("DEVEUI:");
    for (i = 0; i < 8; i++)
        printf("%02x ", devEui[i]);
    printf("\r\n");

    memcpyr(buf, devEui, 8);
}

// provide device key (16 bytes)
void os_getDevKey (u1_t* buf) {
    unsigned i;
    memcpy(buf, DEVKEY, 16);
    printf("DEVKEY:");
    for (i = 0; i < 16; i++)
        printf("%02x ", buf[i]);
    printf("\r\n");
}


//////////////////////////////////////////////////
// MAIN - INITIALIZATION AND STARTUP
//////////////////////////////////////////////////

// application entry point
int main ()
{
    bit_t ok;
    // initialize runtime env
    LMIC_init();

    // reset MAC state
    LMIC_reset();
    // start joining
    ok = LMIC_startJoining();
    printf("joining: %u\r\n", ok);

    for (;;)
        LMIC_mainloop();

    return 0;
}


//////////////////////////////////////////////////
// UTILITY JOB
//////////////////////////////////////////////////

//static osjob_t blinkjob;
ticker_t blink;

static void blinkfunc () {
    // reschedule blink job
    LowPowerTimeout_attach_us(&blink, blinkfunc, 100000);
}


//////////////////////////////////////////////////
// LMIC EVENT CALLBACK
//////////////////////////////////////////////////

void onEvent (ev_t ev)
{
    debug_event(ev);

    switch(ev) {

      // starting to join network
      case EV_JOINING:
          // start blinking
          blinkfunc();
          break;
          
      // network joined, session established
      case EV_JOINED:
          // cancel blink job
          //os_clearCallback(&blinkjob);
          LowPowerTimeout_detach(&blink);
          // switch on LED
          // (don't schedule any new actions)
          break;
    case EV_SCAN_TIMEOUT:
    case EV_BEACON_FOUND:
    case EV_BEACON_MISSED:
    case EV_BEACON_TRACKED:
    case EV_RFU1:
    case EV_JOIN_FAILED:
    case EV_REJOIN_FAILED:
    case EV_TXCOMPLETE:
    case EV_LOST_TSYNC:
    case EV_RESET:
    case EV_RXCOMPLETE:
    case EV_LINK_DEAD:
    case EV_LINK_ALIVE:
    case EV_SCAN_FOUND:
    case EV_TXSTART:
          break;
    }
}
