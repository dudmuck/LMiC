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

#ifndef _hal_hpp_
#define _hal_hpp_

#include "types.h"

// hal:
#include "power_mgmt.h"
#include "lpTimeout.h"
#include "assert.h"
#include "wait_api.h"
#include "mbed_critical.h"
#include "utilities.h"
#include "sleep_api.h"
#include "loraconfig.h"
/*
 * initialize hardware (IO, SPI, TIMER, IRQ).
 */
void _hal_init(void);

#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)
/*
 * control radio RST pin (0=low, 1=high, 2=floating)
 */

void hal_pin_rst (u1_t val);
/*
 * drive radio RX/TX pins (-1=off, 0=rx, 1=txRFO, 2=txPABOOST).
 */
void hal_pin_antsw(s1_t val);

#elif defined(CFG_sx126x_radio)
void hal_sx126x_init(void);
void hal_pin_antsw(u1_t val);
#endif /* CFG_sx1276_radio || CFG_sx1272_radio */


void hal_pin_dbg(u1_t val);
void hal_pin_dbg_toggle(void);
void hal_dbg_pin_rx(bool);
void hal_dbg_pin_latency(bool);

/*
 * perform fatal failure action.
 *   - called by assertions
 *   - action could be HALT or reboot
 */
void hal_failed (void);

void getHWDevEui(u1_t* devEui);

#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)
typedef enum {
    TX_PIN_RFO,
    TX_PIN_PABOOST,
    TX_PIN_BOTH
} txpin_e;

extern txpin_e txpin;
#endif /* CFG_sx1276_radio || CFG_sx1272_radio */

typedef enum {
#if defined(JOINEUI) && defined(ROOT_APPKEY)
    NVM_DEVNONCE,
    NVM_RJCOUNT1,
    NVM_JOINNONCE,
#elif defined(DEVADDR)
    NVM_FCNTUP,
    NVM_NFCNTDOWN,
    NVM_AFCNTDOWN,
#else
    NVM_NONE
#endif
} nvm_e;
void nvm_init(void);
u4_t nvm_read(nvm_e);
void nvm_incr(nvm_e, uint32_t by);
void nvm_service(void);

#ifdef JOINEUI
    /* OTA */
    #define read_FCntUp           LMIC.FCntUp
    #define incr_FCntUp           LMIC.FCntUp++
    #define read_AFCntDown          LMIC.AFCntDown
    #define read_NFCntDown          LMIC.NFCntDown
    #define add_to_NFCntDown(x)    LMIC.NFCntDown += x
    #define add_to_AFCntDown(x)    LMIC.AFCntDown += x
#elif defined(DEVADDR)
    /* ABP */
    #define read_FCntUp           nvm_read(NVM_FCNTUP)
    #define incr_FCntUp           nvm_incr(NVM_FCNTUP, 1)
    #define read_NFCntDown         nvm_read(NVM_NFCNTDOWN)
    #define read_AFCntDown         nvm_read(NVM_AFCNTDOWN)
    #define add_to_NFCntDown(x)    nvm_incr(NVM_NFCNTDOWN, x)
    #define add_to_AFCntDown(x)    nvm_incr(NVM_AFCNTDOWN, x)
#else
    #define read_FCntUp           0
    #define incr_FCntUp           
    #define read_NFCntDown         0
    #define read_AFCntDown         0
    #define add_to_NFCntDown(x)    
    #define add_to_AFCntDown(x)    
#endif

#endif // _hal_hpp_

