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

//! \file
#ifndef _oslmic_h_
#define _oslmic_h_

// Dependencies required for the LoRa MAC in C to run.
// These settings can be adapted to the underlying system.
// You should not, however, change the lmic.[hc]

//================================================================================
//================================================================================
// Target platform as C library

#include <string.h>
#include "hal.h"
#define EV(a,b,c) /**/
#define DO_DEVDB(field1,field2) /**/

#define os_clearMem(a,b)   memset(a,0,b)
#define os_copyMem(a,b,c)  memcpy(a,b,c)

typedef     struct osjob_t osjob_t;
typedef      struct band_t band_t;
typedef   struct chnldef_t chnldef_t;
typedef   struct rxsched_t rxsched_t;
typedef   struct bcninfo_t bcninfo_t;
typedef        const u1_t* xref2cu1_t;
typedef              u1_t* xref2u1_t;
#define TYPEDEF_xref2rps_t     typedef         rps_t* xref2rps_t
#define TYPEDEF_xref2rxsched_t typedef     rxsched_t* xref2rxsched_t
#define TYPEDEF_xref2chnldef_t typedef     chnldef_t* xref2chnldef_t
#define TYPEDEF_xref2band_t    typedef        band_t* xref2band_t
#define TYPEDEF_xref2osjob_t   typedef       osjob_t* xref2osjob_t

#define SIZEOFEXPR(x) sizeof(x)

extern u4_t AESAUX[];
extern u4_t AESKEY[];
#define AESkey ((u1_t*)AESKEY)
#define AESaux ((u1_t*)AESAUX)
#define _FUNC_ADDR(func) (&(func))

u1_t radio_rand1 (void);
#define os_getRndU1() radio_rand1()

#define DEFINE_LMIC  struct lmic_t LMIC
#define DECLARE_LMIC extern struct lmic_t LMIC

void radio_init (void);
#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)
void radio_irq_handler (u1_t dio);
#elif defined(CFG_sx126x_radio)
void radio_irq_handler (void);
#endif

//================================================================================


#ifndef RX_RAMPUP_us
#define RX_RAMPUP_us    2000
#endif
#ifndef TX_RAMPUP_us
#define TX_RAMPUP_us    2000
#endif


#ifndef HAS_os_calls

#ifndef os_getDevKey
//void os_getDevKey (xref2u1_t buf);
#endif
#ifndef os_getArtEui
//void os_getArtEui (xref2u1_t buf);
#endif
#ifndef os_getDevEui
void os_getDevEui (xref2u1_t buf);
#endif
//#if 0
//#endif /* if 0 */
#ifndef os_radio
void os_radio (u1_t mode);
#endif
unsigned radio_get_symbol_period(void);
#ifndef os_getBattLevel
u1_t os_getBattLevel (void);
#endif

#ifndef os_rlsbf4
//! Read 32-bit quantity from given pointer in little endian byte order.
u4_t os_rlsbf4 (xref2cu1_t buf);
#endif
#ifndef os_wlsbf4
//! Write 32-bit quntity into buffer in little endian byte order.
void os_wlsbf4 (xref2u1_t buf, u4_t value);
#endif
#ifndef os_rmsbf4
//! Read 32-bit quantity from given pointer in big endian byte order.
u4_t os_rmsbf4 (xref2cu1_t buf);
#endif
#ifndef os_wmsbf4
//! Write 32-bit quntity into buffer in big endian byte order.
void os_wmsbf4 (xref2u1_t buf, u4_t value);
#endif
#ifndef os_rlsbf2
//! Read 16-bit quantity from given pointer in little endian byte order.
u2_t os_rlsbf2 (xref2cu1_t buf);
#endif
#ifndef os_wlsbf2
//! Write 16-bit quntity into buffer in little endian byte order.
void os_wlsbf2 (xref2u1_t buf, u2_t value);
#endif

//! Get random number (default impl for u2_t).
#ifndef os_getRndU2
#define os_getRndU2() ((u2_t)((os_getRndU1()<<8)|os_getRndU1()))
#endif
#ifndef os_crc16
u2_t os_crc16 (xref2u1_t d, uint len);
#endif

#endif // !HAS_os_calls

// ======================================================================
// AES support 
// !!Keep in sync with lorabase.hpp!!

#ifndef AES_ENC  // if AES_ENC is defined as macro all other values must be too
#define AES_ENC       0x00 
#define AES_DEC       0x01
#define AES_MIC       0x02
#define AES_CTR       0x04
#define AES_MICNOAUX  0x08
#endif
#ifndef AESkey  // if AESkey is defined as macro all other values must be too
extern xref2u1_t AESkey;
extern xref2u1_t AESaux;
#endif
#ifndef os_aes
u4_t os_aes (u1_t mode, xref2u1_t buf, u2_t len);
#endif

typedef union {
    struct {
        uint8_t header;
        uint16_t confFCnt;
        uint8_t dr;
        uint8_t ch;
        uint8_t dir;
        uint32_t DevAddr;
        uint32_t FCnt;
        uint8_t zero8;
        uint8_t lenMsg;
    } __attribute__((packed)) b;
    uint8_t octets[16];
} block_t;


#endif // _oslmic_h_
