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
#include "lmic.h"
#include "debug.h"
#include <stdio.h>

#if !defined(MINRX_SYMS)
#define MINRX_SYMS 5
#endif // !defined(MINRX_SYMS)
#define PAMBL_SYMS 8
#define PAMBL_FSK  5
#define PRERX_FSK  1
#define RXLEN_FSK  (1+5+2)

#if DEVADDR
DEFINE_LMIC = {
    #ifdef OPTNEG
        .SNwkSIntKey = SNwkSIntKey,
        .NwkSEncKey = NwkSEncKey,
        .FNwkSIntKey = FNwkSIntKey,
    #else
        .SNwkSIntKey = nwkSKey,
        .NwkSEncKey = nwkSKey,
        .FNwkSIntKey = nwkSKey,
    #endif

    .AppSKey = appSKey
};
#else
DEFINE_LMIC;
#endif

// Fwd decls.
static void engineUpdate();
static void startScan (void);

// ================================================================================
// BEG OS - default implementations for certain OS suport functions

#if !defined(HAS_os_calls)

#if !defined(os_rlsbf2)
u2_t os_rlsbf2 (xref2cu1_t buf) {
    return (u2_t)(buf[0] | (buf[1]<<8));
}
#endif

#if !defined(os_rlsbf4)
u4_t os_rlsbf4 (xref2cu1_t buf) {
    return (u4_t)(buf[0] | (buf[1]<<8) | ((u4_t)buf[2]<<16) | ((u4_t)buf[3]<<24));
}
#endif


#if !defined(os_rmsbf4)
u4_t os_rmsbf4 (xref2cu1_t buf) {
    return (u4_t)(buf[3] | (buf[2]<<8) | ((u4_t)buf[1]<<16) | ((u4_t)buf[0]<<24));
}
#endif


#if !defined(os_wlsbf2)
void os_wlsbf2 (xref2u1_t buf, u2_t v) {
    buf[0] = v;
    buf[1] = v>>8;
}
#endif

#if !defined(os_wlsbf3)
void os_wlsbf3 (xref2u1_t buf, u4_t v) {
    buf[0] = v;
    buf[1] = v>>8;
    buf[2] = v>>16;
}
#endif

#if !defined(os_wlsbf4)
void os_wlsbf4 (xref2u1_t buf, u4_t v) {
    buf[0] = v;
    buf[1] = v>>8;
    buf[2] = v>>16;
    buf[3] = v>>24;
}
#endif

#if !defined(os_wmsbf4)
void os_wmsbf4 (xref2u1_t buf, u4_t v) {
    buf[3] = v;
    buf[2] = v>>8;
    buf[1] = v>>16;
    buf[0] = v>>24;
}
#endif

#if !defined(os_getBattLevel)
u1_t os_getBattLevel (void) {
    return MCMD_DEVS_BATT_NOINFO;
}
#endif

#if !defined(os_crc16)
// New CRC-16 CCITT(XMODEM) checksum for beacons:
u2_t os_crc16 (xref2u1_t data, uint len) {
    u2_t remainder = 0;
    u2_t polynomial = 0x1021;
    for( uint i = 0; i < len; i++ ) {
        remainder ^= data[i] << 8;
        for( u1_t bit = 8; bit > 0; bit--) {
            if( (remainder & 0x8000) )
                remainder = (remainder << 1) ^ polynomial;
            else 
                remainder <<= 1;
        }
    }
    return remainder;
}
#endif

#endif // !HAS_os_calls

// END OS - default implementations for certain OS suport functions
// ================================================================================

#ifdef ROOT_NWKKEY
//static const u1_t devKey[16] = ROOT_NWKKEY;
static const u1_t root_nwkKey[16] = ROOT_NWKKEY;
#ifdef ROOT_APPKEY
static const u1_t root_appKey[16] = ROOT_APPKEY;
#endif

/*void os_getDevKey(u1_t* buf) {
    memcpy(buf, devKey, 16);
}*/
#endif /* ROOT_NWKKEY */

#ifdef JOINEUI

#ifdef OPTNEG
static const u1_t joinEui[8] = JOINEUI;
#endif

void os_getDevEui (u1_t* buf)
{
#ifdef DEVEUI
    u1_t devEui[8] = DEVEUI;
#else
    u1_t devEui[8];
    getHWDevEui(devEui);
    #ifdef OPTNEG
    /* give 1.1 device an inverted DevEUI */
    for (unsigned i = 0; i < 8; i++)
        devEui[i] ^= 0xff;
    #endif
#endif

    memcpyr(buf, devEui, 8);
}
#endif /* JOINEUI */


// ================================================================================
// BEG AES

/*
static void micB0 (u4_t devaddr, u4_t seqno, int dndir, int len) {
    os_clearMem(AESaux,16);
    AESaux[0]  = 0x49;
    AESaux[5]  = dndir?1:0;
    AESaux[15] = len;
    os_wlsbf4(AESaux+ 6,devaddr);
    os_wlsbf4(AESaux+10,seqno);
}*/

static int aes_verifyMic (xref2cu1_t key, xref2u1_t pdu, unsigned len)
{
    os_copyMem(AESkey,key,16);
    return os_aes(AES_MIC, pdu, len) == os_rmsbf4(pdu+len);
}
/*
static int aes_verifyMic (xref2cu1_t key, u4_t devaddr, u4_t seqno, int dndir, xref2u1_t pdu, int len) {
    micB0(devaddr, seqno, dndir, len);
    os_copyMem(AESkey,key,16);
    return os_aes(AES_MIC, pdu, len) == os_rmsbf4(pdu+len);
}
*/

static void aes_appendMic (block_t* b, xref2u1_t pdu, unsigned len)
{
    os_copyMem(AESaux, b, 16);
    os_copyMem(AESkey, LMIC.FNwkSIntKey, 16);
#ifdef DEBUG_MIC_UP
    debug_buf("pdu", pdu, len);
#endif /* DEBUG_MIC_UP */

#ifdef OPTNEG
    u4_t cmacF, cmacS;

#ifdef DEBUG_MIC_UP
    debug_buf("fB0", AESaux, 16);
    debug_buf("LMIC.FNwkSIntKey", AESkey, 16);
    printf("dr%u ch%u ", LMIC.datarate, LMIC.txChnl);
#endif /* DEBUG_MIC_UP */

    cmacF = os_aes(AES_MIC, pdu, len) >> 16;

    b->b.confFCnt = LMIC.ConfFCntDown;
    b->b.dr = LMIC.datarate;
    b->b.ch = LMIC.txChnl;
    os_copyMem(AESaux, b, 16);

    os_copyMem(AESkey, LMIC.SNwkSIntKey, 16);

#ifdef DEBUG_MIC_UP
    debug_buf("sB0", AESaux, 16);
    debug_buf("LMIC.SNwkSIntKey", AESkey, 16);
#endif /* DEBUG_MIC_UP */

    cmacS = os_aes(AES_MIC, pdu, len) & 0xffff0000;

    os_wmsbf4(pdu+len, cmacF | cmacS);

#else
    /* LoRaWAN 1.0.x */
    u4_t mic = os_aes(AES_MIC, pdu, len);
#ifdef DEBUG_MIC_UP
    debug_buf("b0", AESaux, 16);
    debug_buf("LMIC.FNwkSIntKey", AESkey, 16);
    printf("mic:%08x\r\n", mic);
#endif /* DEBUG_MIC_UP */
    // MSB because of internal structure of AES
    os_wmsbf4(pdu+len, mic);
#endif /* !OPTNEG */
}
/*
static void aes_appendMic (xref2cu1_t key, u4_t devaddr, u4_t seqno, int dndir, xref2u1_t pdu, int len) {
    micB0(devaddr, seqno, dndir, len);
    os_copyMem(AESkey,key,16);
    // MSB because of internal structure of AES
    os_wmsbf4(pdu+len, os_aes(AES_MIC, pdu, len));
}
*/

#ifdef JOINEUI
static void aes_appendMic0 (const u1_t* const key, xref2u1_t pdu, int len) {
    memcpy(AESkey, key, 16);
    os_wmsbf4(pdu+len, os_aes(AES_MIC|AES_MICNOAUX, pdu, len));  // MSB because of internal structure of AES
}

static int aes_verifyMic0 (xref2u1_t pdu, int len) {
    memcpy(AESkey, root_nwkKey, 16);
    return os_aes(AES_MIC|AES_MICNOAUX, pdu, len) == os_rmsbf4(pdu+len);
}

static void aes_encrypt (xref2u1_t pdu, int len)
{
#ifdef OPTNEG
    if (LMIC.JoinReqType == 0xff)
        memcpy(AESkey, root_nwkKey, 16);
    else
        memcpy(AESkey, LMIC.JSEncKey, 16);
#else
    memcpy(AESkey, root_nwkKey, 16);
#endif

    debug_buf("AESkey", AESkey, 16);
    os_aes(AES_ENC, pdu, len);
}
#endif /* JOINEUI */


/* aes_cipher() */
static void aes_cipher (u1_t ctr, xref2cu1_t key, u4_t devaddr, u4_t seqno, int dndir, xref2u1_t payload, int len)
{
    if( len <= 0 )
        return;
    os_clearMem(AESaux, 16);
    AESaux[0] = 1; // mode=cipher / dir=down
    AESaux[15] = ctr; // block counter
    AESaux[5] = dndir?1:0;
    os_wlsbf4(AESaux+ 6,devaddr);
    os_wlsbf4(AESaux+10,seqno);
    os_copyMem(AESkey,key,16);
    os_aes(AES_CTR, payload, len);
}

#ifdef JOINEUI

#ifdef OPTNEG
typedef union {
    struct {
        uint8_t mhdr;
        unsigned int joinNonce : 24;
        unsigned int Home_NetID : 24;
        uint32_t DevAddr;
        struct {
            uint8_t RX2dr       : 4;  // 0,1,2,3
            uint8_t RX1DRoffset : 3;  // 4,5,6
            uint8_t OptNeg      : 1;  // 7
        } DLSettings;
        uint8_t RxDelay;
    } __attribute__((packed)) fields;
    uint8_t octets[13];
} joinAccept_t;

static void aes_sessKeys_1v1 (u4_t jn)
{
    u1_t buf[16];
    u1_t* ptr = buf;

    memset(ptr, 0, 16);
    memcpy(++ptr, &jn, 3);
    ptr += 3;
    memcpyr(ptr, joinEui, 8);
    ptr += 8;
    memcpy(ptr, &LMIC.devNonce, 2);
    ptr += 2;

    memcpy(LMIC.FNwkSIntKey, buf, 16);
    LMIC.FNwkSIntKey[0] = 0x01;
    //debug_buf("gen-FNwkSIntKey", LMIC.FNwkSIntKey, 16);
    memcpy(AESkey, root_nwkKey, 16);
    os_aes(AES_ENC, LMIC.FNwkSIntKey, 16);

    memcpy(LMIC.SNwkSIntKey, buf, 16);
    LMIC.SNwkSIntKey[0] = 0x03;
    //debug_buf("gen-SNwkSIntKey", LMIC.SNwkSIntKey, 16);
    memcpy(AESkey, root_nwkKey, 16);
    os_aes(AES_ENC, LMIC.SNwkSIntKey, 16);

    memcpy(LMIC.NwkSEncKey, buf, 16);
    LMIC.NwkSEncKey[0] = 0x04;
    //debug_buf("gen-NwkSEncKey", LMIC.NwkSEncKey, 16);
    memcpy(AESkey, root_nwkKey, 16);
    os_aes(AES_ENC, LMIC.NwkSEncKey, 16);

    //debug_buf("FNwkSIntKey", LMIC.FNwkSIntKey, 16);
    //debug_buf("NwkSEncKey", LMIC.NwkSEncKey, 16);
    //debug_buf("SNwkSIntKey", LMIC.SNwkSIntKey, 16);

    memcpy(LMIC.AppSKey, buf, 16);
    LMIC.AppSKey[0] = 0x02;
    //debug_buf("gen-AppSKey", LMIC.AppSKey, 16);
    memcpy(AESkey, root_appKey, 16);
    os_aes(AES_ENC, LMIC.AppSKey, 16);

    //debug_buf("AppSKey", LMIC.AppSKey, 16);
}
#endif /* OPTNEG */

//static void aes_sessKeys (u2_t devnonce, xref2cu1_t artnonce, xref2u1_t nwkkey, xref2u1_t artkey) {
static void aes_sessKeys_1v0 (void)
{
    os_clearMem(LMIC.FNwkSIntKey, 16);
    LMIC.FNwkSIntKey[0] = 0x01;
    os_copyMem(LMIC.FNwkSIntKey+1, &LMIC.frame[OFF_JA_ARTNONCE], LEN_ARTNONCE+LEN_NETID);
    os_wlsbf2(LMIC.FNwkSIntKey+1+LEN_ARTNONCE+LEN_NETID, LMIC.devNonce);
    os_copyMem(LMIC.AppSKey, LMIC.FNwkSIntKey, 16);
    LMIC.AppSKey[0] = 0x02;

    memcpy(AESkey, root_nwkKey, 16);
    os_aes(AES_ENC, LMIC.FNwkSIntKey, 16);
    memcpy(AESkey, root_nwkKey, 16);
    os_aes(AES_ENC, LMIC.AppSKey, 16);

    os_copyMem(LMIC.SNwkSIntKey, LMIC.FNwkSIntKey, 16);
    os_copyMem(LMIC.NwkSEncKey, LMIC.FNwkSIntKey, 16);
}
#endif /* JOINEUI */

// END AES
// ================================================================================


// ================================================================================
// BEG LORA

static const u1_t SENSITIVITY[7][3] = {
    // ------------bw----------
    // 125kHz    250kHz    500kHz
    { 141-109,  141-109, 141-109 },  // FSK
    { 141-127,  141-124, 141-121 },  // SF7
    { 141-129,  141-126, 141-123 },  // SF8
    { 141-132,  141-129, 141-126 },  // SF9
    { 141-135,  141-132, 141-129 },  // SF10
    { 141-138,  141-135, 141-132 },  // SF11
    { 141-141,  141-138, 141-135 }   // SF12
};

int getSensitivity (rps_t rps) {
    return -141 + SENSITIVITY[getSf(rps)][getBw(rps)];
}

#if 0
ostime_t calcAirTime (rps_t rps, u1_t plen) {
    u1_t bw = getBw(rps);  // 0,1,2 = 125,250,500kHz
    u1_t sf = getSf(rps);  // 0=FSK, 1..6 = SF7..12
    if( sf == FSK ) {
        return (plen+/*preamble*/5+/*syncword*/3+/*len*/1+/*crc*/2) * /*bits/byte*/8
            * (s4_t)OSTICKS_PER_SEC / /*kbit/s*/50000;
    }
    u1_t sfx = 4*(sf+(7-SF7));
    u1_t q = sfx - (sf >= SF11 ? 8 : 0);
    int tmp = 8*plen - sfx + 28 + (getNocrc(rps)?0:16) - (getIh(rps)?20:0);
    if( tmp > 0 ) {
        tmp = (tmp + q - 1) / q;
        tmp *= getCr(rps)+5;
        tmp += 8;
    } else {
        tmp = 8;
    }
    tmp = (tmp<<2) + /*preamble*/49 /* 4 * (8 + 4.25) */;
    // bw = 125000 = 15625 * 2^3
    //      250000 = 15625 * 2^4
    //      500000 = 15625 * 2^5
    // sf = 7..12
    //
    // osticks =  tmp * OSTICKS_PER_SEC * 1<<sf / bw
    //
    // 3 => counter reduced divisor 125000/8 => 15625
    // 2 => counter 2 shift on tmp
    sfx = sf+(7-SF7) - (3+2) - bw;
    int div = 15625;
    if( sfx > 4 ) {
        // prevent 32bit signed int overflow in last step
        div >>= sfx-4;
        sfx = 4;
    }
    // Need 32bit arithmetic for this last step
    return (((ostime_t)tmp << sfx) * OSTICKS_PER_SEC + div/2) / div;
}
#endif /* if 0 */

extern inline rps_t updr2rps (dr_t dr);
extern inline rps_t dndr2rps (dr_t dr);
extern inline int isFasterDR (dr_t dr1, dr_t dr2);
extern inline int isSlowerDR (dr_t dr1, dr_t dr2);
extern inline dr_t  incDR    (dr_t dr);
extern inline dr_t  decDR    (dr_t dr);
extern inline dr_t  assertDR (dr_t dr);
extern inline dr_t  validDR  (dr_t dr);
extern inline dr_t  lowerDR  (dr_t dr, u1_t n);

extern inline sf_t  getSf    (rps_t params);
extern inline rps_t setSf    (rps_t params, sf_t sf);
extern inline bw_t  getBw    (rps_t params);
extern inline rps_t setBw    (rps_t params, bw_t cr);
extern inline cr_t  getCr    (rps_t params);
extern inline rps_t setCr    (rps_t params, cr_t cr);
extern inline int   getNocrc (rps_t params);
extern inline rps_t setNocrc (rps_t params, int nocrc);
extern inline int   getIh    (rps_t params);
extern inline rps_t setIh    (rps_t params, int ih);
extern inline rps_t makeRps  (sf_t sf, bw_t bw, cr_t cr, int ih, int nocrc);
extern inline int   sameSfBw (rps_t r1, rps_t r2);

// END LORA
// ================================================================================


// Adjust DR for TX retries
//  - indexed by retry count
//  - return steps to lower DR
static const u1_t DRADJUST[2+TXCONF_ATTEMPTS] = {
    // normal frames - 1st try / no retry
    0,
    // confirmed frames
    0,0,1,0,1,0,1,0,0
};

// called from rxschedNext, rxschedInit, processBeacon
static us_timestamp_t calcRxWindow(u1_t secs, dr_t dr)
{
    us_timestamp_t rxoff, err;
    if( secs==0 ) {
        // aka 128 secs (next becaon)
        rxoff = LMIC.drift;
        err = LMIC.lastDriftDiff;
    } else {
        // scheduled RX window within secs into current beacon period
        rxoff = (LMIC.drift * secs) >> BCN_INTV_exp;
        err = (LMIC.lastDriftDiff * secs) >> BCN_INTV_exp;
    }
    u1_t rxsyms = MINRX_SYMS;
    err += LMIC.maxDriftDiff * LMIC.missedBcns;
    LMIC.rxsyms = MINRX_SYMS + (err / dr2hsym(dr));

    return (rxsyms-PAMBL_SYMS) * dr2hsym(dr) + rxoff;
}


// Setup beacon RX parameters assuming we have an error of ms (aka +/-(ms/2))
static void calcBcnRxWindowFromMicros (unsigned us, bit_t ini) {
    if( ini ) {
        LMIC.drift = 0;
        LMIC.maxDriftDiff = 0;
        LMIC.missedBcns = 0;
        LMIC.bcninfo.flags |= BCN_NODRIFT|BCN_NODDIFF;
    }
    us_timestamp_t hsym = dr2hsym(DR_BCN);
    LMIC.bcnRxsyms = MINRX_SYMS + us / hsym;
    LMIC.bcnRxtime_us = LMIC.bcninfo.txtime_us + BCN_INTV_us - (LMIC.bcnRxsyms-PAMBL_SYMS) * hsym;
}


// Setup scheduled RX window (ping/multicast slot)
static void rxschedInit (xref2rxsched_t rxsched) {
    os_clearMem(AESkey,16);
    os_clearMem(LMIC.frame+8,8);
    os_wlsbf4(LMIC.frame, LMIC.bcninfo.time);
    os_wlsbf4(LMIC.frame+4, LMIC.devaddr);
    os_aes(AES_ENC,LMIC.frame,16);
    u1_t intvExp = rxsched->intvExp;
    us_timestamp_t off_us = os_rlsbf2(LMIC.frame) & (0x0FFF >> (7 - intvExp)); // random offset (slot units)
    rxsched->rxbase_us = LMIC.bcninfo.txtime_us +
                       BCN_RESERVE_us +
                       (BCN_SLOT_SPAN_us * off_us); // random offset osticks
    rxsched->slot   = 0;
    rxsched->rxtime_us = rxsched->rxbase_us - calcRxWindow(/*secs BCN_RESERVE*/2+(1<<intvExp), rxsched->dr);
    rxsched->rxsyms = LMIC.rxsyms;
}


static bit_t rxschedNext (xref2rxsched_t rxsched, us_timestamp_t cando) {
  again:
    if( rxsched->rxtime_us - cando >= 0 )
        return 1;
    u1_t slot;
    if( (slot=rxsched->slot) >= 128 )
        return 0;
    u1_t intv = 1<<rxsched->intvExp;
    if( (rxsched->slot = (slot += (intv))) >= 128 )
        return 0;
    rxsched->rxtime_us = rxsched->rxbase_us
        + ((BCN_WINDOW_us * slot) >> BCN_INTV_exp)
        - calcRxWindow(/*secs BCN_RESERVE*/2+slot+intv, rxsched->dr);
    rxsched->rxsyms = LMIC.rxsyms;
    goto again;
}


us_timestamp_t rndDelay (u1_t secSpan) {
    u2_t r;
    r = (os_getRndU2() % secSpan) + 1;
    return r * 1000000;
}


static void txDelay (us_timestamp_t reftime_us, u1_t secSpan)
{
    reftime_us += rndDelay(secSpan);
    if( LMIC.globalDutyRate == 0  ||  (reftime_us - LMIC.globalDutyAvail_us) > 0 ) {
        LMIC.globalDutyAvail_us = reftime_us;
        LMIC.opmode |= OP_RNDTX;
    }
}


void setDrJoin (u1_t reason, u1_t dr)
{
    EV(drChange, INFO, (e_.reason    = reason,
                        e_.deveui    = MAIN::CDEV->getEui(),
                        e_.dr        = dr|DR_PAGE,
                        e_.txpow     = LMIC.adrTxPow,
                        e_.prevdr    = LMIC.datarate|DR_PAGE,
                        e_.prevtxpow = LMIC.adrTxPow));
    LMIC.datarate = dr;
    DO_DEVDB(LMIC.datarate,datarate);
}


static void setDrTxpow (u1_t reason, u1_t dr, s1_t pow) {
    EV(drChange, INFO, (e_.reason    = reason,
                        e_.deveui    = MAIN::CDEV->getEui(),
                        e_.dr        = dr|DR_PAGE,
                        e_.txpow     = pow,
                        e_.prevdr    = LMIC.datarate|DR_PAGE,
                        e_.prevtxpow = LMIC.adrTxPow));
    
    if( pow != KEEP_TXPOW )
        LMIC.adrTxPow = pow;

    if (LMIC.datarate != dr) {
        LMIC.datarate = dr;
        DO_DEVDB(LMIC.datarate,datarate);
        LMIC.opmode |= OP_NEXTCHNL;
    }
}


void LMIC_stopPingable (void) {
    LMIC.opmode &= ~(OP_PINGABLE|OP_PINGINI);
}


void LMIC_setPingable (u1_t intvExp) {
    // Change setting
    LMIC.ping.intvExp = (intvExp & 0x7);
    LMIC.opmode |= OP_PINGABLE;
    // App may call LMIC_enableTracking() explicitely before
    // Otherwise tracking is implicitly enabled here
    if( (LMIC.opmode & (OP_TRACK|OP_SCAN)) == 0  &&  LMIC.bcninfoTries == 0 )
        LMIC_enableTracking(0);
}

static void runEngineUpdate () {
    engineUpdate();
}

static void reportEvent (ev_t ev)
{
    u1_t stop;

    LMIC.opmode |= OP_ENGUP;
    EV(devCond, INFO, (e_.reason = EV::devCond_t::LMIC_EV,
                       e_.eui    = MAIN::CDEV->getEui(),
                       e_.info   = ev));

    stop = onEvent(ev, NULL);
    if (stop) {
        printf("stop ");
        if (LMIC.opmode & OP_JOINING) {
            LMIC.opmode &= ~OP_JOINING;
            printf("join-off ");
        }
        printf("\r\n");
    }
}


static void runReset () {
    // Disable session
    LMIC_reset();
#if defined(JOINEUI)
    LMIC_startJoining();
#endif
    reportEvent(EV_RESET);
}

static void stateJustJoined (void) {
#ifdef JOINEUI
    LMIC.joinTries   = 0;
    LMIC.NFCntDown     = LMIC.FCntUp = 0;
    #ifdef OPTNEG
    LMIC.AFCntDown     = LMIC.FCntUp;
    #endif
#endif /* JOINEUI */
    LMIC.dnConf      = LMIC.flags.adrChanged = LMIC.LinkADRAns = LMIC.flags.DevStatusAns = 0;
    /*LMIC.moreData    = */LMIC.RXParamSetupAns = LMIC.NewChannelAns = LMIC.flags.DutyCycleAns = 0;
    LMIC.PingSlotChannelAns = 0;
    LMIC.upRepeat    = 0;
    LMIC.adrAckReq   = LINK_CHECK_INIT;
    LMIC.dn2Dr       = DR_DNW2;
    LMIC.dn2Freq     = FREQ_DNW2;
    LMIC.bcnChnl     = CHNL_BCN;
    LMIC.ping.freq   = FREQ_PING;
    LMIC.ping.dr     = DR_PING;
    LMIC.ConfFCntDown = 0;
}


// ================================================================================
// Decoding frames


// Decode beacon  - do not overwrite bcninfo unless we have a match!
static int decodeBeacon (void)
{
    MBED_ASSERT(LMIC.dataLen == LEN_BCN); // implicit header RX guarantees this
    xref2u1_t d = LMIC.frame;

    if (!regionBeaconCrcCheck(d))
        return 0;   // first (common) part fails CRC check

    // First set of fields is ok
    u4_t bcnnetid = os_rlsbf4(&d[OFF_BCN_NETID]) & 0xFFFFFF;
    if( bcnnetid != LMIC.netid )
        return -1;  // not the beacon we're looking for

    LMIC.bcninfo.flags &= ~(BCN_PARTIAL|BCN_FULL);
    // Match - update bcninfo structure
    LMIC.bcninfo.snr    = LMIC.snr;
    LMIC.bcninfo.rssi   = LMIC.rssi;
    LMIC.bcninfo.txtime_us = LMIC.rxtime_us - AIRTIME_BCN;
    LMIC.bcninfo.time   = os_rlsbf4(&d[OFF_BCN_TIME]);
    LMIC.bcninfo.flags |= BCN_PARTIAL;

    // Check 2nd set
    if( os_rlsbf2(&d[OFF_BCN_CRC2]) != os_crc16(d,OFF_BCN_CRC2) )
        return 1;
    // Second set of fields is ok
    LMIC.bcninfo.lat    = (s4_t)os_rlsbf4(&d[OFF_BCN_LAT-1]) >> 8; // read as signed 24-bit
    LMIC.bcninfo.lon    = (s4_t)os_rlsbf4(&d[OFF_BCN_LON-1]) >> 8; // ditto
    LMIC.bcninfo.info   = d[OFF_BCN_INFO];
    LMIC.bcninfo.flags |= BCN_FULL;
    return 2;
}

void _rejoin_retry()
{
}

static bit_t decodeFrame (void)
{
    block_t* bp;
    u4_t FCntDown;
    xref2u1_t d = LMIC.frame;
    u1_t hdr    = d[0];
    u1_t ftype  = hdr & HDR_FTYPE;
    int  dlen   = LMIC.dataLen;
    if( dlen < OFF_DAT_OPTS+4 ||
        (hdr & HDR_MAJOR) != HDR_MAJOR_V1 ||
        (ftype != HDR_FTYPE_DADN  &&  ftype != HDR_FTYPE_DCDN) ) {
        // Basic sanity checks failed
        printf("basicFail ");
        EV(specCond, WARN, (e_.reason = EV::specCond_t::UNEXPECTED_FRAME,
                            e_.eui    = MAIN::CDEV->getEui(),
                            e_.info   = dlen < 4 ? 0 : os_rlsbf4(&d[dlen-4]),
                            e_.info2  = hdr + (dlen<<8)));
      norx:
        LMIC.dataLen = 0;
        return 0;
    }
    // Validate exact frame length
    // Note: device address was already read+evaluated in order to arrive here.
    int  fct   = d[OFF_DAT_FCT];
    u4_t addr  = os_rlsbf4(&d[OFF_DAT_ADDR]);
    u4_t seqno = os_rlsbf2(&d[OFF_DAT_SEQNO]);
    int  olen  = fct & FCT_OPTLEN;
    int  ackup = (fct & FCT_ACK) != 0 ? 1 : 0;   // ACK last up frame
    int  poff  = OFF_DAT_OPTS+olen;
    int  pend  = dlen-4;  // MIC

    if (addr != LMIC.devaddr) {
        /* not for this end-device */
        EV(specCond, WARN, (e_.reason = EV::specCond_t::ALIEN_ADDRESS,
                            e_.eui    = MAIN::CDEV->getEui(),
                            e_.info   = addr,
                            e_.info2  = LMIC.devaddr));
        goto norx;
    }
    if (poff > pend) {
        /* bad packet */
        EV(specCond, ERR, (e_.reason = EV::specCond_t::CORRUPTED_FRAME,
                           e_.eui    = MAIN::CDEV->getEui(),
                           e_.info   = 0x1000000 + (poff-pend) + (fct<<8) + (dlen<<16)));
        goto norx;
    }

    int FPort = -1;
    int replayConf = 0;

    //printf("pend%u poff%u ", pend, poff);
    if (pend > poff)
        FPort = d[poff++];

    //printf(" FPort:%d ", FPort);

#ifdef OPTNEG
    if (FPort > 0)
        FCntDown = read_AFCntDown;
    else
        FCntDown = read_NFCntDown;
#else
    FCntDown = read_NFCntDown;
#endif
    // lower 16bits come from downlink, upper 16bits from memory
    seqno |= (FCntDown & 0xffff0000);

    bp = (block_t*)AESaux;

    bp->b.header = 0x49;
    bp->b.confFCnt = LMIC.ConfFCntUp;
    bp->b.dr = 0;
    bp->b.ch = 0;
    bp->b.dir = DOWN_LINK;
    bp->b.DevAddr = LMIC.devaddr;
    bp->b.FCnt = seqno;
    bp->b.zero8 = 0;
    bp->b.lenMsg = pend;

    if( !aes_verifyMic(LMIC.SNwkSIntKey, d, pend) ) {
        /* mic fail */
        EV(spe3Cond, ERR, (e_.reason = EV::spe3Cond_t::CORRUPTED_MIC,
                           e_.eui1   = MAIN::CDEV->getEui(),
                           e_.info1  = Base::lsbf4(&d[pend]),
                           e_.info2  = seqno,
                           e_.info3  = LMIC.devaddr));

        printf("micFail %08x ", os_rmsbf4(d+pend));
        goto norx;
    }
    if( seqno < FCntDown ) {
        printf("seqno<FcntDown ");
        if( (s4_t)seqno > (s4_t)FCntDown ) {
            EV(specCond, INFO, (e_.reason = EV::specCond_t::DNSEQNO_ROLL_OVER,
                                e_.eui    = MAIN::CDEV->getEui(),
                                e_.info   = FCntDown, 
                                e_.info2  = seqno));
            printf("rollover ");
            goto norx;
        }
        // or ftype isnt confirmed downlink
        if( seqno != FCntDown-1 || !LMIC.dnConf || ftype != HDR_FTYPE_DCDN ) {
            EV(specCond, INFO, (e_.reason = EV::specCond_t::DNSEQNO_OBSOLETE,
                                e_.eui    = MAIN::CDEV->getEui(),
                                e_.info   = FCntDown, 
                                e_.info2  = seqno));
            printf("(replay rx%u have%u) ", seqno, FCntDown);
            goto norx;
        }
        // Replay of previous sequence number allowed only if
        // previous frame and repeated both requested confirmation
        replayConf = 1;
    }
    else {
        if( seqno > FCntDown ) {
            EV(specCond, INFO, (e_.reason = EV::specCond_t::DNSEQNO_SKIP,
                                e_.eui    = MAIN::CDEV->getEui(),
                                e_.info   = FCntDown, 
                                e_.info2  = seqno));
        }

        // next number to be expected
#ifdef OPTNEG
        if (LMIC.flags.OptNeg && FPort > 0) {
            add_to_AFCntDown(seqno+1 - FCntDown);
            DO_DEVDB(LMIC.AFCntDown, FCntDown);
        } else {
            add_to_NFCntDown(seqno+1 - FCntDown);
            DO_DEVDB(LMIC.NFCntDown, FCntDown);
        }
#else
        add_to_NFCntDown(seqno+1 - FCntDown);
        DO_DEVDB(LMIC.NFCntDown, FCntDown);
#endif
        // DN frame requested confirmation - provide ACK once with next UP frame
        LMIC.dnConf = (ftype == HDR_FTYPE_DCDN ? FCT_ACK : 0);
        LMIC.ConfFCntDown = 0;
#ifdef OPTNEG
        if (LMIC.flags.OptNeg && ftype == HDR_FTYPE_DCDN)
            LMIC.ConfFCntDown = seqno;
#endif
    }

    if( LMIC.dnConf || (fct & FCT_MORE) )
        LMIC.opmode |= OP_POLL;

    // We heard from network
    LMIC.flags.adrChanged = 0;
#ifdef JOINEUI
    LMIC.joinTries = 0;
#endif /* JOINEUI */
    if( LMIC.adrAckReq != LINK_CHECK_OFF )
        LMIC.adrAckReq = LINK_CHECK_INIT;

    // Process OPTS
    int m = LMIC.rssi - RSSI_OFF - getSensitivity(LMIC.rps);
    LMIC.margin = m < 0 ? 0 : m > 254 ? 254 : m;

    xref2u1_t opts = &d[OFF_DAT_OPTS];
#ifdef OPTNEG
    if (LMIC.flags.OptNeg && olen > 0) {
        u4_t fcnt;   // sequence number for FOpts decryption
        if (FPort > 0)
            fcnt = read_NFCntDown;
        else
            fcnt = seqno;

        aes_cipher(0, LMIC.NwkSEncKey, LMIC.devaddr, fcnt, DOWN_LINK, opts, olen);
    }
#endif

    //printf("olen%u ", olen);

    int oidx = 0;
    while( oidx < olen ) {
        //printf("MCMD:%02x ", opts[oidx]);
        switch( opts[oidx] ) {
        case LinkCheckAns: {
            LMIC.flags.LinkCheckReq = 0;
            //int gwmargin = opts[oidx+1];
            //int ngws = opts[oidx+2];
            onEvent(EV_LINKCHECK_ANS, opts + oidx + 1);
            oidx += 3;
            continue;
        }
        case LinkADRReq: {
            u1_t p1     = opts[oidx+1];            // txpow + DR
            u2_t chmap  = os_rlsbf2(&opts[oidx+2]);// list of enabled channels
            u1_t chpage = (opts[oidx+4] & MCMD_LADR_CHPAGE_MASK) >> MCMD_LADR_CHPAGE_SHIFT;     // channel page ChMaskCntl
            u1_t NbTrans  = opts[oidx+4] & MCMD_LADR_REPEAT_MASK;     // up repeat count
            oidx += 5;

            printf(" LinkADRReq %04x page%u ", chmap, chpage);

            LMIC.LinkADRAns = 0x80 |     // Include an answer into next frame up
                MCMD_LADR_ANS_POWACK | MCMD_LADR_ANS_CHACK | MCMD_LADR_ANS_DRACK;
            if (!mapChannels(chpage, chmap)) {
                printf("badMap ");
                LMIC.LinkADRAns &= ~MCMD_LADR_ANS_CHACK;
            }
            dr_t dr = (dr_t)(p1>>MCMD_LADR_DR_SHIFT);
            printf("dr%u ", dr);
            if (!validDR(dr)) {
                LMIC.LinkADRAns &= ~MCMD_LADR_ANS_DRACK;
                printf("badDR ");
                EV(specCond, ERR, (e_.reason = EV::specCond_t::BAD_MAC_CMD,
                                   e_.eui    = MAIN::CDEV->getEui(),
                                   e_.info   = Base::lsbf4(&d[pend]),
                                   e_.info2  = Base::msbf4(&opts[oidx-4])));
            }
            if( (LMIC.LinkADRAns & 0x7F) == (MCMD_LADR_ANS_POWACK | MCMD_LADR_ANS_CHACK | MCMD_LADR_ANS_DRACK) ) {
                printf(" ok\r\n");
                // Nothing went wrong - use settings
                LMIC.upRepeat = NbTrans;
                setDrTxpow(DRCHG_NWKCMD, dr, pow2dBm(p1));
            } else
                printf(" bad\r\n");
            LMIC.flags.adrChanged = 1;  // Trigger an ACK to NWK
            continue;
        }
        case DevStatusReq: {
            LMIC.flags.DevStatusAns = 1;
            oidx += 1;
            continue;
        }
        case RXParamSetupReq: {
            dr_t dr = (dr_t)(opts[oidx+1] & 0x0F);
            u1_t RX1DRoffset = opts[oidx+1] >> 4;
            u4_t freq = convFreq(&opts[oidx+2]);
            oidx += 5;
            LMIC.RXParamSetupAns = 0x80;   // answer pending
            if( validDR(dr) )
                LMIC.RXParamSetupAns |= MCMD_DN2P_ANS_DRACK;
            if( freq != 0 )
                LMIC.RXParamSetupAns |= MCMD_DN2P_ANS_CHACK;

            LMIC.RXParamSetupAns |= MCMD_DN2P_ANS_RX1OFSACK;
            
            if( LMIC.RXParamSetupAns == (0x80|MCMD_DN2P_ANS_RX1OFSACK|MCMD_DN2P_ANS_DRACK|MCMD_DN2P_ANS_CHACK) ) {
                LMIC.dn2Dr = dr;
                LMIC.dn2Freq = freq;
                LMIC.RX1DRoffset = RX1DRoffset;
                DO_DEVDB(LMIC.dn2Dr,dn2Dr);
                DO_DEVDB(LMIC.dn2Freq,dn2Freq);
            }
            continue;
        }
        case DutyCycleReq: {
            u1_t cap = opts[oidx+1];
            oidx += 2;
            // A value cap=0xFF means device is OFF unless enabled again manually.
            if( cap==0xFF )
                LMIC.opmode |= OP_SHUTDOWN;  // stop any sending
            LMIC.globalDutyRate  = cap & 0xF;
            LMIC.globalDutyAvail_us = LowPowerTimeout_read_us(&LMIC.ticker);
            DO_DEVDB(cap,dutyCap);
            LMIC.flags.DutyCycleAns = 1;
            continue;
        }
        case RXTimingSetupReq: {
            LMIC.rxDelay_sec = opts[++oidx];
            if (LMIC.rxDelay_sec == 0)
                LMIC.rxDelay_sec = 1;
            printf("rxDelay:%u\r\n", LMIC.rxDelay_sec);

            LMIC.flags.RXTimingSetupAns = 1;
            continue;
        }
        case NewChannelReq: {
            u1_t chidx = opts[oidx+1];  // channel
            u4_t freq  = convFreq(&opts[oidx+2]); // freq
            u1_t drs   = opts[oidx+5];  // datarate span
            LMIC.NewChannelAns = 0x80;
            if( freq != 0 && LMIC_setupChannel(chidx, freq, DR_RANGE_MAP(drs&0xF,drs>>4), -1) )
                LMIC.NewChannelAns |= MCMD_SNCH_ANS_DRACK|MCMD_SNCH_ANS_FQACK;
            oidx += 6;
            continue;
        }
        case PingSlotChannelReq: {
            u4_t freq = convFreq(&opts[++oidx]);
            u1_t dr;
            oidx += 4;
            dr = opts[oidx++];
            u1_t flags = 0x80;
            if( freq != 0 ) {
                flags |= MCMD_PING_ANS_FQACK;
                LMIC.ping.freq = freq;
                DO_DEVDB(LMIC.ping.intvExp, pingIntvExp);
                DO_DEVDB(LMIC.ping.freq, pingFreq);
                DO_DEVDB(LMIC.ping.dr, pingDr);
            }
            if (validDR(dr)) {
                flags |= MCMD_PING_ANS_DRACK;
                LMIC.ping.dr = dr;
            }
            LMIC.PingSlotChannelAns = flags;
            continue;
        }
        case PingSlotInfoAns: {
            oidx++;
            LMIC.ping.sent_intvExp = LMIC.ping.intvExp;
            LMIC.flags.PingSlotInfoReq = 0;
        }
        case BeaconTimingAns: {
            // Ignore if tracking already enabled
            if( (LMIC.opmode & OP_TRACK) == 0 ) {
                LMIC.bcnChnl = opts[oidx+3];
                // Enable tracking - bcninfoTries
                LMIC.opmode |= OP_TRACK;
                // Cleared later in txComplete handling - triggers EV_BEACON_FOUND
                MBED_ASSERT(LMIC.bcninfoTries!=0);
                // Setup RX parameters
                LMIC.bcninfo.txtime_us = (LMIC.rxtime_us
                                       + (os_rlsbf2(&opts[oidx+1]) * MCMD_BCNI_TUNIT_us)
                                       + (MCMD_BCNI_TUNIT_us/2)
                                       - BCN_INTV_us);
                LMIC.bcninfo.flags = 0;  // txtime above cannot be used as reference (BCN_PARTIAL|BCN_FULL cleared)
                calcBcnRxWindowFromMicros(MCMD_BCNI_TUNIT_us, 1);  // error of +/-N ms 

                EV(lostFrame, INFO, (e_.reason  = EV::lostFrame_t::BeaconTimingAns,
                                     e_.eui     = MAIN::CDEV->getEui(),
                                     e_.lostmic = Base::lsbf4(&d[pend]),
                                     e_.info    = (LMIC.missedBcns |
                                                   ((LMIC.bcninfo.txtime_us + BCN_INTV_us
                                                               - LMIC.bcnRxtime_us) << 8)),
                                     e_.time    = MAIN::CDEV->ostime2ustime(LMIC.bcninfo.txtime + BCN_INTV_osticks)));
            }
            oidx += 4;
            continue;
        }
#if defined(JOINEUI) && defined(OPTNEG)
        case RekeyConf: {
            LMIC.serverVersion = opts[++oidx];
            oidx++;
            printf("ReKeyConf vers:%u\r\n", LMIC.serverVersion);
            LMIC.flags.need_ReKeyConf = 0;
            continue;
        }
        case DeviceTimeAns: {
            onEvent(EV_DEVICE_TIME_ANS, opts + oidx + 1);
            LMIC.flags.DeviceTimeReq = 0;
        }
        case ForceRejoinReq: {
            u2_t args = opts[++oidx];
            args |= opts[++oidx] << 8;
            LMIC.rejoin.type = (args >> 4) & 7;
            if (LMIC.rejoin.type == 2)
                LMIC.JoinReqType = 2;
            else {
                LMIC.JoinReqType = 2;
                LMIC.rejoin.type = 0;
            }
            LMIC.rejoin.dr = args & 0x0f;
            LMIC.datarate = LMIC.rejoin.dr;
            LMIC.rejoin.retries = 1 + ((args >> 8) & 7);
            LMIC.rejoin.Period = (args >> 11) & 7;
            LowPowerTimeout_attach_us(&LMIC.rejoin.ticker, _rejoin_retry, 50000);
            LMIC.rejoin.forced = true;
        }
#endif /* JOINEUI && OPTNEG */
        default:
            printf("\e[31mbad mcmd %02x\e[0m\r\n", opts[oidx]);
            break;
        } // ..switch( opts[oidx] )
        EV(specCond, ERR, (e_.reason = EV::specCond_t::BAD_MAC_CMD,
                           e_.eui    = MAIN::CDEV->getEui(),
                           e_.info   = Base::lsbf4(&d[pend]),
                           e_.info2  = Base::msbf4(&opts[oidx])));
        break;
    } // ..while( oidx < olen )
    //printf("\r\n");
    if( oidx != olen ) {
        EV(specCond, ERR, (e_.reason = EV::specCond_t::CORRUPTED_FRAME,
                           e_.eui    = MAIN::CDEV->getEui(),
                           e_.info   = 0x1000000 + (oidx) + (olen<<8)));
    }

    if( !replayConf ) {
        // Handle payload only if not a replay
        // Decrypt payload - if any
        if( FPort >= 0  &&  pend-poff > 0 )
            aes_cipher(1, FPort <= 0 ? LMIC.NwkSEncKey : LMIC.AppSKey, LMIC.devaddr, seqno, /*dn*/1, d+poff, pend-poff);

        EV(dfinfo, DEBUG, (e_.deveui  = MAIN::CDEV->getEui(),
                           e_.devaddr = LMIC.devaddr,
                           e_.seqno   = seqno,
                           e_.flags   = (FPort < 0 ? EV::dfinfo_t::NOPORT : 0) | EV::dfinfo_t::DN,
                           e_.mic     = Base::lsbf4(&d[pend]),
                           e_.hdr     = d[LORA::OFF_DAT_HDR],
                           e_.fct     = d[LORA::OFF_DAT_FCT],
                           e_.FPort    = FPort,
                           e_.plen    = dlen,
                           e_.opts.length = olen,
                           memcpy(&e_.opts[0], opts, olen)));
    } else {
        EV(specCond, INFO, (e_.reason = EV::specCond_t::DNSEQNO_REPLAY,
                            e_.eui    = MAIN::CDEV->getEui(),
                            e_.info   = Base::lsbf4(&d[pend]),
                            e_.info2  = seqno));
    }

    if( // NWK acks but we don't have a frame pending
        (ackup && LMIC.txCnt == 0) ||
        // We sent up confirmed and we got a response in DNW1/DNW2
        // BUT it did not carry an ACK - this should never happen
        // Do not resend and assume frame was not ACKed.
        (!ackup && LMIC.txCnt != 0) ) {
        EV(specCond, ERR, (e_.reason = EV::specCond_t::SPURIOUS_ACK,
                           e_.eui    = MAIN::CDEV->getEui(),
                           e_.info   = seqno,
                           e_.info2  = ackup));
    }

    if (LMIC.txCnt != 0) { // we requested an ACK
        LMIC.txrxFlags |= ackup ? TXRX_ACK : TXRX_NACK;
    }

    if( FPort < 0 ) {
        LMIC.txrxFlags |= TXRX_NOPORT;
        LMIC.dataBeg = poff;
        LMIC.dataLen = 0;
    } else {
        LMIC.txrxFlags |= TXRX_PORT;
        LMIC.dataBeg = poff;
        LMIC.dataLen = pend-poff;
    }
    return 1;
} // ..decodeFrame()


// ================================================================================
// TX/RX transaction support

static void setupRx2 (void (*func)(void))
{
    hal_dbg_pin_latency(true);
    LMIC.txrxFlags = TXRX_DNW2;
    LMIC.rps = dndr2rps(LMIC.dn2Dr);
    LMIC.freq = LMIC.dn2Freq;
    LMIC.dataLen = 0;
    LMIC.radio_bh_func = func;
    os_radio(RADIO_RX);
}

static void schedRx2(unsigned delay_us, void (*func)(void))
{
    unsigned sp_us;
    us_timestamp_t target;

    delay_us -= RX_RAMPUP_us;       // how long radio chip takes to startup

    LMIC.rxtime_us = LMIC.txend_us + delay_us;
    target = LMIC.rxtime_us;
    target -= RX_SETUP_TIME_us;   // how long spi commands take
    LowPowerTimeout_attach_us_absolute(&LMIC.ticker, func, target);

    LMIC.rps = dndr2rps(LMIC.dn2Dr);
    sp_us = radio_get_symbol_period();
    LMIC.rxsyms = MINRX_SYMS;
    if (sp_us * LMIC.rxsyms < MIN_RX_WIN_us)
        LMIC.rxsyms = MIN_RX_WIN_us / sp_us;
}

static void setupRx1(void (*func)(void))
{
    hal_dbg_pin_latency(true);
    LMIC.txrxFlags = TXRX_DNW1;
    // Turn LMIC.rps from TX over to RX
    LMIC.rps = dndr2rps(LMIC.dndr);
    LMIC.rps = setNocrc(LMIC.rps,1);
    LMIC.dataLen = 0;
    LMIC.radio_bh_func = func;
    os_radio(RADIO_RX);
}


// Called by HAL once TX complete and delivers exact end of TX time stamp in LMIC.rxtime
static void txDone(unsigned delay_us, void (*func)(void))
{
    us_timestamp_t target;

    updatePostTx();

    delay_us -= RX_RAMPUP_us;       // how long radio chip takes to startup

    if( (LMIC.opmode & (OP_TRACK|OP_PINGABLE|OP_PINGINI)) == (OP_TRACK|OP_PINGABLE) ) {
        rxschedInit(&LMIC.ping);    // note: reuses LMIC.frame buffer!
        LMIC.opmode |= OP_PINGINI;
    }
    // Change RX frequency / rps (US only) before we increment txChnl
    setRx1Params(); // rps set here
    // LMIC.rxsyms carries the TX datarate (can be != LMIC.datarate [confirm retries etc.])
    // Setup receive - LMIC.rxtime is preloaded with 1.5 symbols offset to tune
    // into the middle of the 8 symbols preamble.
    if (regionIsFSK()) {
        LMIC.rxtime_us = LMIC.txend_us + delay_us - PRERX_FSK*160;
        LMIC.rxsyms = RXLEN_FSK;
    } else {
        unsigned sp_us;
        sp_us = radio_get_symbol_period();
        LMIC.rxtime_us = LMIC.txend_us + delay_us;
        LMIC.rxsyms = MINRX_SYMS;
        if (sp_us * LMIC.rxsyms < MIN_RX_WIN_us) {
            LMIC.rxsyms = MIN_RX_WIN_us / sp_us;
        }
    }

    target = LMIC.rxtime_us;
    target -= RX_SETUP_TIME_us;   // how long spi commands take
    LowPowerTimeout_attach_us_absolute(&LMIC.ticker, func, target);
} // ..txDone()


// ======================================== Join frames

#ifdef JOINEUI

static void onJoinFailed () {
    // Notify app - must call LMIC_reset() to stop joining
    // otherwise join procedure continues.
    reportEvent(EV_JOIN_FAILED);
}

static bit_t processJoinAccept(void)
{
    MBED_ASSERT(LMIC.txrxFlags != TXRX_DNW1 || LMIC.dataLen != 0);
    MBED_ASSERT((LMIC.opmode & OP_TXRXPEND)!=0);

    printf("processJoinAccept %u ", LMIC.dataLen);

    if (LMIC.dataLen == 0) {
      nojoinframe:
        printf("nojoinframe ");
        if( (LMIC.opmode & OP_JOINING) == 0 ) {
            MBED_ASSERT((LMIC.opmode & OP_REJOIN_) != 0);
            // REJOIN attempt for roaming
            LMIC.opmode &= ~(OP_REJOIN_|OP_TXRXPEND);
            if (LMIC.joinTries < 10)
                LMIC.joinTries++;
            reportEvent(EV_REJOIN_FAILED);
            printf("rejoin\r\n");
            return 1;
        }
        LMIC.opmode &= ~OP_TXRXPEND;
        u1_t delay = nextJoinState();
        EV(devCond, DEBUG, (e_.reason = EV::devCond_t::NO_JACC,
                            e_.eui    = MAIN::CDEV->getEui(),
                            e_.info   = LMIC.datarate|DR_PAGE,
                            e_.info2  = osticks2ms(delay)));
        // Build next JOIN REQUEST with next engineUpdate call
        // Optionally, report join failed.
        // Both after a random/chosen amount of ticks.
        LowPowerTimeout_attach_us(&LMIC.ticker,
                            (delay&1) != 0
                            ? onJoinFailed      // one JOIN iteration done and failed
                            : runEngineUpdate,  // next step to be delayed
                            delay * 1000000);
        printf("\r\n");
        return 1;
    } // ..if (LMIC.dataLen == 0)

    u1_t hdr  = LMIC.frame[0];
    u1_t dlen = LMIC.dataLen;
    //u4_t mic  = os_rlsbf4(&LMIC.frame[dlen-4]); // safe before modified by encrypt!
    if( (dlen != LEN_JA && dlen != LEN_JAEXT)
        || (hdr & (HDR_FTYPE|HDR_MAJOR)) != (HDR_FTYPE_JACC|HDR_MAJOR_V1) ) {
        EV(specCond, ERR, (e_.reason = EV::specCond_t::UNEXPECTED_FRAME,
                           e_.eui    = MAIN::CDEV->getEui(),
                           e_.info   = dlen < 4 ? 0 : mic,
                           e_.info2  = hdr + (dlen<<8)));
      badframe:
        printf("badframe ");
        if( (LMIC.txrxFlags & TXRX_DNW1) != 0 ) {
            printf("\r\n");
            return 0;
        }
        goto nojoinframe;
    }
    aes_encrypt(LMIC.frame+1, dlen-1);
    //debug_buf("jaDecry", LMIC.frame, dlen);
#ifdef OPTNEG
    joinAccept_t* ja = (joinAccept_t*)LMIC.frame;

    if (ja->fields.DLSettings.OptNeg) {
        u4_t calcMic, rxMic;
        u1_t mic_buf[48];
        u1_t* ptr = mic_buf;
        *ptr++ = LMIC.JoinReqType;
        memcpyr(ptr, joinEui, 8);
        ptr += 8;
        *ptr++ = LMIC.devNonce & 0xff;
        *ptr++ = LMIC.devNonce >> 8;
        memcpy(ptr, LMIC.frame, LMIC.dataLen-4);
        ptr += LMIC.dataLen-4;
        //debug_buf("mic_buf", mic_buf, ptr - mic_buf);

        //debug_buf("JSIntKey", LMIC.JSIntKey, 16);
        memcpy(AESkey, LMIC.JSIntKey, 16);
        calcMic = os_aes(AES_MIC|AES_MICNOAUX, mic_buf, ptr - mic_buf);
        //printf("calcMic:%08x ", calcMic);
        rxMic = os_rmsbf4(LMIC.frame + LMIC.dataLen-4);
        //printf("rxMic:%08x\r\n", rxMic);
        if (calcMic != rxMic) {
            printf("mic %08x != %08x ", calcMic, rxMic);
            goto badframe;
        }
        LMIC.flags.OptNeg = 1;
    } else
#endif /* OPTNEG */
    {
        printf("ja1v0 ");
        if( !aes_verifyMic0(LMIC.frame, dlen-4) ) {
            EV(specCond, ERR, (e_.reason = EV::specCond_t::JOIN_BAD_MIC,
                               e_.info   = mic));
            printf("jaMicFail ");
            goto badframe;
        }
        LMIC.flags.OptNeg = 0;
    }

    u4_t addr = os_rlsbf4(LMIC.frame+OFF_JA_DEVADDR);
    LMIC.devaddr = addr;
    LMIC.netid = os_rlsbf4(&LMIC.frame[OFF_JA_NETID]) & 0xFFFFFF;

    initDefaultChannels(0);

    if (dlen > LEN_JA) {
        dlen = OFF_CFLIST;
        for( u1_t chidx=3; chidx<8; chidx++, dlen+=3 ) {
            u4_t freq = convFreq(&LMIC.frame[dlen]);
            if( freq )
                LMIC_setupChannel(chidx, freq, 0, -1);
        }
    }

    // already incremented when JOIN REQ got sent off
#ifdef OPTNEG
    if (ja->fields.DLSettings.OptNeg) {
        u4_t our_jn = nvm_read(NVM_JOINNONCE);
        if (ja->fields.joinNonce > our_jn) {
            aes_sessKeys_1v1(ja->fields.joinNonce);
            nvm_incr(NVM_JOINNONCE, ja->fields.joinNonce - our_jn);
            LMIC.flags.need_ReKeyConf = 1;
        } else
            printf("JAreplay %u <= %u\r\n", ja->fields.joinNonce, our_jn);
    } else
#endif /* OPTNEG */
        aes_sessKeys_1v0();

    DO_DEVDB(LMIC.netid,   netid);
    DO_DEVDB(LMIC.devaddr, devaddr);
    DO_DEVDB(LMIC.nwkKey,  nwkkey);
    DO_DEVDB(LMIC.AppSKey,  artkey);

    EV(joininfo, INFO, (e_.arteui  = MAIN::CDEV->getArtEui(),
                        e_.deveui  = MAIN::CDEV->getEui(),
                        e_.devaddr = LMIC.devaddr,
                        e_.oldaddr = oldaddr,
                        e_.nonce   = LMIC.devNonce,
                        e_.mic     = mic,
                        e_.reason  = ((LMIC.opmode & OP_REJOIN) != 0
                                      ? EV::joininfo_t::REJOIN_ACCEPT
                                      : EV::joininfo_t::ACCEPT)));
    
    MBED_ASSERT((LMIC.opmode & (OP_JOINING|OP_REJOIN_))!=0);
/*    if( (LMIC.opmode & OP_REJOIN) != 0 ) {
        // Lower DR every try below current UP DR
        LMIC.datarate = lowerDR(LMIC.datarate, LMIC.joinTries);
    }*/
    LMIC.opmode &= ~(OP_JOINING|OP_TRACK|OP_REJOIN_|OP_TXRXPEND|OP_PINGINI) | OP_NEXTCHNL;
    LMIC.txCnt = 0;
    stateJustJoined();
#ifdef OPTNEG
    LMIC.RJcount0 = 0;
#endif
    LMIC.dn2Dr = LMIC.frame[OFF_JA_DLSET] & 0x0F;
    LMIC.RX1DRoffset = (LMIC.frame[OFF_JA_DLSET] >> 4) & 0x07;  // take off OptNeg and Rx2dr
    LMIC.rxDelay_sec = LMIC.frame[OFF_JA_RXDLY];
    if (LMIC.rxDelay_sec == 0)
        LMIC.rxDelay_sec = 1;   

    printf("rx1dro:%u ", LMIC.RX1DRoffset);
    reportEvent(EV_JOINED);
    return 1;
} // ..processJoinAccept()

static void processRx2Jacc () {
    if( LMIC.dataLen == 0 )
        LMIC.txrxFlags = 0;  // nothing in 1st/2nd DN slot
    processJoinAccept();
}

static void setupRx2Jacc () {
    setupRx2(processRx2Jacc);
}

static void processRx1Jacc() {
    if( LMIC.dataLen == 0 || !processJoinAccept() )
        schedRx2(DELAY_JACC2_us, setupRx2Jacc);
}

static void setupRx1Jacc() {
    setupRx1(processRx1Jacc);
}

static void jreqDone() {
    txDone(DELAY_JACC1_us, setupRx1Jacc);
}
#endif /* JOINEUI */

// ======================================== Data frames

// Fwd decl.
static bit_t processDnData(void);

static void processRx2DnDataDelay () {
    processDnData();
}

static void processRx2DnData () {
    //printf("processrx2 %u\r\n", LMIC.dataLen);
    if( LMIC.dataLen == 0 ) {
        LMIC.txrxFlags = 0;  // nothing in 1st/2nd DN slot
        // Delay callback processing to avoid up TX while gateway is txing our missed frame! 
        // Since DNW2 uses SF12 by default we wait 3 secs.
        LowPowerTimeout_attach_us(&LMIC.ticker, processRx2DnDataDelay, DNW2_SAFETY_ZONE_us + rndDelay(2));
        return;
    }
    processDnData();
}


static void setupRx2DnData ()
{
    setupRx2(processRx2DnData);
}


static void processRx1DnData () {
    //printf("processrx1 %u\r\n", LMIC.dataLen);
    if( LMIC.dataLen == 0 || !processDnData() )
        schedRx2((LMIC.rxDelay_sec + DELAY_EXTDNW2_sec) * 1000000, setupRx2DnData);
}


static void setupRx1DnData() {
    setupRx1(processRx1DnData);
}

static void updataDone() {
    txDone(LMIC.rxDelay_sec*1000000, setupRx1DnData);
}

// ======================================== 


static void buildDataFrame (void)
{
    u4_t FCntUp;
    bit_t txdata = ((LMIC.opmode & (OP_TXDATA|OP_POLL)) != OP_POLL);
    u1_t dlen = txdata ? LMIC.pendTxLen : 0;

    // Piggyback MAC options
    // Prioritize by importance
    int  end = OFF_DAT_OPTS;
    //if( (LMIC.opmode & (OP_TRACK|OP_PINGABLE)) == (OP_TRACK|OP_PINGABLE) )
    if (LMIC.flags.PingSlotInfoReq) {
        LMIC.frame[end++] = PingSlotInfoReq;
        LMIC.frame[end++] = LMIC.ping.intvExp;
    }
    if (LMIC.flags.DutyCycleAns) {
        LMIC.frame[end] = DutyCycleAns;
        end += 1;
        LMIC.flags.DutyCycleAns = 0;
    }
    if (LMIC.flags.RXTimingSetupAns) {
        LMIC.frame[end++] = RXTimingSetupAns;
    }
    if( LMIC.RXParamSetupAns) {
        LMIC.frame[end+0] = RXParamSetupAns;
        LMIC.frame[end+1] = LMIC.RXParamSetupAns & ~MCMD_DN2P_ANS_RFU;
        end += 2;
    }
    if( LMIC.flags.DevStatusAns) {  // answer to device status
        LMIC.frame[end+0] = DevStatusAns;
        LMIC.frame[end+1] = os_getBattLevel();
        LMIC.frame[end+2] = LMIC.margin;
        end += 3;
        LMIC.flags.DevStatusAns = 0;
    }
    if( LMIC.LinkADRAns) {  // answer to ADR change
        LMIC.frame[end+0] = LinkADRAns;
        LMIC.frame[end+1] = LMIC.LinkADRAns & ~MCMD_LADR_ANS_RFU;
        end += 2;
        LMIC.LinkADRAns = 0;
    }
    if (LMIC.bcninfoTries > 0) {
        LMIC.frame[end++] = BeaconTimingReq;
    }
    if( LMIC.flags.adrChanged ) {
        if( LMIC.adrAckReq < 0 )
            LMIC.adrAckReq = 0;
        LMIC.flags.adrChanged = 0;
    }
    if( LMIC.PingSlotChannelAns != 0 ) {
        LMIC.frame[end+0] = PingSlotChannelAns;
        LMIC.frame[end+1] = LMIC.PingSlotChannelAns & ~MCMD_PING_ANS_RFU;
        end += 2;
        LMIC.PingSlotChannelAns = 0;
    }
    if( LMIC.NewChannelAns) {
        LMIC.frame[end+0] = NewChannelAns;
        LMIC.frame[end+1] = LMIC.NewChannelAns & ~MCMD_SNCH_ANS_RFU;
        end += 2;
        LMIC.NewChannelAns = 0;
    }
    if (LMIC.flags.LinkCheckReq) {
        LMIC.frame[end++] = LinkCheckReq;
    }
    if (LMIC.flags.DeviceTimeReq) {
        LMIC.frame[end++] = DeviceTimeReq;
    }
#if defined(JOINEUI) && defined(OPTNEG)
    if (LMIC.flags.need_ReKeyConf) {
        LMIC.frame[end++] = RekeyInd;
        LMIC.frame[end++] = 1;  // lorawan-1.1
    }
#endif /* JOINEUI && OPTNEG */
    MBED_ASSERT(end <= OFF_DAT_OPTS+16);

    u1_t flen = end + (txdata ? 5+dlen : 4);
    if( flen > MAX_LEN_FRAME ) {
        // Options and payload too big - delay payload
        txdata = 0;
        flen = end+4;
    }

    FCntUp = read_FCntUp;

#ifdef OPTNEG
    if (LMIC.flags.OptNeg && (end - OFF_DAT_OPTS) > 0) {
        aes_cipher(0, LMIC.NwkSEncKey, LMIC.devaddr, FCntUp, UP_LINK, LMIC.frame+OFF_DAT_OPTS, end-OFF_DAT_OPTS);
    }
#endif /* OPTNEG */

    LMIC.frame[OFF_DAT_HDR] = HDR_FTYPE_DAUP | HDR_MAJOR_V1;
    LMIC.frame[OFF_DAT_FCT] = (LMIC.dnConf | LMIC.adrEnabled
                              | (LMIC.adrAckReq >= 0 ? FCT_ADRARQ : 0)
                              | (end-OFF_DAT_OPTS));
    os_wlsbf4(LMIC.frame+OFF_DAT_ADDR,  LMIC.devaddr);

    if (LMIC.txCnt == 0) {
        incr_FCntUp;
        DO_DEVDB(LMIC.FCntUp,FCntUp);
    } else {
        EV(devCond, INFO, (e_.reason = EV::devCond_t::RE_TX,
                           e_.eui    = MAIN::CDEV->getEui(),
                           e_.info   = FCntUp,
                           e_.info2  = ((LMIC.txCnt+1) |
                                        (DRADJUST[LMIC.txCnt+1] << 8) |
                                        ((LMIC.datarate|DR_PAGE)<<16))));
    }
    os_wlsbf2(LMIC.frame+OFF_DAT_SEQNO, FCntUp);

    // Clear pending DN confirmation
    LMIC.dnConf = 0;

    LMIC.ConfFCntUp = 0;
    if (txdata) {
        if (LMIC.pendTxConf) {
#ifdef OPTNEG
            LMIC.ConfFCntUp = FCntUp;
#endif
            // Confirmed only makes sense if we have a payload (or at least a port)
            LMIC.frame[OFF_DAT_HDR] = HDR_FTYPE_DCUP | HDR_MAJOR_V1;
            if (LMIC.txCnt == 0) {
                LMIC.txCnt = 1;
            }
        }
        LMIC.frame[end] = LMIC.pendTxPort;
        os_copyMem(LMIC.frame+end+1, LMIC.pendTxData, dlen);
        aes_cipher(1, LMIC.pendTxPort == 0 ? LMIC.NwkSEncKey : LMIC.AppSKey,
                   LMIC.devaddr, FCntUp,
                   UP_LINK, LMIC.frame+end+1, dlen);
    }

    block_t block;

    block.b.header = 0x49;
    block.b.confFCnt = 0;
    block.b.dr = 0;
    block.b.ch = 0;
    block.b.dir = UP_LINK;
    block.b.DevAddr = LMIC.devaddr;
    block.b.FCnt = FCntUp;
    block.b.zero8 = 0;
    block.b.lenMsg = flen-4;

    aes_appendMic(&block, LMIC.frame, flen-4);

    EV(dfinfo, DEBUG, (e_.deveui  = MAIN::CDEV->getEui(),
                       e_.devaddr = LMIC.devaddr,
                       e_.seqno   = FCntUp,
                       e_.flags   = (LMIC.pendTxPort < 0 ? EV::dfinfo_t::NOPORT : EV::dfinfo_t::NOP),
                       e_.mic     = Base::lsbf4(&LMIC.frame[flen-4]),
                       e_.hdr     = LMIC.frame[LORA::OFF_DAT_HDR],
                       e_.fct     = LMIC.frame[LORA::OFF_DAT_FCT],
                       e_.port    = LMIC.pendTxPort,
                       e_.plen    = txdata ? dlen : 0,
                       e_.opts.length = end-LORA::OFF_DAT_OPTS,
                       memcpy(&e_.opts[0], LMIC.frame+LORA::OFF_DAT_OPTS, end-LORA::OFF_DAT_OPTS)));
    LMIC.dataLen = flen;

} // ..buildDataFrame()


// Callback from HAL during scan mode or when job timer expires.
static void onBcnRx ()
{
    // If we arrive via job timer make sure to put radio to rest.
    os_radio(RADIO_RST);
    LowPowerTimeout_detach(&LMIC.ticker);

    if( LMIC.dataLen == 0 ) {
        // Nothing received - timeout
        LMIC.opmode &= ~(OP_SCAN | OP_TRACK);
        reportEvent(EV_SCAN_TIMEOUT);
        return;
    }
    if( decodeBeacon() <= 0 ) {
        // Something is wrong with the beacon - continue scan
        LMIC.dataLen = 0;
        os_radio(RADIO_RXON);
        LowPowerTimeout_attach_us_absolute(&LMIC.ticker, onBcnRx,LMIC.bcninfo.txtime_us);
        return;
    }
    // Found our 1st beacon
    // We don't have a previous beacon to calc some drift - assume
    // an max error of 13ms = 128sec*100ppm which is roughly +/-100ppm
    calcBcnRxWindowFromMicros(13000, 1);
    LMIC.opmode &= ~OP_SCAN;          // turn SCAN off
    LMIC.opmode |=  OP_TRACK;         // auto enable tracking
    reportEvent(EV_BEACON_FOUND);    // can be disabled in callback

    if (LMIC.ping.sent_intvExp != LMIC.ping.intvExp)
        LMIC.flags.PingSlotInfoReq = 1; // inform sNS that we are pingable
}


// Enable receiver to listen to incoming beacons
// netid defines when scan stops (any or specific beacon)
// This mode ends with events: EV_SCAN_TIMEOUT/EV_SCAN_BEACON
// Implicitely cancels any pending TX/RX transaction.
// Also cancels an onpoing joining procedure.
static void startScan (void)
{
    MBED_ASSERT(LMIC.devaddr!=0 && (LMIC.opmode & OP_JOINING)==0);
    if( (LMIC.opmode & OP_SHUTDOWN) != 0 )
        return;
    // Cancel onging TX/RX transaction
    LMIC.txCnt = LMIC.dnConf = LMIC.bcninfo.flags = 0;
    LMIC.opmode = (LMIC.opmode | OP_SCAN) & ~(OP_TXRXPEND);
    setBcnRxParams();
    LMIC.rxtime_us = LMIC.bcninfo.txtime_us = BCN_INTV_us + 1000000;
    LowPowerTimeout_attach_us_absolute(&LMIC.ticker, onBcnRx, LMIC.rxtime_us);
    os_radio(RADIO_RXON);
}


bit_t LMIC_enableTracking (u1_t tryBcnInfo) {
    if( (LMIC.opmode & (OP_SCAN|OP_TRACK|OP_SHUTDOWN)) != 0 )
        return 0;  // already in progress or failed to enable
    // If BCN info requested from NWK then app has to take are
    // of sending data up so that BeaconTimingReq can be attached.
    if( (LMIC.bcninfoTries = tryBcnInfo) == 0 )
        startScan();
    return 1;  // enabled
}


void LMIC_disableTracking (void) {
    LMIC.opmode &= ~(OP_SCAN|OP_TRACK);
    LMIC.bcninfoTries = 0;
    engineUpdate();
}



























// ================================================================================
//
// Join stuff
//
// ================================================================================

#ifdef JOINEUI
static void buildJoinRequest (u1_t ftype)
{
    const u1_t appEui[8] = JOINEUI;
    // Do not use pendTxData since we might have a pending
    // user level frame in there. Use RX holding area instead.
    xref2u1_t _d = LMIC.frame;
    _d[OFF_JR_HDR] = ftype;
    if (ftype == HDR_FTYPE_JREQ) {
        memcpyr(_d + OFF_JR_ARTEUI, appEui, 8);
        os_getDevEui(_d + OFF_JR_DEVEUI);
#ifdef ROOT_APPKEY
        /* 1v1 */
        LMIC.devNonce     =  nvm_read(NVM_DEVNONCE);
        nvm_incr(NVM_DEVNONCE, 1);
#else
        /* 1v0 */
        LMIC.devNonce     =  os_getRndU2();
#endif
        os_wlsbf2(_d + OFF_JR_DEVNONCE, LMIC.devNonce);
        aes_appendMic0(root_nwkKey, _d, OFF_JR_MIC);

        EV(joininfo,INFO,(e_.deveui  = MAIN::CDEV->getEui(),
                          e_.arteui  = MAIN::CDEV->getArtEui(),
                          e_.nonce   = LMIC.devNonce,
                          e_.oldaddr = LMIC.devaddr,
                          e_.mic     = Base::lsbf4(&d[LORA::OFF_JR_MIC]),
                          e_.reason  = ((LMIC.opmode & OP_REJOIN) != 0
                                        ? EV::joininfo_t::REJOIN_REQUEST
                                        : EV::joininfo_t::REQUEST)));
        LMIC.dataLen = LEN_JR;
        DO_DEVDB(LMIC.devNonce,devNonce);
#ifdef OPTNEG
        LMIC.JoinReqType = 0xff;
#endif /* OPTNEG */
    }
#ifdef OPTNEG
    else if (ftype == HDR_FTYPE_REJOIN) {
        LMIC.JoinReqType = LMIC.rejoinType;
        if (LMIC.rejoinType == 1) {
            /* sent to JoinServer */
            memcpyr(_d + OFF_RJ1_JOINEUI, joinEui, 8);
            os_getDevEui(_d + OFF_RJ1_DEVEUI);
            LMIC.devNonce = nvm_read(NVM_RJCOUNT1);
            os_wlsbf2(_d + OFF_RJ1_RJCOUNT1, LMIC.devNonce);
            aes_appendMic0(LMIC.JSIntKey, _d, OFF_RJ1_MIC);
            nvm_incr(NVM_RJCOUNT1, 1);
        } else if (LMIC.rejoinType == 2 || LMIC.rejoinType == 0) {
            /* sent to sNS / hNS */
            os_wlsbf3(_d + OFF_RJ02_NETID, LMIC.netid);
            os_getDevEui(_d + OFF_RJ02_DEVEUI);
            LMIC.devNonce = LMIC.RJcount0++;
            os_wlsbf2(_d + OFF_RJ02_RJCOUNT0, LMIC.devNonce);
            aes_appendMic0(LMIC.SNwkSIntKey, _d, OFF_RJ0_MIC);
        }
    }
#endif /* OPTNEG */
}

static void startJoining () {
    reportEvent(EV_JOINING);
}
#endif /* JOINEUI */

// Start join procedure if not already joined.
#if defined(JOINEUI)
bit_t LMIC_startJoining(void)
{
    if( LMIC.devaddr == 0 ) {
        // There should be no TX/RX going on
        MBED_ASSERT((LMIC.opmode & (OP_POLL|OP_TXRXPEND)) == 0);
        // Lift any previous duty limitation
        LMIC.globalDutyRate = 0;
        // Cancel scanning
        LMIC.opmode &= ~(OP_SCAN|OP_REJOIN_|OP_LINKDEAD|OP_NEXTCHNL);
        // Setup state
        LMIC.joinTries = LMIC.txCnt = LMIC.pendTxConf = 0;
        initJoinLoop();
        LMIC.opmode |= OP_JOINING;
        // reportEvent will call engineUpdate which then starts sending JOIN REQUESTS
        LowPowerTimeout_attach_us(&LMIC.ticker, startJoining, TX_RAMPUP_us*5);
        return 1;
    }
    return 0; // already joined
}
#endif /* JOINEUI */

// ================================================================================
//
//
//
// ================================================================================

static void processPingRx () {
    if( LMIC.dataLen != 0 ) {
        LMIC.txrxFlags = TXRX_PING;
        if (decodeFrame()) {
            reportEvent(EV_RXCOMPLETE);
            return;
        }
    }
    // Pick next ping slot
    engineUpdate();
}


static bit_t processDnData (void)
{
    MBED_ASSERT((LMIC.opmode & OP_TXRXPEND)!=0);

    /*printf("processDnData");
    if (LMIC.txrxFlags & TXRX_DNW1)
        printf("1");
    if (LMIC.txrxFlags & TXRX_DNW2)
        printf("2");
    printf(" dataLen%u ", LMIC.dataLen);*/

    if (LMIC.dataLen == 0) {
      norx:
        if (LMIC.txCnt != 0) {
            if (LMIC.txCnt < TXCONF_ATTEMPTS) {
                LMIC.txCnt += 1;
                setDrTxpow(DRCHG_NOACK, lowerDR(LMIC.datarate, DRADJUST[LMIC.txCnt]), KEEP_TXPOW);
                // Schedule another retransmission
                txDelay(LMIC.rxtime_us, RETRY_PERIOD_secs);
                LMIC.opmode &= ~OP_TXRXPEND;
                printf("txConfRetry ");
                engineUpdate();
                return 1;
            }
            LMIC.txrxFlags = TXRX_NACK | TXRX_NOPORT;
        } else {
            // Nothing received - implies no port
            LMIC.txrxFlags = TXRX_NOPORT;
        }
        if( LMIC.adrAckReq != LINK_CHECK_OFF )
            LMIC.adrAckReq += 1;
        LMIC.dataBeg = LMIC.dataLen = 0;
      txcomplete:
        LMIC.opmode &= ~(OP_TXDATA|OP_TXRXPEND);
        if( (LMIC.txrxFlags & (TXRX_DNW1|TXRX_DNW2|TXRX_PING)) != 0  &&  (LMIC.opmode & OP_LINKDEAD) != 0 ) {
            LMIC.opmode &= ~OP_LINKDEAD;
            reportEvent(EV_LINK_ALIVE);
        }
        reportEvent(EV_TXCOMPLETE);
        // If we haven't heard from NWK in a while although we asked for a sign
        // assume link is dead - notify application and keep going
        if( LMIC.adrAckReq > LINK_CHECK_DEAD ) {
            // We haven't heard from NWK for some time although we
            // asked for a response for some time - assume we're disconnected. Lower DR one notch.
            EV(devCond, ERR, (e_.reason = EV::devCond_t::LINK_DEAD,
                              e_.eui    = MAIN::CDEV->getEui(),
                              e_.info   = LMIC.adrAckReq));
            setDrTxpow(DRCHG_NOADRACK, decDR((dr_t)LMIC.datarate), KEEP_TXPOW);
            LMIC.adrAckReq = LINK_CHECK_CONT;
#ifdef OPTNEG
            LMIC.opmode |= OP_REJOIN_ | OP_LINKDEAD;
            LMIC.rejoinType = 0;    // rejoin to sNS/hNS
            //LMIC.rejoinType = 1;    // rejoin to JoinServer
#else
            LMIC.opmode |= OP_JOINING | OP_LINKDEAD;
#endif /* OPTNEG */
            reportEvent(EV_LINK_DEAD);
        } // ..link check dead
        // If this falls to zero the NWK did not answer our BeaconTimingReq commands - try full scan
        if( LMIC.bcninfoTries > 0 ) {
            if( (LMIC.opmode & OP_TRACK) != 0 ) {
                reportEvent(EV_BEACON_FOUND);
                LMIC.bcninfoTries = 0;
            }
            else if( --LMIC.bcninfoTries == 0 ) {
                startScan();   // NWK did not answer - try scan
            }
        }
        return 1;
    } // ..if (LMIC.dataLen == 0)

    if (!decodeFrame()) {
        printf("decodeFrameFail");
        if (LMIC.txrxFlags & TXRX_DNW1)
            printf("1");
        if (LMIC.txrxFlags & TXRX_DNW2)
            printf("2");

        if( (LMIC.txrxFlags & TXRX_DNW1) != 0 ) {
            printf("->ret0\r\n");
            return 0;
        }
        printf("->norx\r\n");
        goto norx;
    }

    LMIC.flags.RXTimingSetupAns = 0;
    LMIC.RXParamSetupAns = 0;
    printf(" datalen%u->txcomplete ", LMIC.dataLen);
    goto txcomplete;
} // ..processDnData


static void processBeacon ()
{
    us_timestamp_t lasttx_us = LMIC.bcninfo.txtime_us;   // save here - decodeBeacon might overwrite
    u1_t flags = LMIC.bcninfo.flags;
    ev_t ev;

    if( LMIC.dataLen != 0 && decodeBeacon() >= 1 ) {
        ev = EV_BEACON_TRACKED;
        if( (flags & (BCN_PARTIAL|BCN_FULL)) == 0 ) {
            // We don't have a previous beacon to calc some drift - assume
            // an max error of 13ms = 128sec*100ppm which is roughly +/-100ppm
            calcBcnRxWindowFromMicros(13000, 0);
            goto rev;
        }
        // We have a previous BEACON to calculate some drift
        s2_t drift = BCN_INTV_us - (LMIC.bcninfo.txtime_us - lasttx_us);
        if( LMIC.missedBcns > 0 ) {
            drift = LMIC.drift + (drift - LMIC.drift) / (LMIC.missedBcns+1);
        }
        if( (LMIC.bcninfo.flags & BCN_NODRIFT) == 0 ) {
            s2_t diff = LMIC.drift - drift;
            if( diff < 0 ) diff = -diff;
            LMIC.lastDriftDiff = diff;
            if( LMIC.maxDriftDiff < diff )
                LMIC.maxDriftDiff = diff;
            LMIC.bcninfo.flags &= ~BCN_NODDIFF;
        }
        LMIC.drift = drift;
        LMIC.missedBcns = 0;
#ifdef JOINEUI
        LMIC.joinTries = 0;
#endif /* JOINEUI */
        LMIC.bcninfo.flags &= ~BCN_NODRIFT;
        EV(devCond,INFO,(e_.reason = EV::devCond_t::CLOCK_DRIFT,
                         e_.eui    = MAIN::CDEV->getEui(),
                         e_.info   = drift,
                         e_.info2  = /*occasion BEACON*/0));
        MBED_ASSERT((LMIC.bcninfo.flags & (BCN_PARTIAL|BCN_FULL)) != 0);
    } else {
        ev = EV_BEACON_MISSED;
        LMIC.bcninfo.txtime_us += BCN_INTV_us- LMIC.drift;
        LMIC.bcninfo.time   += BCN_INTV_sec;
        LMIC.missedBcns++;
        // Delay any possible TX after surmised beacon - it's there although we missed it
        txDelay(LMIC.bcninfo.txtime_us + BCN_RESERVE_us, 4);
#ifdef OPTNEG
        if (LMIC.missedBcns > MAX_MISSED_BCNS) {
            LMIC.opmode |= OP_REJOIN_;  // try if we can roam to another network
            LMIC.rejoinType = 0;    // rejoin to sNS/hNS
            //LMIC.rejoinType = 1;    // rejoin to JoinServer
        }
#endif /* OPTNEG */
        if( LMIC.bcnRxsyms > MAX_RXSYMS ) {
            LMIC.opmode &= ~(OP_TRACK|OP_PINGABLE|OP_PINGINI|OP_REJOIN_);
            reportEvent(EV_LOST_TSYNC);
            return;
        }
    }
    LMIC.bcnRxtime_us = LMIC.bcninfo.txtime_us + BCN_INTV_us - calcRxWindow(0, DR_BCN);
    LMIC.bcnRxsyms = LMIC.rxsyms;    
  rev:
    regionProcessBeacon();
    if( (LMIC.opmode & OP_PINGINI) != 0 )
        rxschedInit(&LMIC.ping);  // note: reuses LMIC.frame buffer!

    reportEvent(ev);
} // ..processBeacon()


static void startRxBcn ()
{
    LMIC.radio_bh_func = processBeacon;
    os_radio(RADIO_RX);
}


static void startRxPing ()
{
    LMIC.radio_bh_func = processPingRx;
    os_radio(RADIO_RX);
}


// Decide what to do next for the MAC layer of a device
static void engineUpdate ()
{
    // Check for ongoing state: scan or TX/RX transaction
    if ((LMIC.opmode & (OP_SCAN|OP_TXRXPEND|OP_SHUTDOWN)) != 0 )  {
        return;
    }

#if defined(JOINEUI)
    if( LMIC.devaddr == 0 && (LMIC.opmode & OP_JOINING) == 0 ) {
        LMIC_startJoining();
        return;
    }
#endif

    us_timestamp_t  now_us    = LowPowerTimeout_read_us(&LMIC.ticker);
    us_timestamp_t  rxtime_us = 0;
    LMIC.txbeg_us  = 0;

    if( (LMIC.opmode & OP_TRACK) != 0 ) {
        // We are tracking a beacon
        rxtime_us = LMIC.bcnRxtime_us;
        rxtime_us -= RX_RAMPUP_us;
        rxtime_us -= RX_SETUP_TIME_us;
    }

    if( (LMIC.opmode & (OP_JOINING|OP_REJOIN_|OP_TXDATA|OP_POLL)) != 0 ) {
        int64_t txHoldoff;
        // Need to TX some data...
        // Assuming txChnl points to channel which first becomes available again.
        bit_t jacc = ((LMIC.opmode & (OP_JOINING|OP_REJOIN_)) != 0 ? 1 : 0);
        // Find next suitable channel and return availability time
        if( (LMIC.opmode & OP_NEXTCHNL) != 0 ) {
            LMIC.txbeg_us = LMIC.txend_us = nextTx(now_us);
            printf(" ch%u\r\n", LMIC.txChnl);
            LMIC.opmode &= ~OP_NEXTCHNL;
        } else {
            LMIC.txbeg_us = LMIC.txend_us;
            printf("\r\n");
        }
        // Delayed TX or waiting for duty cycle?
        if ((LMIC.globalDutyRate != 0 || (LMIC.opmode & OP_RNDTX) != 0)  &&  (LMIC.txbeg_us - LMIC.globalDutyAvail_us) < 0 ) {
            LMIC.txbeg_us = LMIC.globalDutyAvail_us;
        }
        // If we're tracking a beacon...
        // then make sure TX-RX transaction is complete before beacon
        if ((LMIC.opmode & OP_TRACK) != 0 &&
            LMIC.txbeg_us + (jacc ? JOIN_GUARD_us : TXRX_GUARD_us) - rxtime_us > 0 ) {
            // Not enough time to complete TX-RX before beacon - postpone after beacon.
            // In order to avoid clustering of postponed TX right after beacon randomize start!
            txDelay(rxtime_us + BCN_RESERVE_us, 16);
            LMIC.txbeg_us = 0;
            goto checkrx;
        }
        txHoldoff = LMIC.txbeg_us - (now_us + TX_RAMPUP_us);
        // now must be greater than txbeg
        // Earliest possible time vs overhead to setup radio
        if (txHoldoff < 0) {
            // We could send right now!
            LMIC.txbeg_us = now_us;
            dr_t txdr = (dr_t)LMIC.datarate;
#ifdef JOINEUI
            if (jacc) {
                u1_t ftype;
                if ((LMIC.opmode & OP_REJOIN_) != 0) {
                    txdr = lowerDR(txdr, LMIC.joinTries);
                    ftype = HDR_FTYPE_REJOIN;
                } else {
                    ftype = HDR_FTYPE_JREQ;
                }
                buildJoinRequest(ftype);
                LMIC.radio_bh_func = jreqDone;
                LMIC.dndr   = txdr;  // carry TX datarate (can be != LMIC.datarate) over to txDone/setupRx1
            } else
#endif /* JOINEUI */
            {
                u4_t FCntUp, NFCntDown;
                NFCntDown = read_NFCntDown;
                if (NFCntDown >= 0xFFFFFF80) {
                    // Imminent roll over - proactively reset MAC
                    EV(specCond, INFO, (e_.reason = EV::specCond_t::DNSEQNO_ROLL_OVER,
                                        e_.eui    = MAIN::CDEV->getEui(),
                                        e_.info   = LMIC.NFCntDown, 
                                        e_.info2  = 0));
                    // Device has to react! NWK will not roll over and just stop sending.
                    // Thus, we have N frames to detect a possible lock up.
                  reset:
                    LMIC.radio_bh_func = runReset;
                    LMIC.radio_bottom_half = true;
                    return;
                }
                FCntUp = read_FCntUp;
                if ((LMIC.txCnt == 0 && FCntUp == 0xFFFFFFFF)) {
                    // Roll over of up seq counter
                    EV(specCond, ERR, (e_.reason = EV::specCond_t::UPSEQNO_ROLL_OVER,
                                       e_.eui    = MAIN::CDEV->getEui(),
                                       e_.info2  = FCntUp));
                    // Do not run RESET event callback from here!
                    // App code might do some stuff after send unaware of RESET.
                    goto reset;
                }
                buildDataFrame();
                LMIC.radio_bh_func = updataDone;
                //printf("dr:%u->", txdr);
                LMIC.dndr = txdr_to_rx1dr[txdr][LMIC.RX1DRoffset];  // carry TX datarate (can be != LMIC.datarate) over to txDone/setupRx1
                //printf("%u\r\n", LMIC.dndr);
            }
            LMIC.rps    = setCr(updr2rps(txdr), (cr_t)LMIC.errcr);
            LMIC.opmode = (LMIC.opmode & ~(OP_POLL|OP_RNDTX)) | OP_TXRXPEND | OP_NEXTCHNL;
            updateTx();
            reportEvent(EV_TXSTART);
            os_radio(RADIO_TX);
            return;
        }
        // Cannot yet TX
        if( (LMIC.opmode & OP_TRACK) == 0 ) {
            goto txdelay; // We don't track the beacon - nothing else to do - so wait for the time to TX
        }
        // Consider RX tasks
        if (LMIC.txbeg_us == 0 ) { // zero indicates no TX pending
            LMIC.txbeg_us += 500;  // TX delayed by one tick (insignificant amount of time)
        }
    } else {
        // No TX pending - no scheduled RX
        if ((LMIC.opmode & OP_TRACK) == 0) {
            return;
        }
    }

    // Are we pingable?
  checkrx:
    if( (LMIC.opmode & OP_PINGINI) != 0 ) {
        // One more RX slot in this beacon period?
        if (rxschedNext(&LMIC.ping, now_us + RX_RAMPUP_us + RX_SETUP_TIME_us)) {
            us_timestamp_t target;
            
            if (LMIC.txbeg_us != 0  &&  (LMIC.txbeg_us - LMIC.ping.rxtime_us) < 0)
                goto txdelay;
            LMIC.rxsyms  = LMIC.ping.rxsyms;
            LMIC.rxtime_us  = LMIC.ping.rxtime_us;
            LMIC.freq    = LMIC.ping.freq;
            LMIC.rps     = dndr2rps(LMIC.ping.dr);
            LMIC.dataLen = 0;
            MBED_ASSERT(LMIC.rxtime_us - now_us+ RX_RAMPUP_us+RX_SETUP_TIME_us >= 0 );
            target = LMIC.rxtime_us;
            target -= RX_RAMPUP_us;
            target -= RX_SETUP_TIME_us;
            LowPowerTimeout_attach_us_absolute(&LMIC.ticker, startRxPing, target);
            return;
        }
        // no - just wait for the beacon
    }

    if( LMIC.txbeg_us != 0  &&  (LMIC.txbeg_us - rxtime_us) < 0 ) {
        goto txdelay;
    }

    setBcnRxParams();
    LMIC.rxsyms = LMIC.bcnRxsyms;
    LMIC.rxtime_us = LMIC.bcnRxtime_us;
    if (now_us - rxtime_us >= 0) {
        LMIC.radio_bh_func = processBeacon;
        os_radio(RADIO_RX);
        return;
    }
    LowPowerTimeout_attach_us_absolute(&LMIC.ticker, startRxBcn, rxtime_us);
    return;

  txdelay:
    EV(devCond, INFO, (e_.reason = EV::devCond_t::TX_DELAY,
                       e_.eui    = MAIN::CDEV->getEui(),
                       e_.info   = (txbeg_us-now_us),
                       e_.info2  = LMIC.FCntUp-1));
    LowPowerTimeout_attach_us_absolute(&LMIC.ticker, runEngineUpdate, LMIC.txbeg_us - TX_RAMPUP_us);
}


void LMIC_setAdrMode (bit_t enabled) {
    LMIC.adrEnabled = enabled ? FCT_ADREN : 0;
}


//  Should we have/need an ext. API like this?
void LMIC_setDrTxpow (dr_t dr, s1_t txpow) {
    setDrTxpow(DRCHG_SET, dr, txpow);
}


void LMIC_shutdown (void)
{
    LowPowerTimeout_detach(&LMIC.ticker);
    os_radio(RADIO_RST);
    LMIC.opmode |= OP_SHUTDOWN;
}


void LMIC_reset (void)
{
    EV(devCond, INFO, (e_.reason = EV::devCond_t::LMIC_EV,
                       e_.eui    = MAIN::CDEV->getEui(),
                       e_.info   = EV_RESET));
    os_radio(RADIO_RST);
    LowPowerTimeout_detach(&LMIC.ticker);

#ifndef DEVADDR
    os_clearMem((xref2u1_t)&LMIC,SIZEOFEXPR(LMIC));

    #if defined(OPTNEG)
    os_clearMem(LMIC.JSEncKey, 16);
    LMIC.JSEncKey[0] = 0x05;
    os_getDevEui(LMIC.JSEncKey+1);
    memcpy(AESkey, root_nwkKey, 16);
    os_aes(AES_ENC, LMIC.JSEncKey, 16);

    os_clearMem(LMIC.JSIntKey, 16);
    LMIC.JSIntKey[0] = 0x06;
    os_getDevEui(LMIC.JSIntKey+1);
    memcpy(AESkey, root_nwkKey, 16);
    os_aes(AES_ENC, LMIC.JSIntKey, 16);

    debug_buf("JSEncKey", LMIC.JSEncKey, 16);
    debug_buf("JSIntKey", LMIC.JSIntKey, 16);
    #endif /* OPTNEG */

#endif
    LMIC.devaddr      =  0;
    LMIC.opmode       =  OP_NONE;
    LMIC.errcr        =  CR_4_5;
    LMIC.adrEnabled   =  FCT_ADREN;
    LMIC.dn2Dr        =  DR_DNW2;   // we need this for 2nd DN window of join accept
    LMIC.dn2Freq      =  FREQ_DNW2; // ditto
    LMIC.rxDelay_sec  =  _DELAY_DNW1_sec;
    LMIC.ping.freq    =  FREQ_PING; // defaults for ping
    LMIC.ping.dr      =  DR_PING;   // ditto
    LMIC.ping.intvExp =  0xFF;
    LMIC.ping.sent_intvExp =  0xFF;

    initDefaultChannels(0);

    DO_DEVDB(LMIC.devaddr,      devaddr);
    DO_DEVDB(LMIC.devNonce,     devNonce);
    DO_DEVDB(LMIC.dn2Dr,        dn2Dr);
    DO_DEVDB(LMIC.dn2Freq,      dn2Freq);
    DO_DEVDB(LMIC.ping.freq,    pingFreq);
    DO_DEVDB(LMIC.ping.dr,      pingDr);
    DO_DEVDB(LMIC.ping.intvExp, pingIntvExp);
} // ..LMIC_reset()


void LMIC_init (void)
{
    _hal_init();
    debug_init();
    radio_init();
    nvm_init();

    LMIC.opmode = OP_SHUTDOWN;

}


void LMIC_clrTxData (void)
{
    LMIC.opmode &= ~(OP_TXDATA|OP_TXRXPEND|OP_POLL);
    LMIC.pendTxLen = 0;
    if( (LMIC.opmode & (OP_JOINING|OP_SCAN)) != 0 ) // do not interfere with JOINING
        return;

    os_radio(RADIO_RST);
    engineUpdate();
}


void LMIC_setTxData (void)
{
    LMIC.opmode |= OP_TXDATA;
    if( (LMIC.opmode & OP_JOINING) == 0 )
        LMIC.txCnt = 0;             // cancel any ongoing TX/RX retries
    engineUpdate();
}

//
int LMIC_setTxData2 (u1_t port, xref2cu1_t data, u1_t dlen, u1_t confirmed)
{
    if (dlen > SIZEOFEXPR(LMIC.pendTxData))
        return -2;
    if (data != (xref2u1_t)0)
        os_copyMem(LMIC.pendTxData, data, dlen);

    LMIC.pendTxConf = confirmed;
    LMIC.pendTxPort = port;
    LMIC.pendTxLen  = dlen;
    LMIC_setTxData();
    return 0;
}


// Send a payload-less message to signal device is alive
void LMIC_sendAlive (void) {
    LMIC.opmode |= OP_POLL;
    engineUpdate();
}


#if defined(JOINEUI) && defined(OPTNEG)
// Check if other networks are around.
void LMIC_tryRejoin (u1_t type)
{
    LMIC.opmode |= OP_REJOIN_;
    LMIC.rejoinType = type;
    engineUpdate();
}
#endif /* JOINEUI && OPTNEG */

#ifdef DEVADDR
/* ABP permanent session keys */
    #ifdef OPTNEG
        const u1_t SNwkSIntKey[16] = SNWKSINTKEY;
        const u1_t NwkSEncKey[16] = NWKSENCKEY;
        const u1_t FNwkSIntKey[16] = FNWKSINTKEY;
    #else
        const u1_t nwkSKey[16] = NWK_S_KEY;
    #endif
const u1_t appSKey[16] = APP_S_KEY;

//! \brief Setup given session keys
//! and put the MAC in a state as if 
//! a join request/accept would have negotiated just these keys.
//! It is crucial that the combinations `devaddr/nwkkey` and `devaddr/artkey`
//! are unique within the network identified by `netid`.
//! NOTE: on Harvard architectures when session keys are in flash:
//!  Caller has to fill in LMIC.{nwk,art}Key  before and pass {nwk,art}Key are NULL
//! \param netid a 24 bit number describing the network id this device is using
//! \param devaddr the 32 bit session address of the device. It is strongly recommended
//!    to ensure that different devices use different numbers with high probability.
//! \param nwkKey  the 16 byte network session key used for message integrity.
//!     If NULL the caller has copied the key into `LMIC.nwkKey` before.
//! \param AppSKey  the 16 byte application router session key used for message confidentiality.
//!     If NULL the caller has copied the key into `LMIC.AppSKey` before.
void LMIC_setSession()
{
    LMIC.netid = NET_ID;
    LMIC.devaddr = DEVADDR;

#ifdef OPTNEG
    LMIC.flags.OptNeg = 1;
#else
    LMIC.flags.OptNeg = 0;
#endif /* OPTNEG */

//    debug_buf("nwkSKey", nwkSKey, 16);
//    debug_buf("LMIC.FNwkSIntKey", LMIC.FNwkSIntKey, 16);
    
    initDefaultChannels(0);
 
    LMIC.opmode &= ~(OP_JOINING|OP_TRACK|OP_REJOIN|OP_TXRXPEND|OP_PINGINI);
    LMIC.opmode |= OP_NEXTCHNL;
    stateJustJoined();
    DO_DEVDB(LMIC.netid,   netid);
    DO_DEVDB(LMIC.devaddr, devaddr);
    DO_DEVDB(LMIC.nwkSKey,  nwkkey);
    DO_DEVDB(LMIC.AppSKey,  artkey);
    DO_DEVDB(LMIC.FCntUp, FCntUp);
    DO_DEVDB(LMIC.FCntDown, FCntDown);
}
#endif /* DEVADDR */

// Enable/disable link check validation.
// LMIC sets the ADRACKREQ bit in UP frames if there were no DN frames
// for a while. It expects the network to provide a DN message to prove
// connectivity with a span of UP frames. If this no such prove is coming
// then the datarate is lowered and a LINK_DEAD event is generated.
// This mode can be disabled and no connectivity prove (ADRACKREQ) is requested
// nor is the datarate changed.
// This must be called only if a session is established (e.g. after EV_JOINED)
void LMIC_setLinkCheckMode (bit_t enabled) {
    LMIC.flags.adrChanged = 0;
    LMIC.adrAckReq = enabled ? LINK_CHECK_INIT : LINK_CHECK_OFF;
}

 
void LMIC_mainloop()
{
    if (LMIC.radio_bottom_half) {
        if (LMIC.radio_bh_func)
            LMIC.radio_bh_func();

        LMIC.radio_bottom_half = false;
    }

    nvm_service();

    sleep_manager_sleep_auto();

    if (LMIC.opmode & OP_ENGUP) {
        LMIC.opmode &= ~OP_ENGUP;
        engineUpdate();
    }

}
