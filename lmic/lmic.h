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

//! @file
//! @brief LMIC API

#ifndef _lmic_h_
#define _lmic_h_

#include "oslmic.h"
#if defined(CFG_us915)
#include "lmic_us915.h"
#elif defined(CFG_eu868)
#include "lmic_eu868.h"
#endif
#include "lorabase.h"
#include "loraconfig.h"

// LMIC version
#define LMIC_VERSION_MAJOR 1
#define LMIC_VERSION_MINOR 6
#define LMIC_VERSION_BUILD 1468577746

#ifdef DEVADDR
    #if (DEVADDR & 0xff000000) == 0xfe000000    // NetID type 6
        #define NWK_ID      ((DEVADDR >> 7) & 0x1ffff)  // 17bits
        #define NET_ID      (0xe00000 | NWK_ID)
    #elif (DEVADDR & 0xfe000000) == 0xfc000000  // NetID type 5
        #define NWK_ID      ((DEVADDR >> 13) & 0x1fff)  // 13bits
        #define NET_ID      (0xa00000 | NWK_ID)
    #elif (DEVADDR & 0xf8000000) == 0xf0000000  // NetID type 4
        #define NWK_ID      ((DEVADDR >> 16) & 0x7ff)  // 11bits
        #define NET_ID      (0x800000 | NWK_ID) // 21lsb id
    #elif (DEVADDR & 0xf0000000) == 0xe0000000  // NetID type 3
        #define NWK_ID      ((DEVADDR >> 18) & 0x3ff)  // 10bits
        #define NET_ID      (0x600000 | NWK_ID) // 21lsb id
    #elif (DEVADDR & 0xe0000000) == 0xc0000000  // NetID type 2
        #define NWK_ID      ((DEVADDR >> 20) & 0x1ff)  // 9bits
        #define NET_ID      (0x400000 | NWK_ID) // 9lsb id
    #elif (DEVADDR & 0xc0000000) == 0x80000000  // NetID type 1
        #define NWK_ID      ((DEVADDR >> 24) & 0x3f)  // 6bits
        #define NET_ID      (0x200000 | NWK_ID) // 6lsb id
    #elif (DEVADDR & 0x80000000) == 0x00000000  // NetID type 0
        #define NWK_ID      ((DEVADDR >> 25) & 0x3f)  // 6bits
        #define NET_ID      (0x000000 | NWK_ID) // 6lsb id
    #endif

    #if defined(SNWKSINTKEY) && defined(NWKSENCKEY) && defined(FNWKSINTKEY)
        #define OPTNEG
        extern const u1_t SNwkSIntKey[16];
        extern const u1_t NwkSEncKey[16];
        extern const u1_t FNwkSIntKey[16];
    #else
        #undef OPTNEG
        extern const u1_t nwkSKey[16];
    #endif
    extern const u1_t appSKey[16];
#elif defined(JOINEUI)
    /* OTA */
    #ifdef ROOT_APPKEY          
        #define OPTNEG
    #else
        #undef OPTNEG
    #endif
#endif 

#ifndef OPTNEG
    /* v1.0 lorawan only uses a single network session key */
    #define NwkSEncKey      NwkSKey
    #define SNwkSIntKey     NwkSKey
    #define FNwkSIntKey     NwkSKey
#endif

//! Only For Antenna Tuning Tests !
//#define CFG_TxContinuousMode 1

enum { MAX_FRAME_LEN      =  64 };   //!< Library cap on max frame length
enum { TXCONF_ATTEMPTS    =   8 };   //!< Transmit attempts for confirmed frames
enum { MAX_MISSED_BCNS    =  20 };   // threshold for triggering rejoin requests
enum { MAX_RXSYMS         = 100 };   // stop tracking beacon beyond this

enum { LINK_CHECK_CONT    =  12 ,    // continue with this after reported dead link
       LINK_CHECK_DEAD    =  24 ,    // after this UP frames and no response from NWK assume link is dead
       LINK_CHECK_INIT    = -12 ,    // UP frame count until we inc datarate
       LINK_CHECK_OFF     =-128 };   // link check disabled

enum { TIME_RESYNC        = 6*128 }; // secs
enum { TXRX_GUARD_ms      =  6000 };  // msecs - don't start TX-RX transaction before beacon
enum { TXRX_GUARD_us      =  TXRX_GUARD_ms * 1000};
enum { JOIN_GUARD_ms      =  9000 };  // msecs - don't start Join Req/Acc transaction before beacon
enum { JOIN_GUARD_us      =  JOIN_GUARD_ms * 1000};
enum { TXRX_BCNEXT_secs   =     2 };  // secs - earliest start after beacon time
enum { RETRY_PERIOD_secs  =     3 };  // secs - random period for retrying a confirmed send

// Keep in sync with evdefs.hpp::drChange
enum { DRCHG_SET, DRCHG_NOJACC, DRCHG_NOACK, DRCHG_NOADRACK, DRCHG_NWKCMD };
enum { KEEP_TXPOW = -128 };


//! \internal
struct rxsched_t {
    u1_t     dr;
    u1_t     intvExp;   // 0..7
    u1_t     sent_intvExp;   // 0..7
    u1_t     slot;      // runs from 0 to 128
    u1_t     rxsyms;
    us_timestamp_t  rxbase_us;
    us_timestamp_t  rxtime_us;    // start of next spot
    u4_t     freq;
};
TYPEDEF_xref2rxsched_t;  //!< \internal


//! Parsing and tracking states of beacons.
enum { BCN_NONE    = 0x00,   //!< No beacon received
       BCN_PARTIAL = 0x01,   //!< Only first (common) part could be decoded (info,lat,lon invalid/previous)
       BCN_FULL    = 0x02,   //!< Full beacon decoded
       BCN_NODRIFT = 0x04,   //!< No drift value measured yet
       BCN_NODDIFF = 0x08 }; //!< No differential drift measured yet
//! Information about the last and previous beacons.
struct bcninfo_t {
    us_timestamp_t  txtime_us;  //!< Time when the beacon was sent
    s1_t     rssi;    //!< Adjusted RSSI value of last received beacon
    s1_t     snr;     //!< Scaled SNR value of last received beacon
    u1_t     flags;   //!< Last beacon reception and tracking states. See BCN_* values.
    u4_t     time;    //!< GPS time in seconds of last beacon (received or surrogate)
    //
    u1_t     info;    //!< Info field of last beacon (valid only if BCN_FULL set)
    s4_t     lat;     //!< Lat field of last beacon (valid only if BCN_FULL set)
    s4_t     lon;     //!< Lon field of last beacon (valid only if BCN_FULL set)
};

// purpose of receive window - lmic_t.rxState
enum { RADIO_RST=0, RADIO_TX=1, RADIO_RX=2, RADIO_RXON=3 };
// Netid values /  lmic_t.netid
enum { NETID_NONE=(int)~0U, NETID_MASK=(int)0xFFFFFF };
// MAC operation modes (lmic_t.opmode).
enum { OP_NONE     = 0x0000,
       OP_SCAN     = 0x0001, // radio scan to find a beacon
       OP_TRACK    = 0x0002, // track my networks beacon (netid)
       OP_JOINING  = 0x0004, // device joining in progress (blocks other activities)
       OP_TXDATA   = 0x0008, // TX user data (buffered in pendTxData)
       OP_POLL     = 0x0010, // send empty UP frame to ACK confirmed DN/fetch more DN data
       OP_REJOIN_  = 0x0020, // occasionally send JOIN REQUEST
       OP_SHUTDOWN = 0x0040, // prevent MAC from doing anything
       OP_TXRXPEND = 0x0080, // TX/RX transaction pending
       OP_RNDTX    = 0x0100, // prevent TX lining up after a beacon
       OP_PINGINI  = 0x0200, // pingable is initialized and scheduling active
       OP_PINGABLE = 0x0400, // we're pingable
       OP_NEXTCHNL = 0x0800, // find a new channel
       OP_LINKDEAD = 0x1000, // link was reported as dead
       OP_TESTMODE = 0x2000, // developer test mode
       OP_ENGUP    = 0x4000, // run engineUpdate outside of ISR context
};
// TX-RX transaction flags - report back to user
enum { TXRX_ACK    = 0x80,   // confirmed UP frame was acked
       TXRX_NACK   = 0x40,   // confirmed UP frame was not acked
       TXRX_NOPORT = 0x20,   // set if a frame with a port was RXed, clr if no frame/no port
       TXRX_PORT   = 0x10,   // set if a frame with a port was RXed, LMIC.frame[LMIC.dataBeg-1] => port
       TXRX_DNW1   = 0x01,   // received in 1st DN slot
       TXRX_DNW2   = 0x02,   // received in 2dn DN slot
       TXRX_PING   = 0x04 }; // received in a scheduled RX slot
// Event types for event callback
enum _ev_t { EV_SCAN_TIMEOUT=1, EV_BEACON_FOUND,
             EV_BEACON_MISSED, EV_BEACON_TRACKED,
#ifdef JOINEUI
             EV_JOINING, EV_JOINED, EV_JOIN_FAILED, EV_REJOIN_FAILED,
#endif /* JOINEUI */
             EV_RFU1,
             EV_TXCOMPLETE, EV_LOST_TSYNC, EV_RESET,
             EV_RXCOMPLETE, EV_LINK_DEAD, EV_LINK_ALIVE, EV_SCAN_FOUND,
             EV_TXSTART, EV_LINKCHECK_ANS, EV_DEVICE_TIME_ANS };
typedef enum _ev_t ev_t;

typedef struct {
    bool forced;
    uint8_t dr;
    uint8_t type;
    uint8_t retries;
    uint8_t Period;
    ticker_t ticker;
    struct {
        uint8_t MaxTimeN;
        uint8_t MaxCountN;
        unsigned uplinks_since;
        bool enabled;
    } type0;
} rejoin_t;

struct lmic_t {
    // Radio settings TX/RX (also accessed by HAL)
    us_timestamp_t  txstart_us;
    us_timestamp_t  txbeg_us;
    us_timestamp_t  txend_us;
    us_timestamp_t  rxtime_us;
    u4_t        freq;
    s1_t        rssi;
    s1_t        snr;
    rps_t       rps;
    u1_t        rxsyms;
    u1_t        dndr;
    u1_t        RX1DRoffset;
    s1_t        txpow;     // dBm

    ticker_t    ticker;

    // Channel scheduling
    region_t region;
    u1_t        txChnl;          // channel for next TX
    u1_t        globalDutyRate;  // max rate: 1/2^k
    us_timestamp_t  globalDutyAvail_us; // time device can send again
    
    u4_t        netid;        // current network id (~0 - none)
    u2_t        opmode;
    u1_t        upRepeat;     // configured up repeat
    s1_t        adrTxPow;     // ADR adjusted TX power
    u1_t        datarate;     // current data rate
    u1_t        errcr;        // error coding rate (used for TX only)
#ifdef JOINEUI
    u1_t        joinTries;    // adjustment for rejoin datarate
    #ifdef OPTNEG
    u1_t        JoinReqType;
    u1_t        rejoinType;
    u2_t        RJcount0;
    #endif
#endif /* JOINEUI */
    s2_t        drift;        // last measured drift
    s2_t        lastDriftDiff;
    s2_t        maxDriftDiff;

    u1_t        pendTxPort;
    u1_t        pendTxConf;   // confirmed data
    u1_t        pendTxLen;    // +0x80 = confirmed
    u1_t        pendTxData[MAX_LEN_PAYLOAD];

    u2_t        devNonce;     // last generated nonce

#ifdef DEVADDR
    #ifdef OPTNEG
    const u1_t* const SNwkSIntKey;
    const u1_t* const NwkSEncKey;
    const u1_t* const FNwkSIntKey;
    #else
    const u1_t* const NwkSKey;
    #endif

    const u1_t* const AppSKey;
#else 
    /* OTA */
    #ifdef OPTNEG
    u1_t        JSEncKey[16];   // join
    u1_t        JSIntKey[16];   // join
    u1_t        SNwkSIntKey[16];   // sNS MIC key (serving mac portion)
    u1_t        NwkSEncKey[16];   // NS encryption key
    u1_t        FNwkSIntKey[16];   // fNS MIC key (forwarding portion)
    #else
    u1_t        NwkSKey[16];   // one session key for NS
    #endif

    u1_t        AppSKey[16];   // application router session key
    u4_t        NFCntDown;      // device level down stream seqno
    #ifdef OPTNEG
    u4_t        AFCntDown;      // device level down stream seqno
    rejoin_t    rejoin;
    #endif
    u4_t        FCntUp;
#endif
    devaddr_t   devaddr;
    u2_t        ConfFCntUp;
    u2_t        ConfFCntDown;

    u1_t        dnConf;       // dn frame confirm pending: LORA::FCT_ACK or 0
    s1_t        adrAckReq;    // counter until we reset data rate (0=off)

    u1_t        margin;
    u1_t        LinkADRAns;      // link adr adapt answer pending
    u1_t        adrEnabled;
    u1_t        NewChannelAns;      // answer set new channel

    struct {
        uint16_t OptNeg           : 1;    // 0
        uint16_t RXTimingSetupAns : 1;    // 1
        uint16_t DevStatusAns     : 1;    // 2
        uint16_t DutyCycleAns     : 1;    // 3
        uint16_t adrChanged       : 1;    // 4
        uint16_t LinkCheckReq     : 1;    // 5
        uint16_t DeviceTimeReq    : 1;    // 6
        uint16_t PingSlotInfoReq  : 1;    // 7
#if defined(JOINEUI) && defined(OPTNEG)
        uint16_t need_ReKeyConf   : 1;    // 8
#endif
    } flags;

    u1_t        rxDelay_sec;
    // 2nd RX window (after up stream)
    u1_t        dn2Dr;
    u4_t        dn2Freq;
    u1_t        RXParamSetupAns;       // 0=no answer pend, 0x80+ACKs

    // Class B state
    u1_t        missedBcns;   // unable to track last N beacons
    u1_t        bcninfoTries; // how often to try (scan mode only)
    u1_t        PingSlotChannelAns;   // answer set cmd and ACK bits
    rxsched_t   ping;         // pingable setup

    // Public part of MAC state
    u1_t        txCnt;
    u1_t        txrxFlags;  // transaction flags (TX-RX combo)
    u1_t        dataBeg;    // 0 or start of data (dataBeg-1 is port)
    u1_t        dataLen;    // 0 no data or zero length data, >0 byte count of data
    u1_t        frame[MAX_LEN_FRAME];

    u1_t        bcnChnl;
    u1_t        bcnRxsyms;    // 
    us_timestamp_t  bcnRxtime_us;
    bcninfo_t   bcninfo;      // Last received beacon info

    u1_t        noRXIQinversion;
    u1_t        serverVersion;  // from ReKeyConf

    void (*radio_bh_func)(void);
    bool radio_bottom_half;
};
//! \var struct lmic_t LMIC
//! The state of LMIC MAC layer is encapsulated in this variable.
DECLARE_LMIC; //!< \internal

//! Construct a bit map of allowed datarates from drlo to drhi (both included). 
#define DR_RANGE_MAP(drlo,drhi) (((u2_t)0xFFFF<<(drlo)) & ((u2_t)0xFFFF>>(15-(drhi))))
bit_t LMIC_setupChannel (u1_t channel, u4_t freq, u2_t drmap, s1_t band);
void  LMIC_disableChannel (u1_t channel);

void  LMIC_setDrTxpow   (dr_t dr, s1_t txpow);  // set default/start DR/txpow
void  LMIC_setAdrMode   (bit_t enabled);        // set ADR mode (if mobile turn off)
#if defined(JOINEUI) && defined(ROOT_NWKKEY)
bit_t LMIC_startJoining (void);
#endif /* JOINEUI && ROOT_NWKKEY */

void  LMIC_shutdown     (void);
void  LMIC_init         (void);
void  LMIC_reset        (void);
void  LMIC_clrTxData    (void);
void  LMIC_setTxData    (void);
int   LMIC_setTxData2   (u1_t port, xref2cu1_t data, u1_t dlen, u1_t confirmed);
void  LMIC_sendAlive    (void);

bit_t LMIC_enableTracking  (u1_t tryBcnInfo);
void  LMIC_disableTracking (void);

void  LMIC_stopPingable  (void);
void  LMIC_setPingable   (u1_t intvExp);
#if defined(JOINEUI) && defined(OPTNEG)
void  LMIC_tryRejoin     (u1_t type);
#endif /* JOINEUI && OPTNEG */

//void LMIC_setSession (devaddr_t devaddr, const u1_t * const nwkKey, const u1_t * const AppSKey);
#ifdef DEVADDR
void LMIC_setSession (void);
#endif
void LMIC_setLinkCheckMode (bit_t enabled);



// Special APIs - for development or testing
// !!!See implementation for caveats!!!

/****************** region... ******************/
extern const u1_t txdr_to_rx1dr[][N_UP_DR];    // [Rx1DRoffset][txdr]
u1_t mapChannels (u1_t chpage, u2_t chmap);
u4_t convFreq (xref2u1_t ptr);
void setRx1Params(void);
u1_t nextJoinState(void);
void initDefaultChannels(bit_t join);
void initJoinLoop(void);
void setBcnRxParams(void);
us_timestamp_t nextTx(us_timestamp_t);
void updateTx(void);
void updatePostTx(void);
bool regionBeaconCrcCheck(xref2u1_t);
void regionProcessBeacon(void);
bool regionIsFSK(void);

/****************** ...region ******************/

void setDrJoin (u1_t reason, u1_t dr);
void LMIC_mainloop(void);
us_timestamp_t rndDelay (u1_t secSpan);

// Special APIs - for development or testing
#define isTESTMODE() 0

u1_t onEvent(ev_t e, const void* args);

#endif // _lmic_h_
