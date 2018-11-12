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

#ifndef _lorabase_h_
#define _lorabase_h_

// ================================================================================
// BEG: Keep in sync with lorabase.hpp
//
#define UP_LINK                0
#define DOWN_LINK              1

enum _cr_t { CR_4_5=0, CR_4_6, CR_4_7, CR_4_8 };
enum _sf_t { FSK=0, SF7, SF8, SF9, SF10, SF11, SF12, SFrfu };
enum _bw_t { BW125=0, BW250, BW500, BWrfu };
typedef u1_t cr_t;
typedef u1_t sf_t;
typedef u1_t bw_t;
typedef u1_t dr_t;
// Radio parameter set (encodes SF/BW/CR/IH/NOCRC)
typedef u2_t rps_t;
TYPEDEF_xref2rps_t;

enum { ILLEGAL_RPS = 0xFF };

// Global maximum frame length
enum { STD_PREAMBLE_LEN  =  8 };
enum { MAX_LEN_FRAME     = 64 };
enum { LEN_DEVNONCE      =  2 };
enum { LEN_ARTNONCE      =  3 };
enum { LEN_NETID         =  3 };
enum { DELAY_JACC1_sec   =  5 }; // in secs
enum { DELAY_JACC1_us    = DELAY_JACC1_sec * 1000000}; // in usecs
enum { _DELAY_DNW1_sec    = 1 }; // in secs down window #1
enum { _DELAY_DNW1_us     = _DELAY_DNW1_sec * 1000000}; // in usecs down window #1
enum { DELAY_EXTDNW2_sec = 1 }; // in secs
enum { DELAY_JACC2_sec   = DELAY_JACC1_sec + (int)DELAY_EXTDNW2_sec }; // in secs
enum { DELAY_JACC2_us    = DELAY_JACC2_sec * 1000000};
enum { _DELAY_DNW2_sec    = _DELAY_DNW1_sec  + (int)DELAY_EXTDNW2_sec }; // in secs down window #1
enum { _DELAY_DNW2_us     = _DELAY_DNW2_sec * 1000000};
enum { BCN_INTV_exp      = 7 };
enum { BCN_INTV_sec      = 1<<BCN_INTV_exp };
enum { BCN_INTV_ms       = BCN_INTV_sec * 1000L};
enum { BCN_INTV_us       = BCN_INTV_ms * 1000L};
enum { BCN_RESERVE_ms    = 2120 };   // space reserved for beacon and NWK management
enum { BCN_GUARD_ms      = 3000 };   // end of beacon period to prevent interference with beacon
enum { BCN_SLOT_SPAN_ms  =   30 };   // 2^12 reception slots a this span
enum { BCN_WINDOW_ms     = BCN_INTV_ms-(int)BCN_GUARD_ms-(int)BCN_RESERVE_ms };
enum { BCN_WINDOW_us     = BCN_WINDOW_ms*1000L};
enum { BCN_RESERVE_us    = 2120000 };
enum { BCN_GUARD_us      = 3000000 };
enum { BCN_SLOT_SPAN_us  =   30000 };


enum {
    // Rejoin type 0,2 frame format
    OFF_RJ02_HDR      = 0,
    OFF_RJ02_TYPE     = OFF_RJ02_HDR + 1,
    OFF_RJ02_NETID    = OFF_RJ02_TYPE + 1,
    OFF_RJ02_DEVEUI   = OFF_RJ02_NETID + 3,
    OFF_RJ02_RJCOUNT0 = OFF_RJ02_DEVEUI + 8,
    OFF_RJ0_MIC       = OFF_RJ02_RJCOUNT0 + 2
};
enum {
    // Rejoin type 1 frame format
    OFF_RJ1_HDR      = 0,
    OFF_RJ1_TYPE     = OFF_RJ1_HDR + 1,
    OFF_RJ1_JOINEUI  = OFF_RJ1_TYPE + 1,
    OFF_RJ1_DEVEUI   = OFF_RJ1_JOINEUI + 8,
    OFF_RJ1_RJCOUNT1 = OFF_RJ1_DEVEUI + 8,
    OFF_RJ1_MIC      = OFF_RJ1_RJCOUNT1 + 2
};
enum {
    // Join Request frame format
    OFF_JR_HDR      = 0,
    OFF_JR_ARTEUI   = 1,
    OFF_JR_DEVEUI   = 9,
    OFF_JR_DEVNONCE = 17,
    OFF_JR_MIC      = 19,
    LEN_JR          = 23
};
enum {
    // Join Accept frame format
    OFF_JA_HDR      = 0,
    OFF_JA_ARTNONCE = 1,
    OFF_JA_NETID    = 4,
    OFF_JA_DEVADDR  = 7,
    OFF_JA_RFU      = 11,
    OFF_JA_DLSET    = 11,
    OFF_JA_RXDLY    = 12,
    OFF_CFLIST      = 13,
    LEN_JA          = 17,
    LEN_JAEXT       = 17+16
};
enum {
    // Data frame format
    OFF_DAT_HDR      = 0,
    OFF_DAT_ADDR     = 1,
    OFF_DAT_FCT      = 5,
    OFF_DAT_SEQNO    = 6,
    OFF_DAT_OPTS     = 8,
};
enum { MAX_LEN_PAYLOAD = MAX_LEN_FRAME-(int)OFF_DAT_OPTS-4 };
enum {
    // Bitfields in frame format octet
    HDR_FTYPE   = 0xE0,
    HDR_RFU     = 0x1C,
    HDR_MAJOR   = 0x03
};
enum { HDR_FTYPE_DNFLAG = 0x20 };  // flags DN frame except for HDR_FTYPE_PROP
enum {
    // Values of frame type bit field
    HDR_FTYPE_JREQ   = 0x00,
    HDR_FTYPE_JACC   = 0x20,
    HDR_FTYPE_DAUP   = 0x40,  // data (unconfirmed) up
    HDR_FTYPE_DADN   = 0x60,  // data (unconfirmed) dn
    HDR_FTYPE_DCUP   = 0x80,  // data confirmed up
    HDR_FTYPE_DCDN   = 0xA0,  // data confirmed dn
    HDR_FTYPE_REJOIN = 0xC0,  // rejoin for roaming
    HDR_FTYPE_PROP   = 0xE0
};
enum {
    HDR_MAJOR_V1 = 0x00,
};
enum {
    // Bitfields in frame control octet
    FCT_ADREN  = 0x80,
    FCT_ADRARQ = 0x40,
    FCT_ACK    = 0x20,
    FCT_MORE   = 0x10,   // also in DN direction: Class B indicator
    FCT_OPTLEN = 0x0F,
};
enum {
    // In UP direction: signals class B enabled
    FCT_CLASSB = FCT_MORE
};
enum {
    NWKID_MASK = (int)0xFE000000,
    NWKID_BITS = 7
};

// MAC uplink commands   downwlink too
enum {
    // Class A
    LinkCheckReq       = 0x02, // -  link check request : -
    LinkADRAns         = 0x03, // -  link ADR answer    : u1:7-3:RFU, 3/2/1: pow/DR/Ch ACK
    DutyCycleAns       = 0x04, // -  duty cycle answer  : -
    RXParamSetupAns    = 0x05, // -  2nd DN slot status : u1:7-2:RFU  1/0:datarate/channel ack
    DevStatusAns       = 0x06, // -  device status ans  : u1:battery 0,1-254,255=?, u1:7-6:RFU,5-0:margin(-32..31)
    NewChannelAns      = 0x07, // -  set new channel    : u1: 7-2=RFU, 1/0:DR/freq ACK
    RXTimingSetupAns   = 0x08, // - ans timing of rx slots : -
    RekeyInd           = 0x0b, // -                     : -
    DeviceTimeReq      = 0x0d, // -  device time request : -
    // Class B
    PingSlotInfoReq    = 0x10, // -  pingability indic  : u1: 7=RFU, 6-4:interval, 3-0:datarate
    PingSlotChannelAns = 0x11, // -  ack ping freq      : u1: 7-1:RFU, 0:freq ok
    BeaconTimingReq    = 0x12, // -  next beacon start  : -
};

// MAC downlink commands
enum {
    // Class A
    LinkCheckAns       = 0x02, // link check answer  : u1:margin 0-254,255=unknown margin / u1:gwcnt
    LinkADRReq         = 0x03, // link ADR request   : u1:DR/TXPow, u2:chmask, u1:chpage/repeat
    DutyCycleReq       = 0x04, // duty cycle cap     : u1:255 dead [7-4]:RFU, [3-0]:cap 2^-k
    RXParamSetupReq    = 0x05, // 2nd DN window param: u1:7-4:RFU/3-0:datarate, u3:freq
    DevStatusReq       = 0x06, // device status req  : -
    NewChannelReq      = 0x07, // set new channel    : u1:chidx, u3:freq, u1:DRrange
    RXTimingSetupReq   = 0x08, // - set timing of rx slots : u1: 
    RekeyConf          = 0x0b, //                  : u1: server version
    DeviceTimeAns      = 0x0d, //                  : u1-4:seconds, u5:subseconds
    ForceRejoinReq     = 0x0e, // -  force rejoin request : -
    // Class B
    PingSlotInfoAns    = 0x10,  // 
    PingSlotChannelReq = 0x11, // set ping freq      : u3: freq
    BeaconTimingAns    = 0x12, // next beacon start  : u2: delay(in TUNIT millis), u1:channel
};

enum {
    MCMD_BCNI_TUNIT_us = 30000  // time unit of delay value in micros
};
enum {
    MCMD_LADR_ANS_RFU    = 0xF8, // RFU bits
    MCMD_LADR_ANS_POWACK = 0x04, // 0=not supported power level
    MCMD_LADR_ANS_DRACK  = 0x02, // 0=unknown data rate
    MCMD_LADR_ANS_CHACK  = 0x01, // 0=unknown channel enabled
};
enum {
    MCMD_DN2P_ANS_RFU       = 0xf8, // RFU bits
    MCMD_DN2P_ANS_RX1OFSACK = 0x04, // 0=rx1 dr offset bad
    MCMD_DN2P_ANS_DRACK     = 0x02, // 0=unknown data rate
    MCMD_DN2P_ANS_CHACK     = 0x01, // 0=unknown channel enabled
};
enum {
    MCMD_SNCH_ANS_RFU    = 0xFC, // RFU bits
    MCMD_SNCH_ANS_DRACK  = 0x02, // 0=unknown data rate
    MCMD_SNCH_ANS_FQACK  = 0x01, // 0=rejected channel frequency
};
enum {
    MCMD_PING_ANS_RFU   = 0xFE,
    MCMD_PING_ANS_FQACK = 0x01,
    MCMD_PING_ANS_DRACK = 0x02
};

enum {
    MCMD_DEVS_EXT_POWER   = 0x00, // external power supply
    MCMD_DEVS_BATT_MIN    = 0x01, // min battery value
    MCMD_DEVS_BATT_MAX    = 0xFE, // max battery value
    MCMD_DEVS_BATT_NOINFO = 0xFF, // unknown battery level
};

// Bit fields byte#3 of LinkADRReq payload
enum {
    MCMD_LADR_N3RFU_MASK  = 0x80,
    MCMD_LADR_CHPAGE_MASK = 0xF0,   // ChMaskCntl
    MCMD_LADR_REPEAT_MASK = 0x0F,   // NbTrans
    MCMD_LADR_REPEAT_1    = 0x01,
    MCMD_LADR_CHPAGE_1    = 0x10
};
#define MCMD_LADR_CHPAGE_SHIFT      4
// Bit fields byte#0 of LinkADRReq payload
enum {
    MCMD_LADR_DR_MASK    = 0xF0,
    MCMD_LADR_POW_MASK   = 0x0F,
    MCMD_LADR_DR_SHIFT   = 4,
    MCMD_LADR_POW_SHIFT  = 0,
#if defined(CFG_eu868)
    MCMD_LADR_SF12      = DR_SF12<<4,
    MCMD_LADR_SF11      = DR_SF11<<4,
    MCMD_LADR_SF10      = DR_SF10<<4,
    MCMD_LADR_SF9       = DR_SF9 <<4,
    MCMD_LADR_SF8       = DR_SF8 <<4,
    MCMD_LADR_SF7       = DR_SF7 <<4,
    MCMD_LADR_SF7B      = DR_SF7B<<4,
    MCMD_LADR_FSK       = DR_FSK <<4,

    MCMD_LADR_20dBm     = 0,
    MCMD_LADR_14dBm     = 1,
    MCMD_LADR_11dBm     = 2,
    MCMD_LADR_8dBm      = 3,
    MCMD_LADR_5dBm      = 4,
    MCMD_LADR_2dBm      = 5,
#elif defined(CFG_us915)
    MCMD_LADR_SF10      = DR_SF10<<4,
    MCMD_LADR_SF9       = DR_SF9 <<4,
    MCMD_LADR_SF8       = DR_SF8 <<4,
    MCMD_LADR_SF7       = DR_SF7 <<4,
    MCMD_LADR_SF8C      = DR_SF8C<<4,
    MCMD_LADR_SF12CR    = DR_SF12CR<<4,
    MCMD_LADR_SF11CR    = DR_SF11CR<<4,
    MCMD_LADR_SF10CR    = DR_SF10CR<<4,
    MCMD_LADR_SF9CR     = DR_SF9CR<<4,
    MCMD_LADR_SF8CR     = DR_SF8CR<<4,
    MCMD_LADR_SF7CR     = DR_SF7CR<<4,

    MCMD_LADR_30dBm     = 0,
    MCMD_LADR_28dBm     = 1,
    MCMD_LADR_26dBm     = 2,
    MCMD_LADR_24dBm     = 3,
    MCMD_LADR_22dBm     = 4,
    MCMD_LADR_20dBm     = 5,
    MCMD_LADR_18dBm     = 6,
    MCMD_LADR_16dBm     = 7,
    MCMD_LADR_14dBm     = 8,
    MCMD_LADR_12dBm     = 9,
    MCMD_LADR_10dBm     = 10
#endif
};

// Device address
typedef u4_t devaddr_t;

// RX quality (device)
enum { RSSI_OFF=64, SNR_SCALEUP=4 };

inline sf_t  getSf   (rps_t params)            { return   (sf_t)(params &  0x7); }
inline rps_t setSf   (rps_t params, sf_t sf)   { return (rps_t)((params & ~0x7) | sf); }
inline bw_t  getBw   (rps_t params)            { return  (bw_t)((params >> 3) & 0x3); }
inline rps_t setBw   (rps_t params, bw_t cr)   { return (rps_t)((params & ~0x18) | (cr<<3)); }
inline cr_t  getCr   (rps_t params)            { return  (cr_t)((params >> 5) & 0x3); }
inline rps_t setCr   (rps_t params, cr_t cr)   { return (rps_t)((params & ~0x60) | (cr<<5)); }
inline int   getNocrc(rps_t params)            { return        ((params >> 7) & 0x1); }
inline rps_t setNocrc(rps_t params, int nocrc) { return (rps_t)((params & ~0x80) | (nocrc<<7)); }
inline int   getIh   (rps_t params)            { return        ((params >> 8) & 0xFF); }
inline rps_t setIh   (rps_t params, int ih)    { return (rps_t)((params & ~0xFF00) | (ih<<8)); }
inline rps_t makeRps (sf_t sf, bw_t bw, cr_t cr, int ih, int nocrc) {
    return sf | (bw<<3) | (cr<<5) | (nocrc?(1<<7):0) | ((ih&0xFF)<<8);
}
#define MAKERPS(sf,bw,cr,ih,nocrc) ((rps_t)((sf) | ((bw)<<3) | ((cr)<<5) | ((nocrc)?(1<<7):0) | ((ih&0xFF)<<8)))
// Two frames with params r1/r2 would interfere on air: same SFx + BWx 
inline int sameSfBw(rps_t r1, rps_t r2) { return ((r1^r2)&0x1F) == 0; }

extern const u1_t _DR2RPS_CRC[];
inline rps_t updr2rps (dr_t dr) { return (rps_t)_DR2RPS_CRC[dr+1]; }
inline rps_t dndr2rps (dr_t dr) { return setNocrc(updr2rps(dr),1); }
inline int isFasterDR (dr_t dr1, dr_t dr2) { return dr1 > dr2; }
inline int isSlowerDR (dr_t dr1, dr_t dr2) { return dr1 < dr2; }
inline dr_t  incDR    (dr_t dr) { return _DR2RPS_CRC[dr+2]==ILLEGAL_RPS ? dr : (dr_t)(dr+1); } // increase data rate
inline dr_t  decDR    (dr_t dr) { return _DR2RPS_CRC[dr  ]==ILLEGAL_RPS ? dr : (dr_t)(dr-1); } // decrease data rate
inline dr_t  assertDR (dr_t dr) { return _DR2RPS_CRC[dr+1]==ILLEGAL_RPS ? DR_DFLTMIN : dr; }   // force into a valid DR
inline bit_t validDR  (dr_t dr) { return _DR2RPS_CRC[dr+1]!=ILLEGAL_RPS; } // in range
inline dr_t  lowerDR  (dr_t dr, u1_t n) { while(n--){dr=decDR(dr);} return dr; } // decrease data rate by n steps

//
// BEG: Keep in sync with lorabase.hpp
// ================================================================================

// Convert between dBm values and power codes (MCMD_LADR_XdBm)
//s1_t pow2dBm (u1_t mcmdLadr_p1);
// Sensitivity at given SF/BW
int getSensitivity (rps_t rps);


#endif // _lorabase_h_
