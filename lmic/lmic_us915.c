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
#if defined(CFG_us915)
#include "lmic.h"

void initDefaultChannels(bit_t join)
{
#ifdef FSB
    for (u1_t i=0; i<4; i++)
        LMIC.region.channelMap[i] = 0x0000;
    LMIC.region.channelMap[FSB >> 1] = (FSB & 1) ? 0xff00 : 0x00ff;
    LMIC.region.channelMap[4] = 1 << FSB;
#else
    for (u1_t i=0; i<4; i++)
        LMIC.region.channelMap[i] = 0xFFFF;
    LMIC.region.channelMap[4] = 0x00FF;
#endif
}

u4_t convFreq (xref2u1_t ptr) {
    u4_t freq = (os_rlsbf4(ptr-1) >> 8) * 100;
    if( freq < US915_FREQ_MIN || freq > US915_FREQ_MAX )
        freq = 0;
    return freq;
}

bit_t LMIC_setupChannel (u1_t chidx, u4_t freq, u2_t drmap, s1_t band) {
    if( chidx < 72 || chidx >= 72+MAX_XCHANNELS )
        return 0; // channels 0..71 are hardwired
    chidx -= 72;
    LMIC.region.xchFreq[chidx] = freq;
    LMIC.region.xchDrMap[chidx] = drmap==0 ? DR_RANGE_MAP(DR_SF10,DR_SF8C) : drmap;
    LMIC.region.channelMap[chidx>>4] |= (1<<(chidx&0xF));
    return 1;
}

void LMIC_disableChannel (u1_t channel) {
    if( channel < 72+MAX_XCHANNELS )
        LMIC.region.channelMap[channel>>4] &= ~(1<<(channel&0xF));
}

#define MCMD_LADR_CHP_125ON     6  // All 125 kHz  ON: ChMask applies to channels 64 to 71
#define MCMD_LADR_CHP_125OFF    7  // All 125 kHz OFF: ChMask applies to channels 64 to 71

u1_t mapChannels(u1_t chpage, u2_t chmap)
{
    if (chpage == MCMD_LADR_CHP_125ON || chpage == MCMD_LADR_CHP_125OFF) {
        u2_t en125 = chpage == MCMD_LADR_CHP_125ON ? 0xFFFF : 0x0000;
        for (u1_t u=0; u<4; u++)
            LMIC.region.channelMap[u] = en125;
        LMIC.region.channelMap[64/16] = chmap;
    } else if (chpage == 5) {
        /* lower 8bits of chmap are FSB (sub-band) enable bits */
        for (unsigned fsb = 0; fsb < 8; fsb++) {
            if (chmap & 1<<fsb) {
                LMIC.region.channelMap[fsb >> 1] |= (fsb & 1) ? 0xff00 : 0x00ff;
                LMIC.region.channelMap[4] |= 1 << fsb;
            } else {
                LMIC.region.channelMap[fsb >> 1] &= (fsb & 1) ? 0x00ff : 0xff00;
                LMIC.region.channelMap[4] &= ~(1 << fsb);
            }
        }
    } else {
        if (chpage >= (72+MAX_XCHANNELS+15)/16)
            return 0;
        LMIC.region.channelMap[chpage] = chmap;
    }
    return 1;
}

void updateTx()
{
    u1_t chnl = LMIC.txChnl;
    if( chnl < 64 ) {
        LMIC.freq = US915_125kHz_UPFBASE + chnl*US915_125kHz_UPFSTEP;
        LMIC.txpow = 30;
        return;
    }
    LMIC.txpow = 26;
    if( chnl < 64+8 ) {
        LMIC.freq = US915_500kHz_UPFBASE + (chnl-64)*US915_500kHz_UPFSTEP;
    } else {
        MBED_ASSERT(chnl < 64+8+MAX_XCHANNELS);
        LMIC.freq = LMIC.region.xchFreq[chnl-72];
    }

}

void updatePostTx()
{
    // Update global duty cycle stats
    if (LMIC.globalDutyRate != 0) {
        us_timestamp_t airtime_us = LMIC.txend_us - LMIC.txstart_us;
        LMIC.globalDutyAvail_us = LMIC.txbeg_us + (airtime_us << LMIC.globalDutyRate);
        //printf("updateTx gda %llu\r\n", LMIC.globalDutyAvail_us);
    }
}

// US does not have duty cycling - return now as earliest TX time
//#define nextTx(now) (_nextTx(),(now))
us_timestamp_t nextTx(us_timestamp_t now_us)
{
    if (LMIC.region.chRnd == 0)
        LMIC.region.chRnd = os_getRndU1() & 0x3F;

    if (LMIC.datarate >= DR_SF8C) { // 500kHz
        u1_t map = LMIC.region.channelMap[64/16] & 0xFF;
        for( u1_t i=0; i<8; i++ ) {
            if( (map & (1<<(++LMIC.region.chRnd & 7))) != 0 ) {
                LMIC.txChnl = 64 + (LMIC.region.chRnd & 7);
                return now_us;
            }
        }
    } else { // 125kHz
        for( u1_t i=0; i<64; i++ ) {
            u1_t chnl = ++LMIC.region.chRnd & 0x3F;
            if( (LMIC.region.channelMap[(chnl >> 4)] & (1<<(chnl & 0xF))) != 0 ) {
                LMIC.txChnl = chnl;
                return now_us;
            }
        }
    }
    // No feasible channel  found! Keep old one.
    return now_us;
}

void setBcnRxParams (void) {
    LMIC.dataLen = 0;
    LMIC.freq = US915_500kHz_DNFBASE + LMIC.bcnChnl * US915_500kHz_DNFSTEP;
    LMIC.rps  = setIh(setNocrc(dndr2rps((dr_t)DR_BCN),1),LEN_BCN);
}

void setRx1Params()
{
    LMIC.freq = US915_500kHz_DNFBASE + (LMIC.txChnl & 0x7) * US915_500kHz_DNFSTEP;
    if( /* TX datarate */LMIC.dndr < DR_SF8C )
        LMIC.dndr += DR_SF10CR - DR_SF10;
    else if( LMIC.dndr == DR_SF8C )
        LMIC.dndr = DR_SF7CR;
    LMIC.rps = dndr2rps(LMIC.dndr);
}

void initJoinLoop (void)
{
    LMIC.region.chRnd = 0;
    //calling nextTx now --- LMIC.txChnl = 0;
    LMIC.adrTxPow = 20;
    MBED_ASSERT((LMIC.opmode & OP_NEXTCHNL)==0);
    setDrJoin(DRCHG_SET, DR_SF7);
    nextTx(0);  // pick correct channel for first attempt
    LMIC.txend_us = LowPowerTimeout_read_us(&LMIC.ticker);
    //printf("ijl txend %llu\r\n", LMIC.txend_us);
}

u1_t nextJoinState(void)
{
    us_timestamp_t deferment;
    // Try the following:
    //   SF7/8/9/10  on a random channel 0..63
    //   SF8C        on a random channel 64..71
    //
    //u1_t failed = 0;
    u1_t failed = 1;    // try only once

    if (LMIC.datarate != DR_SF8C) {
        setDrJoin(DRCHG_SET, DR_SF8C);
    } else {
        s1_t dr = DR_SF7 - ++LMIC.txCnt;
        if( dr < DR_SF10 ) {
            dr = DR_SF10;
            failed = 1; // All DR exhausted - signal failed
        }
        setDrJoin(DRCHG_SET, dr);
    }
    nextTx(0); // get txChnl appropriate for current datarate
    LMIC.opmode &= ~OP_NEXTCHNL;
    LMIC.txend_us = LowPowerTimeout_read_us(&LMIC.ticker);
    deferment = DNW2_SAFETY_ZONE_us;
    LMIC.txend_us += deferment;
    return failed;
}

const u1_t txdr_to_rx1dr[5][N_UP_DR] = {    // [Rx1DRoffset][txdr]
    /* DR0 */ { 10,  9,  8,  8},
    /* DR1 */ { 11, 10,  9,  8},
    /* DR2 */ { 12, 11, 10,  9},
    /* DR3 */ { 13, 12, 11, 10},
    /* DR4 */ { 13, 13, 12, 11}
};

const u1_t _DR2RPS_CRC[] = {
    ILLEGAL_RPS,
    MAKERPS(SF10, BW125, CR_4_5, 0, 0),
    MAKERPS(SF9 , BW125, CR_4_5, 0, 0),
    MAKERPS(SF8 , BW125, CR_4_5, 0, 0),
    MAKERPS(SF7 , BW125, CR_4_5, 0, 0),
    MAKERPS(SF8 , BW500, CR_4_5, 0, 0),
    ILLEGAL_RPS ,
    ILLEGAL_RPS ,
    ILLEGAL_RPS ,
    MAKERPS(SF12, BW500, CR_4_5, 0, 0),
    MAKERPS(SF11, BW500, CR_4_5, 0, 0),
    MAKERPS(SF10, BW500, CR_4_5, 0, 0),
    MAKERPS(SF9 , BW500, CR_4_5, 0, 0),
    MAKERPS(SF8 , BW500, CR_4_5, 0, 0),
    MAKERPS(SF7 , BW500, CR_4_5, 0, 0),
    ILLEGAL_RPS
};

//TODO #define maxFrameLen(dr) ((dr)<=DR_SF11CR ? maxFrameLens[(dr)] : 0xFF)
//TODO const u1_t maxFrameLens [] = { 24,66,142,255,255,255,255,255,  66,142 };

// Table below defines the size of one symbol as
//   symtime = 256us * 2^T(sf,bw)
// 256us is called one symunit. 
//                 SF:                                  
//      BW:      |__7___8___9__10__11__12
//      125kHz   |  2   3   4   5   6   7
//      250kHz   |  1   2   3   4   5   6
//      500kHz   |  0   1   2   3   4   5
//  
// Times for half symbol per DR
// Per DR table to minimize rounding errors
const unsigned DR2HSYM_us[] = {
    128<<5,  // DR_SF10     DR_SF12CR
    128<<4,  // DR_SF9      DR_SF11CR
    128<<3,  // DR_SF8      DR_SF10CR
    128<<2,  // DR_SF7      DR_SF9CR
    128<<1,  // DR_SF8C     DR_SF8CR
    128<<0,  // -------     DR_SF7CR
};

bool regionBeaconCrcCheck(xref2u1_t d)
{
    return os_rlsbf2(&d[OFF_BCN_CRC1]) == os_crc16(d,OFF_BCN_CRC1);
}

void regionProcessBeacon()
{
    LMIC.bcnChnl = (LMIC.bcnChnl+1) & 7;
}

bool regionIsFSK()
{
    return false;   /* no FSK in us915 */
}

#endif /* CFG_us915 */
