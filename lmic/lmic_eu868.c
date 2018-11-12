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
#if defined(CFG_eu868)
#include "lmic.h"

enum { NUM_DEFAULT_CHANNELS=6 };
static const u4_t iniChannelFreq[12] = {
    // Join frequencies and duty cycle limit (0.1%)
    EU868_F1|BAND_MILLI, EU868_J4|BAND_MILLI,
    EU868_F2|BAND_MILLI, EU868_J5|BAND_MILLI,
    EU868_F3|BAND_MILLI, EU868_J6|BAND_MILLI,
    // Default operational frequencies
    EU868_F1|BAND_CENTI, EU868_F2|BAND_CENTI, EU868_F3|BAND_CENTI,
    EU868_F4|BAND_MILLI, EU868_F5|BAND_MILLI, EU868_F6|BAND_DECI
};

void initDefaultChannels (bit_t join)
{
    os_clearMem(&LMIC.region.channelFreq, sizeof(LMIC.region.channelFreq));
    os_clearMem(&LMIC.region.channelDrMap, sizeof(LMIC.region.channelDrMap));
    os_clearMem(&LMIC.region.bands, sizeof(LMIC.region.bands));

    LMIC.region.channelMap = (1 << NUM_DEFAULT_CHANNELS) - 1;
    u1_t su = join ? 0 : NUM_DEFAULT_CHANNELS;
    for( u1_t fu=0; fu<NUM_DEFAULT_CHANNELS; fu++,su++ ) {
        LMIC.region.channelFreq[fu]  = iniChannelFreq[su];
        LMIC.region.channelDrMap[fu] = DR_RANGE_MAP(DR_SF12,DR_SF7);
    }
// IBM LMIC had:
//    if( !join ) {
//        LMIC.region.channelDrMap[5] = DR_RANGE_MAP(DR_SF12,DR_SF7);
//        LMIC.region.channelDrMap[1] = DR_RANGE_MAP(DR_SF12,DR_FSK);
//    }

    LMIC.region.bands[BAND_MILLI].txcap    = 1000;  // 0.1%
    LMIC.region.bands[BAND_MILLI].txpow    = 14;
    LMIC.region.bands[BAND_MILLI].lastchnl = os_getRndU1() % MAX_CHANNELS;
    LMIC.region.bands[BAND_CENTI].txcap    = 100;   // 1%
    LMIC.region.bands[BAND_CENTI].txpow    = 14;
    LMIC.region.bands[BAND_CENTI].lastchnl = os_getRndU1() % MAX_CHANNELS;
    LMIC.region.bands[BAND_DECI ].txcap    = 10;    // 10%
    LMIC.region.bands[BAND_DECI ].txpow    = 27;
    LMIC.region.bands[BAND_DECI ].lastchnl = os_getRndU1() % MAX_CHANNELS;
    LMIC.region.bands[BAND_MILLI].avail_us = 
    LMIC.region.bands[BAND_CENTI].avail_us =
    LMIC.region.bands[BAND_DECI ].avail_us = LowPowerTimeout_read_us(&LMIC.ticker);
}

bit_t LMIC_setupBand (u1_t bandidx, s1_t txpow, u2_t txcap) {
    if( bandidx > BAND_AUX ) return 0;
    //band_t* b = &LMIC.region.bands[bandidx];
    xref2band_t b = &LMIC.region.bands[bandidx];
    b->txpow = txpow;
    b->txcap = txcap;
    b->avail_us = LowPowerTimeout_read_us(&LMIC.ticker);
    b->lastchnl = os_getRndU1() % MAX_CHANNELS;
    return 1;
}

bit_t LMIC_setupChannel (u1_t chidx, u4_t freq, u2_t drmap, s1_t band) {
    if( chidx >= MAX_CHANNELS )
        return 0;
    if( band == -1 ) {
        if( freq >= 869400000 && freq <= 869650000 )
            freq |= BAND_DECI;   // 10% 27dBm
        else if( (freq >= 868000000 && freq <= 868600000) ||
                 (freq >= 869700000 && freq <= 870000000)  )
            freq |= BAND_CENTI;  // 1% 14dBm 
        else 
            freq |= BAND_MILLI;  // 0.1% 14dBm
    } else {
        if( band > BAND_AUX ) return 0;
        freq = (freq&~3) | band;
    }
    LMIC.region.channelFreq [chidx] = freq;
    LMIC.region.channelDrMap[chidx] = drmap==0 ? DR_RANGE_MAP(DR_SF12,DR_SF7) : drmap;
    LMIC.region.channelMap |= 1<<chidx;  // enabled right away
    return 1;
}

void LMIC_disableChannel (u1_t channel) {
    LMIC.region.channelFreq[channel] = 0;
    LMIC.region.channelDrMap[channel] = 0;
    LMIC.region.channelMap &= ~(1<<channel);
}

u4_t convFreq (xref2u1_t ptr) {
    u4_t freq = (os_rlsbf4(ptr-1) >> 8) * 100;
    if( freq < EU868_FREQ_MIN || freq > EU868_FREQ_MAX )
        freq = 0;
    return freq;
}

u1_t mapChannels (u1_t chpage, u2_t chmap)
{
    // Bad page, disable all channel, enable non-existent
    //if( chpage != 0 || chmap==0 || (chmap & ~LMIC.region.channelMap) != 0 )
    if (chpage != 0 && chpage != 6)
        return 0;  // illegal input
    for( u1_t chnl=0; chnl<MAX_CHANNELS; chnl++ ) {
        if (chpage == 0) {
            if( (chmap & (1<<chnl)) != 0 && LMIC.region.channelFreq[chnl] == 0 )
                chmap &= ~(1<<chnl); // ignore - channel is not defined
        } else if (chpage == 6) {
            /* enable all defined channels */
            if (LMIC.region.channelFreq[chnl] != 0 )
                chmap |= 1 << chnl;
            else
                chmap &= ~(1 << chnl);
        }
    }
    LMIC.region.channelMap = chmap;
    return 1;
}


void updateTx()
{
    u4_t freq = LMIC.region.channelFreq[LMIC.txChnl];
    // Update global/band specific duty cycle stats
    // Update channel/global duty cycle stats
    xref2band_t band = &LMIC.region.bands[freq & 0x3];
    LMIC.freq  = freq & ~(u4_t)3;
    LMIC.txpow = band->txpow;
}

void updatePostTx()
{
    u4_t freq = LMIC.region.channelFreq[LMIC.txChnl];
    xref2band_t band = &LMIC.region.bands[freq & 0x3];
    unsigned airtime_us = LMIC.txend_us - LMIC.txstart_us;
    band->avail_us = LMIC.txbeg_us + airtime_us * band->txcap;
    printf("updateTx %llu = %llu + %u * %u  ", band->avail_us, LMIC.txbeg_us, airtime_us, band->txcap);
    if (LMIC.globalDutyRate != 0) {
        LMIC.globalDutyAvail_us = LMIC.txbeg_us + (airtime_us << LMIC.globalDutyRate);
        printf(" gda:%llu ", LMIC.globalDutyAvail_us);
    }
}

us_timestamp_t nextTx (us_timestamp_t now_us)
{
    u1_t bmap=0xF;
    do {
        u1_t band=0;
        //us_timestamp_t mintime_us = now_us + /*10h*/(3600 * 10 * 1000000);
        us_timestamp_t mintime_us = /*10h*/10;
        mintime_us *= 3600; // sec per hour
        mintime_us *= 1000000; // us per sec
        mintime_us += now_us;
        for( u1_t bi=0; bi<4; bi++ ) {
            if( (bmap & (1<<bi)) && mintime_us - LMIC.region.bands[bi].avail_us > 0 )
                mintime_us = LMIC.region.bands[band = bi].avail_us;
        }
        // Find next channel in given band
        u1_t chnl = LMIC.region.bands[band].lastchnl;
        for( u1_t ci=0; ci<MAX_CHANNELS; ci++ ) {
            if( (chnl = (chnl+1)) >= MAX_CHANNELS )
                chnl -=  MAX_CHANNELS;
            if( (LMIC.region.channelMap & (1<<chnl)) != 0  &&  // channel enabled
                (LMIC.region.channelDrMap[chnl] & (1<<(LMIC.datarate&0xF))) != 0  &&
                band == (LMIC.region.channelFreq[chnl] & 0x3) ) { // in selected band
                LMIC.txChnl = LMIC.region.bands[band].lastchnl = chnl;
                return mintime_us;
            }
        }
        if( (bmap &= ~(1<<band)) == 0 ) {
            // No feasible channel  found!
            return mintime_us;
        }
    } while(1);
}


void setBcnRxParams (void) {
    LMIC.dataLen = 0;
    LMIC.freq = LMIC.region.channelFreq[LMIC.bcnChnl] & ~(u4_t)3;
    LMIC.rps  = setIh(setNocrc(dndr2rps((dr_t)DR_BCN),1),LEN_BCN);
}

void setRx1Params() { /*LMIC.freq/rps remain unchanged*/ }

void initJoinLoop (void) {
#if CFG_TxContinuousMode
  LMIC.txChnl = 0;
#else
    LMIC.txChnl = os_getRndU1() % NUM_DEFAULT_CHANNELS;
#endif
    LMIC.adrTxPow = 14;
    setDrJoin(DRCHG_SET, DR_SF7);
    initDefaultChannels(1);
    MBED_ASSERT((LMIC.opmode & OP_NEXTCHNL)==0);
    LMIC.txend_us = LMIC.region.bands[BAND_MILLI].avail_us + rndDelay(8);
    printf("ijl %llu\r\n", LMIC.txend_us);
}

u1_t nextJoinState (void)
{
    u1_t failed = 0;

    // Try 869.x and then 864.x with same DR
    // If both fail try next lower datarate
    if( ++LMIC.txChnl == NUM_DEFAULT_CHANNELS )
        LMIC.txChnl = 0;
    if( (++LMIC.txCnt & 1) == 0 ) {
        // Lower DR every 2nd try (having tried 868.x and 864.x with the same DR)
        if( LMIC.datarate == DR_SF12 )
            failed = 1; // we have tried all DR - signal EV_JOIN_FAILED
        else
            setDrJoin(DRCHG_NOJACC, decDR((dr_t)LMIC.datarate));
    }
    // Clear NEXTCHNL because join state engine controls channel hopping
    LMIC.opmode &= ~OP_NEXTCHNL;
    // Move txend to randomize synchronized concurrent joins.
    // Duty cycle is based on txend.
    us_timestamp_t time_us = LowPowerTimeout_read_us(&LMIC.ticker);
    if( time_us - LMIC.region.bands[BAND_MILLI].avail_us < 0 )
        time_us = LMIC.region.bands[BAND_MILLI].avail_us;
    LMIC.txend_us = time_us +
        (isTESTMODE()
         // Avoid collision with JOIN ACCEPT @ SF12 being sent by GW (but we missed it)
         ? DNW2_SAFETY_ZONE_us
         // Otherwise: randomize join (street lamp case):
         // SF12:255, SF11:127, .., SF7:8secs
         : DNW2_SAFETY_ZONE_us + rndDelay(255>>LMIC.datarate));
    // 1 - triggers EV_JOIN_FAILED event
    printf("njs txend %llu ", LMIC.txend_us);
    return failed;
}

const u1_t txdr_to_rx1dr[8][N_UP_DR] = {    // [Rx1DRoffset][txdr]
    /* DR0 */ { 0, 0, 0, 0, 0, 0},
    /* DR1 */ { 1, 0, 0, 0, 0, 0},
    /* DR2 */ { 2, 1, 0, 0, 0, 0},
    /* DR3 */ { 3, 2, 1, 0, 0, 0},
    /* DR4 */ { 4, 3, 2, 1, 0, 0},
    /* DR5 */ { 5, 4, 3, 2, 1, 0},
    /* DR6 */ { 6, 5, 4, 3, 2, 1},
    /* DR7 */ { 7, 6, 5, 4, 3, 2}
};

const u1_t _DR2RPS_CRC[] = {
    ILLEGAL_RPS,
    (u1_t)MAKERPS(SF12, BW125, CR_4_5, 0, 0),
    (u1_t)MAKERPS(SF11, BW125, CR_4_5, 0, 0),
    (u1_t)MAKERPS(SF10, BW125, CR_4_5, 0, 0),
    (u1_t)MAKERPS(SF9,  BW125, CR_4_5, 0, 0),
    (u1_t)MAKERPS(SF8,  BW125, CR_4_5, 0, 0),
    (u1_t)MAKERPS(SF7,  BW125, CR_4_5, 0, 0),
    (u1_t)MAKERPS(SF7,  BW250, CR_4_5, 0, 0),
    (u1_t)MAKERPS(FSK,  BW125, CR_4_5, 0, 0),
    ILLEGAL_RPS
};

const s1_t TXPOWLEVELS[] = {
    20, 14, 11, 8, 5, 2, 0,0, 0,0,0,0, 0,0,0,0
};

// TODO #define maxFrameLen(dr) ((dr)<=DR_SF9 ? maxFrameLens[(dr)] : 0xFF)
// TODO const u1_t maxFrameLens [] = { 64,64,64,123 };

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
    128<<7,  // DR_SF12
    128<<6,  // DR_SF11
    128<<5,  // DR_SF10
    128<<4,  // DR_SF9
    128<<3,  // DR_SF8
    128<<2,  // DR_SF7
    128<<1,  // DR_SF7B
    80  // FSK -- not used (time for 1/2 byte)
};

bool regionBeaconCrcCheck(xref2u1_t d)
{
    return d[OFF_BCN_CRC1] == (u1_t)os_crc16(d,OFF_BCN_CRC1);
}

void regionProcessBeacon(void) { }

bool regionIsFSK()
{
    return /* TX datarate */LMIC.rxsyms == DR_FSK;
}

#endif /* CFG_eu868 */
