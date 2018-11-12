#if defined(CFG_sx126x_radio)

#include "lmic.h"
#include "hal_spi.h"
#include "radio_sx126x.h"

/* direct register access */
#define REG_ADDR_IRQ_STATUS            0x58a // 16bit
#define REG_ADDR_IRQ_MASK              0x58c // 16bit
#define REG_ADDR_MODCFG                0x680 // 8bit
#define REG_ADDR_BITRATE               0x6a1 // 24bit fsk
#define REG_ADDR_FREQDEV               0x6a4 // 18bit fsk
#define REG_ADDR_SHAPECFG              0x6a7 // 5bit
#define REG_ADDR_FSK_PKTCTRL0          0x6b3 // 8bit
#define REG_ADDR_FSK_PKTCTRL1          0x6b4 // 3bit
#define REG_ADDR_FSK_PREAMBLE_TXLEN    0x6b5 // 16bit
#define REG_ADDR_FSK_SYNC_LEN          0x6b7 // 7bit
#define REG_ADDR_FSK_PKTCTRL2          0x6ba // 8bit
#define REG_ADDR_FSK_PAYLOAD_LEN       0x6bb // 8bit
#define REG_ADDR_SYNCADDR              0x6c0 // 64bit fsk
#define REG_ADDR_NODEADDR              0x6cd // 8bit fsk
#define REG_ADDR_NODEADDRCOMP          0x6cf // 2bit fsk

#define REG_ADDR_LORA_TXPKTLEN         0x702 // 8bit
#define REG_ADDR_LORA_CONFIG0          0x703 // 8bit  bw/sf
#define REG_ADDR_LORA_CONFIG1          0x704 // 8bit  ppm_offset, fixlen, invertiq, cr
#define REG_ADDR_LORA_CONFIG2          0x705 // 8bit  crcType
#define REG_ADDR_LORA_IRQ_MASK         0x70a // 24bit
#define REG_ADDR_LORA_PREAMBLE_SYMBNB  0x73a // 16bit
#define REG_ADDR_LORA_SYNC             0x740 // config22, config23: frame sync peak position

#define REG_ADDR_DIGFECTL              0x804 // 6bits
#define REG_ADDR_BWSEL                 0x807 // 5bits
#define REG_ADDR_RANDOM                0x819 //        ro
#define REG_ADDR_RFFREQ                0x88b // 31bits
#define REG_ADDR_FREQ_OFFSET           0x88f // 19bits
#define REG_ADDR_ANACTRL6              0x8d7 // 6bits
#define REG_ADDR_ANACTRL7              0x8d8 // 6bits
#define REG_ADDR_ANACTRL15             0x8e1 // 7bits
#define REG_ADDR_OCP                   0x8e7
#define REG_ADDR_                      0x

/***************************************************************/

#define RX_TIMEOUT_SINGLE         0x000000  /* stop RX after first packet */
#define RX_TIMEOUT_CONTINUOUS     0xffffff  /* keep RXing */

#define PACKET_TYPE_GFSK    0
#define PACKET_TYPE_LORA    1

#define LORA_CR_4_5         1
#define LORA_CR_4_6         2
#define LORA_CR_4_7         3
#define LORA_CR_4_8         4

#define LORA_BW_7           0x00 // 7.81 kHz real
#define LORA_BW_10          0x08 // 10.42 kHz real
#define LORA_BW_15          0x01 // 15.63 kHz real
#define LORA_BW_20          0x09 // 20.83 kHz real
#define LORA_BW_31          0x02 // 31.25 kHz real
#define LORA_BW_41          0x0A // 41.67 kHz real
#define LORA_BW_62          0x03 // 62.50 kHz real
#define LORA_BW_125         0x04 // 125 kHz real
#define LORA_BW_250         0x05 // 250 kHz real
#define LORA_BW_500         0x06 // 500 kHz real 

#define SET_RAMP_10U        0x00
#define SET_RAMP_20U        0x01
#define SET_RAMP_40U        0x02
#define SET_RAMP_80U        0x03
#define SET_RAMP_200U       0x04
#define SET_RAMP_800U       0x05
#define SET_RAMP_1700U      0x06
#define SET_RAMP_3400U      0x07

/***************************************************************/
typedef enum {
    CHIPMODE_NONE = 0,
    CHIPMODE_RX,
    CHIPMODE_TX
} chipMote_e;

chipMote_e chipMode;
void (*chipModeChange)(void);

typedef enum {
    STBY_RC = 0,
    STBY_XOSC
} stby_t;

typedef union {
    struct {
        uint8_t rtcWakeup    : 1;    // 0
        uint8_t rfu          : 1;    // 1
        uint8_t warmStart    : 1;    // 2
    } bits;
    uint8_t octet;
} sleepConfig_t;

typedef union {
    struct {    // 
        uint16_t TxDone           : 1;    // 0
        uint16_t RxDone           : 1;    // 1
        uint16_t PreambleDetected : 1;    // 2
        uint16_t SyncWordValid    : 1;    // 3
        uint16_t HeaderValid      : 1;    // 4
        uint16_t HeaderErr        : 1;    // 5
        uint16_t CrCerr           : 1;    // 6
        uint16_t CadDone          : 1;    // 7
        uint16_t CadDetected      : 1;    // 8
        uint16_t Timeout          : 1;    // 9
        uint16_t res              : 6;    // 10,11,12,13,14,15
    } bits;
    uint16_t word;
} IrqFlags_t;

typedef union {
    struct {
        uint8_t PreambleLengthHi;   // param1
        uint8_t PreambleLengthLo;   // param2
        uint8_t HeaderType;         // param3
        uint8_t PayloadLength;      // param4
        uint8_t CRCType;            // param5
        uint8_t InvertIQ;           // param6
        uint8_t unused[2];
    } lora;
    struct {
        uint8_t PreambleLengthHi;       // param1
        uint8_t PreambleLengthLo;       // param2
        uint8_t PreambleDetectorLength; // param3
        uint8_t SyncWordLength;         // param4
        uint8_t AddrComp;               // param5
        uint8_t PacketType;             // param6
        uint8_t PayloadLength;          // param7
        uint8_t CRCType;                // param8
        uint8_t Whitening;              // param9
    } gfsk;
    uint8_t buf[8];
} PacketParams_t;

typedef union {
    struct {
        uint8_t spreadingFactor; // param1
        uint8_t bandwidth; // param2
        uint8_t codingRate; // param3
        uint8_t LowDatarateOptimize; // param4
    } lora;
    struct {
        uint8_t bitrateHi;  // param1
        uint8_t bitrateMid; // param2
        uint8_t bitrateLo;  // param3
        uint8_t PulseShape; // param4
        uint8_t bandwidth;  // param5
        uint8_t fdevHi;     // param6
        uint8_t fdevMid;    // param7
        uint8_t fdevLo;     // param8
    } gfsk;
    uint8_t buf[8];
} ModulationParams_t;

/***************************************************************/

static u1_t randbuf[16];

void radio_start_rx(unsigned timeout)
{
    uint8_t buf[8];

    buf[0] = timeout >> 16;
    buf[1] = timeout >> 8;
    buf[2] = timeout;
    radio_xfer(OPCODE_SET_RX, 3, 0, buf);

    chipMode = CHIPMODE_RX;
    if (chipModeChange)
        chipModeChange();

    hal_dbg_pin_rx(1);
}

uint32_t radio_readReg(uint16_t addr, uint8_t len)
{
    uint32_t ret = 0;
    unsigned i;

    uint8_t buf[7];
    buf[0] = addr >> 8;
    buf[1] = (uint8_t)addr;
    radio_xfer(OPCODE_READ_REGISTER, 2, 3+len, buf);
    for (i = 0; i < len; i++) {
        ret <<= 8;
        ret |= buf[i+3];
    }
    return ret;
}

void radio_setStandby(stby_t stby)
{
    uint8_t octet = stby;
    hal_dbg_pin_rx(0);
    radio_xfer(OPCODE_SET_STANDBY, 1, 0, &octet);

    chipMode = CHIPMODE_NONE;
    if (chipModeChange)
        chipModeChange();

    hal_pin_antsw(0);
}

void SX126x_setSleep(bool warmStart, bool rtcWakeup)
{
    sleepConfig_t sc;

    hal_dbg_pin_rx(0);
    sc.octet = 0;
    sc.bits.rtcWakeup = rtcWakeup;
    sc.bits.warmStart = warmStart;
    radio_xfer(OPCODE_SET_SLEEP, 1, 0, &sc.octet);

    chipMode = CHIPMODE_NONE;
    if (chipModeChange)
        chipModeChange();

    hal_pin_antsw(0);
}

void radio_writeReg(uint16_t addr, uint32_t data, uint8_t len)
{
    uint8_t buf[6];
    uint8_t n;
    buf[0] = addr >> 8;
    buf[1] = (uint8_t)addr;
    for (n = len; n > 0; n--) {
        buf[n+1] = (uint8_t)data;
        data >>= 8;
    }
    radio_xfer(OPCODE_WRITE_REGISTER, 2+len, 2+len, buf);
}

void radio_init ()
{
    //uint8_t st;
    u4_t* u4ptr = (u4_t*)randbuf;
    unsigned i;

    hal_sx126x_init();

    /*do {
        radio_xfer(OPCODE_GET_STATUS, 0, 1, &st);
        printf("status:%02x\r\n", st);
        wait_ms(50);
    } while (st == 0);*/

    core_util_critical_section_enter();

    radio_start_rx(RX_TIMEOUT_CONTINUOUS);

    for (i = 0; i < 4; i++) {
        unsigned n;
        n = radio_readReg(REG_ADDR_RANDOM, 4);

        //printf("random:%u\r\n", n);
        *u4ptr++ = n;
        for (n = 0; n < 0x100; n++)
            asm("nop");
    }
    randbuf[0] = 16; // set initial index 

    radio_setStandby(STBY_RC);  // STBY_XOSC

    {
        uint8_t type = PACKET_TYPE_LORA;
        radio_xfer(OPCODE_SET_PACKET_TYPE, 1, 0, &type);
    }

#if defined(PUBLIC_NETWORK)
    radio_writeReg(REG_ADDR_LORA_SYNC, 0x3444, 2);
#elif defined(PRIVATE_NETWORK)
    radio_writeReg(REG_ADDR_LORA_SYNC, 0x1424, 2);
#else
    #error public_or_private
#endif
    {
        uint8_t en = 1;
        radio_xfer(OPCODE_SET_DIO2_AS_RFSWITCH, 1, 0, &en);
    }

    core_util_critical_section_exit();

    //tx_test();
}

// return next random byte derived from seed buffer
// (buf[0] holds index of next byte to be returned)
u1_t radio_rand1 () {
    u1_t i = randbuf[0];
    MBED_ASSERT( i != 0 );
    if( i==16 ) {
        os_aes(AES_ENC, randbuf, 16); // encrypt seed with any key
        i = 0;
    }
    u1_t v = randbuf[i++];
    randbuf[0] = i;
    return v;
}

unsigned radio_get_symbol_period()
{
    unsigned bw_period;
    sf_t sf = getSf(LMIC.rps) + 6;

    switch (getBw(LMIC.rps)) {
    case BW125: bw_period = 8; break;
    case BW250: bw_period = 4; break;
    case BW500: bw_period = 2; break;
    default:
        MBED_ASSERT(0);
    }

    return (1 << sf) * bw_period;
}

static const u2_t LORA_RXDONE_FIXUP[] = {
    [FSK]  =     0,
    [SF7]  =     0,
    [SF8]  =  1648,
    [SF9]  =  3265,
    [SF10] =  7049,
    [SF11] = 13641,
    [SF12] = 31189,
};

void radio_irq_handler()
{
    us_timestamp_t now_us;
    IrqFlags_t irqFlags, clearIrqFlags;
    uint8_t buf[4];

    if (deep_wake) {
        now_us = deep_wake_at;
        deep_wake = false;
    } else
        now_us = LowPowerTimeout_read_us(&LMIC.ticker);


    radio_xfer(OPCODE_GET_IRQ_STATUS, 0, 3, buf);
    irqFlags.word = buf[1] << 8;
    irqFlags.word |= buf[2];
    clearIrqFlags.word = 0;
    if (irqFlags.bits.TxDone) {
        LMIC.txend_us = now_us - 43; // TXDONE FIXUP
        clearIrqFlags.bits.TxDone = 1;
        chipMode = CHIPMODE_NONE;
    }
    if (irqFlags.bits.RxDone) {
        int8_t s;
        if(getBw(LMIC.rps) == BW125) {
            now_us -= LORA_RXDONE_FIXUP[getSf(LMIC.rps)];
        }
        LMIC.rxtime_us = now_us;

        radio_xfer(OPCODE_GET_RX_BUFFER_STATUS, 0, 3, buf);

        LMIC.dataLen = buf[1];

        //readBuf(RegFifo, LMIC.frame, LMIC.dataLen);
        readBuf(LMIC.dataLen, buf[2], LMIC.frame);
        radio_xfer(OPCODE_GET_PACKET_STATUS, 0, 4, buf);
        s = buf[2];
        LMIC.snr  = s / 4.0;
        LMIC.rssi = -buf[1] / 2.0;   // TODO FSK
        clearIrqFlags.bits.RxDone = 1;
    }
    if (irqFlags.bits.Timeout) {
        LMIC.dataLen = 0;
        chipMode = CHIPMODE_NONE;
        hal_dbg_pin_rx(0);
        clearIrqFlags.bits.Timeout = 1;
    }

    if (clearIrqFlags.word != 0) {
        buf[0] = clearIrqFlags.word >> 8;
        buf[1] = (uint8_t)clearIrqFlags.word;
        radio_xfer(OPCODE_CLEAR_IRQ_STATUS, 2, 0, buf);
    }

    if (chipModeChange)
        chipModeChange();

    LMIC.radio_bottom_half = true;
} // ..radio_irq_handler()

#define MHZ_TO_FRF              1048576 // = (1<<25) / Fxtal_MHz
#define KHZ_TO_FRF              1048.576
#define HZ_TO_FRF              1.048576
uint8_t radio_setHz(unsigned Hz)
{
    unsigned frf = Hz * HZ_TO_FRF;
    uint8_t buf[4];

    buf[0] = frf >> 24;
    buf[1] = frf >> 16;
    buf[2] = frf >> 8;
    buf[3] = frf;
    radio_xfer(OPCODE_SET_RF_FREQUENCY, 4, 0, buf);
    return buf[3];
}

void startrx(bool single)
{
    //u1_t symbs;
    float sp;
    ModulationParams_t mp;
    uint8_t buf[2];
    radio_setStandby(STBY_RC);  // STBY_XOSC

/*    if (single) {
        symbs = 8;
        if (symbs * sp < RX_WINDOW_OPEN_MS) {
            symbs = RX_WINDOW_OPEN_MS / sp;
        }
    } else
        symbs = 0;*/

    if (single)
        radio_xfer(OPCODE_SET_LORA_SYMBOL_TIMEOUT, 1, 0, &LMIC.rxsyms);

    hal_pin_antsw(1);

    radio_setHz(LMIC.freq);

    buf[0] = 0; // TX base address
    buf[1] = 0; // RX base address
    radio_xfer(OPCODE_SET_BUFFER_BASE_ADDR, 2, 0, buf);

    {
        float khz;
#if defined(CFG_us915)
        mp.lora.spreadingFactor = getSf(LMIC.rps) + 6;
#else
#error implementRegion
#endif
        switch (getBw(LMIC.rps)) {
            case BW125: mp.lora.bandwidth = LORA_BW_125; khz = 125; break;
            case BW250: mp.lora.bandwidth = LORA_BW_250; khz = 250; break;
            case BW500: mp.lora.bandwidth = LORA_BW_500; khz = 500; break;
            default:
                MBED_ASSERT(0);
        }

        switch( getCr(LMIC.rps) ) {
            case CR_4_5: mp.lora.codingRate = LORA_CR_4_5; break;
            case CR_4_6: mp.lora.codingRate = LORA_CR_4_6; break;
            case CR_4_7: mp.lora.codingRate = LORA_CR_4_7; break;
            case CR_4_8: mp.lora.codingRate = LORA_CR_4_8; break;
            default:
                MBED_ASSERT(0);
        }

        sp = (1 << mp.lora.spreadingFactor) / khz;
        /* TCXO dependent */
        if (sp > 16)
            mp.lora.LowDatarateOptimize = 1; // param4
        else
            mp.lora.LowDatarateOptimize = 0; // param4

        radio_xfer(OPCODE_SET_MODULATION_PARAMS, 4, 0, mp.buf);
    }

    {
        PacketParams_t pp;

        pp.lora.PreambleLengthHi = 0;
        pp.lora.PreambleLengthLo = 8;
        pp.lora.HeaderType = getIh(LMIC.rps);

        pp.lora.CRCType = getNocrc(LMIC.rps) ? 0 : 1;
        pp.lora.InvertIQ = 1;
        pp.lora.PayloadLength = 255;

        radio_xfer(OPCODE_SET_PACKET_PARAMS, 6, 0, pp.buf);
    }

    {
        uint8_t buf[8];
        IrqFlags_t irqEnable;
        irqEnable.word = 0;
        irqEnable.bits.RxDone = 1;
        irqEnable.bits.Timeout = 1;

        buf[0] = irqEnable.word >> 8;    // enable bits
        buf[1] = irqEnable.word; // enable bits
        buf[2] = irqEnable.word >> 8;     // dio1
        buf[3] = irqEnable.word;  // dio1
        buf[4] = 0; // dio2
        buf[5] = 0; // dio2
        buf[6] = 0; // dio3
        buf[7] = 0; // dio3
        radio_xfer(OPCODE_SET_DIO_IRQ_PARAMS, 8, 0, buf);
    }
    /////////////////////////////

    radio_start_rx(single ? RX_TIMEOUT_SINGLE : RX_TIMEOUT_CONTINUOUS);

} // ..startrx()

void radio_set_tx_dbm(bool is1262, int8_t dbm)
{
    uint8_t buf[4];
    // use OCP default

    buf[3] = 1;
    if (is1262) {
        buf[0] = 4;
        buf[1] = 7;
        buf[2] = 0;

        if (dbm > 22)
            dbm = 22;
        else if (dbm < -3)
            dbm = -3;
    } else {
        if (dbm == 15)
            buf[0] = 6;
        else
            buf[0] = 4;
        buf[1] = 0;
        buf[2] = 1;

        if (dbm > 14)
            dbm = 14;
        else if (dbm < -3)
            dbm = -3;
    }
    radio_xfer(OPCODE_SET_PA_CONFIG, 4, 0, buf);

    if (is1262 && dbm > 18) {
        /* OCP is set by chip whenever SetPaConfig() is called */
        radio_writeReg(REG_ADDR_OCP, 0x38, 1);
    }

    // SetTxParams
    buf[0] = dbm;
    //if (opt == 0) txco
    buf[1] = SET_RAMP_200U;
    radio_xfer(OPCODE_SET_TX_PARAMS, 2, 0, buf);
}

static void txlora ()
{
    uint8_t buf[8];

    radio_setStandby(STBY_RC);  // STBY_XOSC

    // configure frequency
#ifdef FAIL_TX
    if (LMIC.datarate < 4)
        radio_setHz(LMIC.freq + 100000);
    else
        radio_setHz(LMIC.freq + 400000);
#else
    radio_setHz(LMIC.freq);
#endif

    radio_set_tx_dbm(true, LMIC.txpow);

    buf[0] = 0; // TX base address
    buf[1] = 0; // RX base address
    radio_xfer(OPCODE_SET_BUFFER_BASE_ADDR, 2, 0, buf);

    writeBuf(LMIC.dataLen, 0, LMIC.frame);

    {
        float sp, khz;
        ModulationParams_t mp;
#if defined(CFG_us915)
        mp.lora.spreadingFactor = getSf(LMIC.rps) + 6;
#else
#error implementRegion
#endif
        switch (getBw(LMIC.rps)) {
            case BW125: mp.lora.bandwidth = LORA_BW_125; khz = 125; break;
            case BW250: mp.lora.bandwidth = LORA_BW_250; khz = 250; break;
            case BW500: mp.lora.bandwidth = LORA_BW_500; khz = 500; break;
            default:
                MBED_ASSERT(0);
        }

        switch( getCr(LMIC.rps) ) {
            case CR_4_5: mp.lora.codingRate = LORA_CR_4_5; break;
            case CR_4_6: mp.lora.codingRate = LORA_CR_4_6; break;
            case CR_4_7: mp.lora.codingRate = LORA_CR_4_7; break;
            case CR_4_8: mp.lora.codingRate = LORA_CR_4_8; break;
            default:
                MBED_ASSERT(0);
        }

        sp = (1 << mp.lora.spreadingFactor) / khz;
        /* TCXO dependent */
        if (sp > 16)
            mp.lora.LowDatarateOptimize = 1; // param4
        else
            mp.lora.LowDatarateOptimize = 0; // param4

        radio_xfer(OPCODE_SET_MODULATION_PARAMS, 4, 0, mp.buf);
    }

    {
        PacketParams_t pp;

        pp.lora.PreambleLengthHi = 0;
        pp.lora.PreambleLengthLo = 8;
        pp.lora.HeaderType = getIh(LMIC.rps);

        pp.lora.CRCType = getNocrc(LMIC.rps) ? 0 : 1;
        pp.lora.InvertIQ = 0;
        pp.lora.PayloadLength = LMIC.dataLen;

        radio_xfer(OPCODE_SET_PACKET_PARAMS, 6, 0, pp.buf);
    }

    {
        IrqFlags_t irqEnable;
        irqEnable.word = 0;
        irqEnable.bits.TxDone = 1;
        irqEnable.bits.Timeout = 1;

        buf[0] = irqEnable.word >> 8;    // enable bits
        buf[1] = irqEnable.word; // enable bits
        buf[2] = irqEnable.word >> 8;     // dio1
        buf[3] = irqEnable.word;  // dio1
        buf[4] = 0; // dio2
        buf[5] = 0; // dio2
        buf[6] = 0; // dio3
        buf[7] = 0; // dio3
        radio_xfer(OPCODE_SET_DIO_IRQ_PARAMS, 8, 0, buf);
    }

    hal_pin_antsw(1);

    buf[0] = 0x40;
    buf[1] = 0x00;
    buf[2] = 0x00;
    radio_xfer(OPCODE_SET_TX, 3, 0, buf);

    chipMode = CHIPMODE_TX;
    if (chipModeChange)
        chipModeChange();

/*    for (unsigned i = 0; i < 10; i++) {
        uint8_t st;
        radio_xfer(OPCODE_GET_STATUS, 0, 1, &st);
        printf("status:%02x\r\n", st);
        //wait_ms(5);
    }*/
}


void os_radio (u1_t mode)
{
    core_util_critical_section_enter();

    switch (mode) {
        case RADIO_RST:
            // put radio to sleep
            SX126x_setSleep(true, false);
            break;

        case RADIO_TX:
            // transmit frame now
            //starttx(); // buf=LMIC.frame, len=LMIC.dataLen
            //if(getSf(LMIC.rps) == FSK) { // FSK modem
            //    txfsk();
            //} else { // LoRa modem
            txlora();
            //}
        break;

        case RADIO_RX:
            // receive frame now (exactly at rxtime)
            startrx(true);
            break;

        case RADIO_RXON:
            // start scanning for beacon now
            startrx(false);
            break;
    }

    core_util_critical_section_exit();
}



#endif /* CFG_sx126x_radio */
