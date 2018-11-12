
enum { MAX_BANDS    =  4 };
enum { MAX_CHANNELS = 16 };      //!< Max supported channels

//! \internal
struct band_t {
    u2_t     txcap;     // duty cycle limitation: 1/txcap
    s1_t     txpow;     // maximum TX power
    u1_t     lastchnl;  // last used channel
    us_timestamp_t  avail_us;     // channel is blocked until this time
};
TYPEDEF_xref2band_t; //!< \internal

typedef struct {
    band_t      bands[MAX_BANDS];
    u4_t        channelFreq[MAX_CHANNELS];
    u2_t        channelDrMap[MAX_CHANNELS];
    u2_t        channelMap;
} region_t;

enum { DR_PAGE_EU868 = 0x00 };

enum _dr_eu868_t { DR_SF12=0, DR_SF11, DR_SF10, DR_SF9, DR_SF8, DR_SF7, DR_SF7B, DR_FSK, DR_NONE };
enum { DR_DFLTMIN = DR_SF7 };
enum { DR_PAGE = DR_PAGE_EU868 };

// Default frequency plan for EU 868MHz ISM band
// Bands:
//  g1 :   1%  14dBm  
//  g2 : 0.1%  14dBm  
//  g3 :  10%  27dBm  
//                 freq             band     datarates
enum { EU868_F1 = 868100000,      // g1   SF7-12 
       EU868_F2 = 868300000,      // g1   SF7-12 FSK SF7/250         
       EU868_F3 = 868500000,      // g1   SF7-12         
       EU868_F4 = 868850000,      // g2   SF7-12         
       EU868_F5 = 869050000,      // g2   SF7-12         
       EU868_F6 = 869525000,      // g3   SF7-12         
       EU868_J4 = 864100000,      // g2   SF7-12  used during join
       EU868_J5 = 864300000,      // g2   SF7-12   ditto
       EU868_J6 = 864500000,      // g2   SF7-12   ditto
};
enum { EU868_FREQ_MIN = 863000000,
       EU868_FREQ_MAX = 870000000 };

enum { CHNL_PING         = 5 };
enum { FREQ_PING         = EU868_F6 };  // default ping freq
enum { DR_PING           = DR_SF9 };    // default ping DR
enum { CHNL_DNW2         = 5 };
enum { FREQ_DNW2         = EU868_F6 };
enum { DR_DNW2           = DR_SF12 };
enum { CHNL_BCN          = 5 };
enum { FREQ_BCN          = EU868_F6 };
enum { DR_BCN            = DR_SF9 };
enum { AIRTIME_BCN       = 144384 };  // micros

enum {
    // Beacon frame format EU SF9
    OFF_BCN_NETID    = 0,         
    OFF_BCN_TIME     = 3,
    OFF_BCN_CRC1     = 7,
    OFF_BCN_INFO     = 8,
    OFF_BCN_LAT      = 9,
    OFF_BCN_LON      = 12,
    OFF_BCN_CRC2     = 15,
    LEN_BCN          = 17
};


enum { LIMIT_CHANNELS = (1<<4) };   // EU868 will never have more channels

enum { BAND_MILLI=0, BAND_CENTI=1, BAND_DECI=2, BAND_AUX=3 };
bit_t LMIC_setupBand (u1_t bandidx, s1_t txpow, u2_t txcap);

#define pow2dBm(p) (TXPOWLEVELS[(p&MCMD_LADR_POW_MASK)>>MCMD_LADR_POW_SHIFT])
extern const s1_t TXPOWLEVELS[];

#define dr2hsym(dr) (DR2HSYM_us[(dr)])
extern const unsigned DR2HSYM_us[];

#define DNW2_SAFETY_ZONE_us       3000000

#define N_UP_DR     6

