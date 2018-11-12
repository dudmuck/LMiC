enum { MAX_XCHANNELS = 2 };      // extra channels in RAM, channels 0-71 are immutable 

typedef struct {
    u4_t        xchFreq[MAX_XCHANNELS];    // extra channel frequencies (if device is behind a repeater)
    u2_t        xchDrMap[MAX_XCHANNELS];   // extra channel datarate ranges  ---XXX: ditto
    u2_t        channelMap[(72+MAX_XCHANNELS+15)/16];  // enabled bits
    u2_t        chRnd;        // channel randomizer
} region_t;

enum { DR_PAGE_US915 = 0x10 };

enum _dr_us915_t { DR_SF10=0, DR_SF9, DR_SF8, DR_SF7, DR_SF8C, DR_NONE,
                   // Devices behind a router:
                   DR_SF12CR=8, DR_SF11CR, DR_SF10CR, DR_SF9CR, DR_SF8CR, DR_SF7CR };
enum { DR_DFLTMIN = DR_SF8C };
enum { DR_PAGE = DR_PAGE_US915 };

// Default frequency plan for US 915MHz
enum { US915_125kHz_UPFBASE = 902300000,
       US915_125kHz_UPFSTEP =    200000,
       US915_500kHz_UPFBASE = 903000000,
       US915_500kHz_UPFSTEP =   1600000,
       US915_500kHz_DNFBASE = 923300000,
       US915_500kHz_DNFSTEP =    600000
};
enum { US915_FREQ_MIN = 902000000,
       US915_FREQ_MAX = 928000000 };

enum { CHNL_PING         = 0 }; // used only for default init of state (follows beacon - rotating)
enum { FREQ_PING         = US915_500kHz_DNFBASE + CHNL_PING*US915_500kHz_DNFSTEP };  // default ping freq
enum { DR_PING           = DR_SF10CR };       // default ping DR
enum { CHNL_DNW2         = 0 };
enum { FREQ_DNW2         = US915_500kHz_DNFBASE + CHNL_DNW2*US915_500kHz_DNFSTEP };
enum { DR_DNW2           = DR_SF12CR };
enum { CHNL_BCN          = 0 }; // used only for default init of state (rotating beacon scheme)
enum { DR_BCN            = DR_SF10CR };
enum { AIRTIME_BCN       = 72192 };  // micros

enum {
    // Beacon frame format US SF10
    OFF_BCN_NETID    = 0,         
    OFF_BCN_TIME     = 3,
    OFF_BCN_CRC1     = 7,
    OFF_BCN_INFO     = 9,
    OFF_BCN_LAT      = 10,
    OFF_BCN_LON      = 13,
    OFF_BCN_RFU1     = 16,
    OFF_BCN_CRC2     = 17,
    LEN_BCN          = 19
};

enum { MAX_TXPOW_125kHz = 30 };

#define pow2dBm(p) ((s1_t)(30 - (((p)&MCMD_LADR_POW_MASK)<<1)))

#define dr2hsym(dr) (DR2HSYM_us[(dr)&7])  // map DR_SFnCR -> 0-6
extern const unsigned DR2HSYM_us[];

#define DNW2_SAFETY_ZONE_us       750000

#define N_UP_DR     4

