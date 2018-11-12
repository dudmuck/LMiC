/* NRF52-DK PCA10040 */

#define SPI_MISO_PIN        24
#define SPI_MOSI_PIN        23
#define SPI_SCK_PIN         25

#define RX_SETUP_TIME_us   700   /* how long it takes to configure radio receiver */
#define MIN_RX_WIN_us       20000      // minimum preamble listening period considering clock error and timing jitter

#define D8_PIN              19
#define DBG_RX_PIN           5      // D17 p6-4

#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)

    #define SPI_SS_PIN          22      // D10
    #define RXTX_PIN            30      // A4

    #define PIN_DIO0_IN         13    // D2
    #define PIN_DIO1_IN         14    // D3

#elif defined(CFG_sx126x_radio)

    #define SPI_SS_PIN              18  // D7
    #define RADIO_BUSY_PIN          14  // D3
    #define RADIO_XTAL_SEL_PIN      29  // A3
    #define RADIO_DEVICE_SEL_PIN    28  // A2
    #define TX_LED_PIN              30  // A4
    #define RX_LED_PIN              31  // A5
    #define RADIO_ANTSW_PIN         19  // D8, LED3
    #define PIN_DIO1_IN             16  // D5

#endif

#define RADIO_RESET_PIN      3      // A0



