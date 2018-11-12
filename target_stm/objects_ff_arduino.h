#ifdef TARGET_FF_ARDUINO
/*** pins present on all arduino form-factor stm32 boards ***/

#define RADIO_RESET_PIN     GPIO_PIN_0   // arduino A0 = PA0
#define RADIO_RESET_PORT    GPIOA
#define RADIO_RESET_PORT_ENABLE     __HAL_RCC_GPIOA_CLK_ENABLE()

#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)

    #define NSS_PORT                         GPIOB  // D10 = PB6
    #define NSS_PIN                          GPIO_PIN_6
    #define NSS_PORT_ENABLE                  __HAL_RCC_GPIOB_CLK_ENABLE()

    #define RADIO_RXTX_PIN      GPIO_PIN_1  // arduino A4 = PC1
    #define RADIO_RXTX_PORT     GPIOC
    #define RADIO_RXTX_PORT_ENABLE      __HAL_RCC_GPIOC_CLK_ENABLE()

#elif defined(CFG_sx126x_radio)

    #define NSS_PORT                GPIOA  // D7 = PA8
    #define NSS_PIN                 GPIO_PIN_8
    #define NSS_PORT_ENABLE         __HAL_RCC_GPIOA_CLK_ENABLE()

    #define TX_LED_PIN              GPIO_PIN_1  // A4 = PC1
    #define TX_LED_PORT             GPIOC

    #define RX_LED_PIN              GPIO_PIN_0  // A5 = PC0
    #define RX_LED_PORT             GPIOC

    #define RADIO_XTAL_SEL_PIN      GPIO_PIN_0    // A3 = PB0
    #define RADIO_XTAL_SEL_PORT     GPIOB

    #define RADIO_BUSY_PIN          GPIO_PIN_3 // D3
    #define RADIO_BUSY_PORT         GPIOB

    #define RADIO_ANTSW_PIN         GPIO_PIN_9  // D8
    #define RADIO_ANTSW_PORT        GPIOA

    #define RADIO_DEVICE_SEL_PIN    GPIO_PIN_4  // A2 = PA4
    #define RADIO_DEVICE_SEL_PORT   GPIOA

#endif /* ..radio */

#endif /* TARGET_FF_ARDUINO */
