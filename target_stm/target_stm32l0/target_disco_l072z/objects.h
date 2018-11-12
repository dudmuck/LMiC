/* Definition for USARTx clock resources */
#define USARTx                           USART2
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_2
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF4_USART2
#define USARTx_RX_PIN                    GPIO_PIN_3
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF4_USART2

#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

/* Definition for SPIx clock resources */
#define SPIx                             SPI1
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOB_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define SPIx_FORCE_RESET()               __HAL_RCC_SPI1_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI1_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_3
#define SPIx_SCK_GPIO_PORT               GPIOB
#define SPIx_SCK_AF                      GPIO_AF0_SPI1
#define SPIx_MISO_PIN                    GPIO_PIN_6
#define SPIx_MISO_GPIO_PORT              GPIOA
#define SPIx_MISO_AF                     GPIO_AF0_SPI1
#define SPIx_MOSI_PIN                    GPIO_PIN_7
#define SPIx_MOSI_GPIO_PORT              GPIOA
#define SPIx_MOSI_AF                     GPIO_AF0_SPI1

/* how fast SCK rate */
#define SPI_BAUDRATEPRESCALER            SPI_BAUDRATEPRESCALER_64
#define RX_SETUP_TIME_us                 2300   /* how long to configure radio receiver */

#define MIN_RX_WIN_us       10000      // minimum preamble listening period considering clock error and timing jitter

#define LATE_PORT     GPIOA
#define LATE_PIN      GPIO_PIN_11
#define LATE_PORT_ENABLE      __HAL_RCC_GPIOA_CLK_ENABLE()

#define RXDBG_PORT     GPIOA
#define RXDBG_PIN      GPIO_PIN_12
#define RXDBG_PORT_ENABLE      __HAL_RCC_GPIOA_CLK_ENABLE()

#define RADIO_RESET_PIN     GPIO_PIN_0   // PC0
#define RADIO_RESET_PORT    GPIOC
#define RADIO_RESET_PORT_ENABLE     __HAL_RCC_GPIOC_CLK_ENABLE()

#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)

    #define NSS_PORT                         GPIOA  // PA15
    #define NSS_PIN                          GPIO_PIN_15
    #define NSS_PORT_ENABLE                  __HAL_RCC_GPIOA_CLK_ENABLE()

    #define DIO0_PORT_ENABLE        __HAL_RCC_GPIOB_CLK_ENABLE()
    #define DIO0_PORT               GPIOB   // PB4
    #define DIO0_PIN                GPIO_PIN_4
    #define DIO0_IRQn               EXTI4_15_IRQn
    #define DIO0_IRQHandler         EXTI4_15_IRQHandler

    #define DIO1_PORT_ENABLE        __HAL_RCC_GPIOB_CLK_ENABLE()
    #define DIO1_PORT               GPIOB   // PB1
    #define DIO1_PIN                GPIO_PIN_1
    #define DIO1_IRQn               EXTI0_1_IRQn
    #define DIO1_IRQHandler         EXTI0_1_IRQHandler

#elif defined(CFG_sx126x_radio)
#endif /* ..radio */
