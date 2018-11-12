/******** NUCLEO_L152RE ********************/

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
#define USARTx_TX_AF                     GPIO_AF7_USART2
#define USARTx_RX_PIN                    GPIO_PIN_3
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF7_USART2

#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

/*********************************************************************/

/* Definition for SPIx clock resources */
#define SPIx                             SPI1
#define SPIx_CLK_ENABLE()                __HAL_RCC_SPI1_CLK_ENABLE()
#define SPIx_SCK_GPIO_CLK_ENABLE()       __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MISO_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define SPIx_MOSI_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define SPIx_FORCE_RESET()               __HAL_RCC_SPI1_FORCE_RESET()
#define SPIx_RELEASE_RESET()             __HAL_RCC_SPI1_RELEASE_RESET()

/* Definition for SPIx Pins */
#define SPIx_SCK_PIN                     GPIO_PIN_5
#define SPIx_SCK_GPIO_PORT               GPIOA
#define SPIx_SCK_AF                      GPIO_AF5_SPI1
#define SPIx_MISO_PIN                    GPIO_PIN_6
#define SPIx_MISO_GPIO_PORT              GPIOA
#define SPIx_MISO_AF                     GPIO_AF5_SPI1
#define SPIx_MOSI_PIN                    GPIO_PIN_7
#define SPIx_MOSI_GPIO_PORT              GPIOA
#define SPIx_MOSI_AF                     GPIO_AF5_SPI1

/* how fast SCK rate */
#define SPI_BAUDRATEPRESCALER            SPI_BAUDRATEPRESCALER_64
#define RX_SETUP_TIME_us                 2300   /* how long to configure radio receiver */

#define MIN_RX_WIN_us       10000      // minimum preamble listening period considering clock error and timing jitter


/*********************************************************************/

#define RXDBG_PORT     GPIOC
#define RXDBG_PIN      GPIO_PIN_3
#define RXDBG_PORT_ENABLE      __HAL_RCC_GPIOC_CLK_ENABLE()

#define LATE_PORT     GPIOC
#define LATE_PIN      GPIO_PIN_2
#define LATE_PORT_ENABLE      __HAL_RCC_GPIOC_CLK_ENABLE()

#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)

    #define DIO0_PORT_ENABLE        __HAL_RCC_GPIOA_CLK_ENABLE()
    #define DIO0_PORT               GPIOA   // D2 = PA10
    #define DIO0_PIN                GPIO_PIN_10
    #define DIO0_IRQn               EXTI15_10_IRQn
    #define DIO0_IRQHandler         EXTI15_10_IRQHandler

    #define DIO1_PORT_ENABLE        __HAL_RCC_GPIOB_CLK_ENABLE()
    #define DIO1_PORT               GPIOB   // D3 = PB3
    #define DIO1_PIN                GPIO_PIN_3
    #define DIO1_IRQn               EXTI3_IRQn
    #define DIO1_IRQHandler         EXTI3_IRQHandler

#elif defined(CFG_sx126x_radio)

    #define DIO1_PORT_ENABLE        __HAL_RCC_GPIOB_CLK_ENABLE()
    #define DIO1_PORT               GPIOB   // D5 = PB4
    #define DIO1_PIN                GPIO_PIN_4
    #define DIO1_IRQn               EXTI4_IRQn
    #define DIO1_IRQHandler         EXTI4_IRQHandler

#endif /* ..radio */

