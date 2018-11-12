#include "hal.h"

#define CRF1_PORT_ENABLE      __HAL_RCC_GPIOA_CLK_ENABLE()
#define CRF2_PORT_ENABLE      __HAL_RCC_GPIOC_CLK_ENABLE()
#define CRF3_PORT_ENABLE      __HAL_RCC_GPIOC_CLK_ENABLE()

#define CRF1_PIN        GPIO_PIN_1
#define CRF1_PORT       GPIOA

#define CRF2_PIN        GPIO_PIN_2
#define CRF2_PORT       GPIOC

#define CRF3_PIN        GPIO_PIN_1
#define CRF3_PORT       GPIOC

void hal_board_io_init()
{
    GPIO_InitTypeDef GPIO_InitStruct;

    CRF1_PORT_ENABLE;
    CRF2_PORT_ENABLE;
    CRF3_PORT_ENABLE;

    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = CRF1_PIN;
    HAL_GPIO_Init(CRF1_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CRF2_PIN;
    HAL_GPIO_Init(CRF2_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CRF3_PIN;
    HAL_GPIO_Init(CRF3_PORT, &GPIO_InitStruct);

    txpin = TX_PIN_BOTH;
}

void hal_pin_antsw(s1_t val)
{
    switch (val) {
        case -1:    // sleep
            HAL_GPIO_WritePin(CRF1_PORT, CRF1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CRF2_PORT, CRF2_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CRF3_PORT, CRF3_PIN, GPIO_PIN_RESET);
            break;
        case 0:     // rx
            HAL_GPIO_WritePin(CRF1_PORT, CRF1_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(CRF2_PORT, CRF2_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CRF3_PORT, CRF3_PIN, GPIO_PIN_RESET);
            break;
        case 1:     // tx RFO
            HAL_GPIO_WritePin(CRF1_PORT, CRF1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CRF2_PORT, CRF2_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(CRF3_PORT, CRF3_PIN, GPIO_PIN_RESET);
            break;
        case 2:     // tx PABOOST
            HAL_GPIO_WritePin(CRF1_PORT, CRF1_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CRF2_PORT, CRF2_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(CRF3_PORT, CRF3_PIN, GPIO_PIN_SET);
            break;
    }
}
