#include "hal.h"
#include "hal_spi.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include <string.h>

#define _LED_ON(pin)    (NRF_GPIO->OUTSET = (1 << pin))
#define _LED_OFF(pin)    (NRF_GPIO->OUTCLR = (1 << pin))

void getHWDevEui(u1_t* devEui)
{
    void* ptr;

    ptr = (void*)&NRF_FICR->DEVICEID[1];
    memcpy(devEui, ptr, 4);

    devEui += 4;
    ptr = (void*)&NRF_FICR->DEVICEID[0];
    memcpy(devEui, ptr, 4);
}

#if defined(CFG_sx126x_radio)
void hal_pin_antsw (u1_t val)
{
    if (val)
        NRF_GPIO->OUTSET = (1 << RADIO_ANTSW_PIN);
    else
        NRF_GPIO->OUTCLR = (1 << RADIO_ANTSW_PIN);
}

#endif /* CFG_sx126x_radio */

void hal_pin_rst(u1_t val)
{
    if(val == 0 || val == 1) { // drive pin
        nrf_gpio_cfg_output(RADIO_RESET_PIN);
        if (val)
            NRF_GPIO->OUTSET = (1 << RADIO_RESET_PIN);
        else
            NRF_GPIO->OUTCLR = (1 << RADIO_RESET_PIN);
    } else { // keep pin floating
        nrf_gpio_cfg_input(RADIO_RESET_PIN, NRF_GPIO_PIN_NOPULL);
    }
}

void hal_dbg_pin_latency(bool hi)
{
    if (hi)
        NRF_GPIO->OUTSET = (1 << D8_PIN);
    else
        NRF_GPIO->OUTCLR = (1 << D8_PIN);
}

void hal_dbg_pin_rx(bool hi)
{
    if (hi)
        NRF_GPIO->OUTSET = (1 << DBG_RX_PIN);
    else
        NRF_GPIO->OUTCLR = (1 << DBG_RX_PIN);
}

/*
void test_foo()
{
    nrf_gpio_cfg_output(0);
    nrf_gpio_cfg_output(1);
    nrf_gpio_cfg_output(21);
    nrf_gpio_cfg_output(5);
    nrf_gpio_cfg_output(6);
    nrf_gpio_cfg_output(7);
    nrf_gpio_cfg_output(8);
    nrf_gpio_cfg_output(9);
    nrf_gpio_cfg_output(10);

    unsigned i;
    for (;;) {
        NRF_GPIO->OUTSET = (1 << 0);
        NRF_GPIO->OUTSET = (1 << 1);
        NRF_GPIO->OUTSET = (1 << 21);
        NRF_GPIO->OUTSET = (1 << 5);
        NRF_GPIO->OUTSET = (1 << 6);
        NRF_GPIO->OUTSET = (1 << 7);
        NRF_GPIO->OUTSET = (1 << 8);
        NRF_GPIO->OUTSET = (1 << 9);
        NRF_GPIO->OUTSET = (1 << 10);
        //hal_dbg_pin_latency(true);
        //hal_dbg_pin_rx(true);
        for (i = 0; i < 1000; i++)
            asm("nop");

        NRF_GPIO->OUTCLR = (1 << 0);
        NRF_GPIO->OUTCLR = (1 << 1);
        NRF_GPIO->OUTCLR = (1 << 21);
        NRF_GPIO->OUTCLR = (1 << 5);
        NRF_GPIO->OUTCLR = (1 << 6);
        NRF_GPIO->OUTCLR = (1 << 7);
        NRF_GPIO->OUTCLR = (1 << 8);
        NRF_GPIO->OUTCLR = (1 << 9);
        NRF_GPIO->OUTCLR = (1 << 10);
        //hal_dbg_pin_latency(false);
        //hal_dbg_pin_rx(false);
        for (i = 0; i < 1000; i++)
            asm("nop");
    }
}
*/

static void dio_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)
    radio_irq_handler(pin == PIN_DIO0_IN ? 0 : 1);
#elif defined(CFG_sx126x_radio)
    radio_irq_handler();
#endif
}

#if defined(CFG_sx126x_radio)
void hal_sx126x_init(void)
{
    if ((NRF_GPIO->IN >> RADIO_BUSY_PIN) & 0x1UL) {
        nrf_gpio_cfg_output(RADIO_RESET_PIN);
        NRF_GPIO->OUTCLR = (1 << RADIO_RESET_PIN);
        wait_ms(20);
        nrf_gpio_cfg_input(RADIO_RESET_PIN, NRF_GPIO_PIN_NOPULL);
        while ((NRF_GPIO->IN >> RADIO_BUSY_PIN) & 0x1UL)
            ;

        wait_ms(10);

        while ((NRF_GPIO->IN >> RADIO_BUSY_PIN) & 0x1UL)
            ;
    }
}
#endif /* CFG_sx126x_radio */

void hal_io_init()
{   
    ret_code_t err_code;

    nrf_gpio_cfg_input(RADIO_RESET_PIN, NRF_GPIO_PIN_NOPULL);

#if defined(CFG_sx126x_radio)
    #ifdef TARGET_FF_ARDUINO
        nrf_gpio_cfg_input(RADIO_BUSY_PIN, NRF_GPIO_PIN_NOPULL);
        nrf_gpio_cfg_input(RADIO_XTAL_SEL_PIN, NRF_GPIO_PIN_NOPULL);
        nrf_gpio_cfg_input(RADIO_DEVICE_SEL_PIN, NRF_GPIO_PIN_NOPULL);

        nrf_gpio_cfg_output(TX_LED_PIN);
        nrf_gpio_cfg_output(RX_LED_PIN);
        nrf_gpio_cfg_output(RADIO_ANTSW_PIN);
    #else
        #error implement_non_arduino_target
    #endif
#endif

    nrf_gpio_cfg_output(D8_PIN);
    nrf_gpio_cfg_output(DBG_RX_PIN);

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    in_config.pull = NRF_GPIO_PIN_NOPULL;

#if defined(CFG_sx1276_radio) || defined(CFG_sx1272_radio)
    err_code = nrf_drv_gpiote_in_init(PIN_DIO0_IN, &in_config, dio_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(PIN_DIO0_IN, true);
#endif /* CFG_sx1276_radio || CFG_sx1272_radio */

    err_code = nrf_drv_gpiote_in_init(PIN_DIO1_IN, &in_config, dio_handler);
    APP_ERROR_CHECK(err_code);

    //dio1_callback = dioIrq;
    nrf_drv_gpiote_in_event_enable(PIN_DIO1_IN, true);

    //test_foo();
}

void mbed_sdk_init(void);

void _hal_init()
{
    mbed_sdk_init();

    hal_io_init();

    hal_spi_init();

}

