
#include "lp_ticker_api.h"
#include "nrf_drv_rtc.h"
//#include "nrfx_rtc.h"
#include "nrf_drv_clock.h"

#include "mbed_critical.h"

#define PRESCALE        1

static const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2); /**< Declaring an instance of nrf_drv_rtc for RTC2. */

#define RTC_COUNTER_BITS        24u
#define RTC_FREQ                32768u

/* LP ticker is driven by 32kHz clock and counter length is 24 bits. */
const ticker_info_t* lp_ticker_get_info()
{
    static const ticker_info_t info = {
        RTC_FREQ,
        RTC_COUNTER_BITS
    };
    return &info;
}

//volatile uint32_t Upper;

static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (int_type == NRFX_RTC_INT_COMPARE0) {
        lp_ticker_irq_handler();
    } /*else if (int_type == NRFX_RTC_INT_OVERFLOW) {
        Upper++;
    }*/
/*    if (int_type == NRFX_RTC_INT_COMPARE1)
    if (int_type == NRFX_RTC_INT_COMPARE2)
    if (int_type == NRFX_RTC_INT_COMPARE3)
    if (int_type == NRFX_RTC_INT_TICK)
*/
}

static void lfclk_config(void)
{   
/*  already done
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
*/

    nrf_drv_clock_lfclk_request(NULL);
}

void lp_ticker_init(void)
{
    uint32_t err_code;
    lfclk_config();

    //Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = PRESCALE - 1;
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //nrfx_rtc_overflow_enable(&rtc, true);

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);
}

uint32_t lp_ticker_read()
{
    return nrfx_rtc_counter_get(&rtc);
}

void lp_ticker_free()
{
    nrf_drv_rtc_disable(&rtc);
    nrf_drv_rtc_uninit(&rtc);
}

unsigned cc_channel = 0;

void lp_ticker_disable_interrupt(void)
{
    uint32_t int_mask = RTC_CHANNEL_INT_MASK(cc_channel);
    nrf_rtc_event_disable(rtc.p_reg, int_mask);
    nrf_rtc_int_disable(rtc.p_reg, int_mask);
}

void lp_ticker_clear_interrupt(void)
{
    nrf_rtc_event_t event    = RTC_CHANNEL_EVENT_ADDR(cc_channel);
    nrf_rtc_event_clear(rtc.p_reg, event);
}

void lp_ticker_fire_interrupt(void)
{
    uint32_t err_code;
/*    core_util_critical_section_enter();

    lp_ticker_interrupt_fire = true;

    NVIC_SetPendingIRQ(RTC2_IRQn);

    core_util_critical_section_exit();*/
    err_code = nrf_drv_rtc_cc_set(&rtc, cc_channel, nrfx_rtc_counter_get(&rtc) + 5, true);
    APP_ERROR_CHECK(err_code);
}

void lp_ticker_set_interrupt(timestamp_t ticks_count)
{
    uint32_t int_mask = RTC_CHANNEL_INT_MASK(cc_channel);
/*    common_rtc_set_interrupt(timestamp,
        LP_TICKER_CC_CHANNEL, LP_TICKER_INT_MASK);*/
    /* Set ticks value when interrupt should be fired.
     * Interrupt scheduling is performed in upper layers. */

    core_util_critical_section_enter();

    /* Wrap ticks_count before comparisons. */
    ticks_count = RTC_WRAP(ticks_count);

    /* COMPARE occurs when a CC register is N and the COUNTER value transitions from N-1 to N.
     * If the COUNTER is N, writing N+2 to a CC register is guaranteed to trigger a
     * COMPARE event at N+2.
     */
    const uint32_t now = nrf_rtc_counter_get(rtc.p_reg);

    if (now == ticks_count ||
        RTC_WRAP(now + 1) == ticks_count) {
        ticks_count = RTC_WRAP(ticks_count + 2);
    }

    nrf_rtc_cc_set(rtc.p_reg, cc_channel, ticks_count);

    if (!nrf_rtc_int_is_enabled(rtc.p_reg, int_mask)) {
        nrf_rtc_event_t event    = RTC_CHANNEL_EVENT_ADDR(cc_channel);
        nrf_rtc_event_clear(rtc.p_reg, event);
        nrf_rtc_int_enable(rtc.p_reg, int_mask);
    }

    core_util_critical_section_exit();
}
