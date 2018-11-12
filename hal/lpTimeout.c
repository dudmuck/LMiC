#include "lpTimeout.h"
#include "mbed_critical.h"
#include "power_mgmt.h"

static inline void init(ticker_t* ticker)
{
    if (!ticker->initialized) {
        ticker->_function = NULL;
        ticker->_lock_deepsleep = true;
        timer_event_initialize(&ticker->timer_event);

        ticker->initialized = true;
    }
}

void ticker_irq(uint32_t id)
{
    void (*local)(void);
    ticker_t* ticker = (ticker_t*)id;

    //for repeating ticker: timer_event_insert_absolute(&ticker->timer_event, ticker->timer_event.event.timestamp + ticker->_delay, id, ticker_irq);
    local = ticker->_function;
    LowPowerTimeout_detach(ticker); // this is timeout: one shot

    /*if (ticker->_function)
        ticker->_function();*/
    if (local)
        local();
}

void setup(ticker_t* ticker, us_timestamp_t t, bool absolute)
{
    us_timestamp_t target;
    core_util_critical_section_enter();
    timer_event_remove(&ticker->timer_event);
    ticker->_delay = t;
    target = ticker->_delay;
    if (!absolute)
        target += ticker_read_us(ticker->timer_event._ticker_data);

    timer_event_insert_absolute(
        &ticker->timer_event,
        target,
        (uint32_t)ticker,
        ticker_irq
    );
    core_util_critical_section_exit();
}

void LowPowerTimeout_attach_us(ticker_t* ticker, void (*func)(void), us_timestamp_t t)
{
    init(ticker);

    core_util_critical_section_enter();
    // lock only for the initial callback setup and this is not low power ticker
    if (!ticker->_function && ticker->_lock_deepsleep) {
        sleep_manager_lock_deep_sleep();
    }
    ticker->_function = func;
    setup(ticker, t, false);
    core_util_critical_section_exit();
}

void LowPowerTimeout_attach_us_absolute(ticker_t* ticker, void (*func)(void), us_timestamp_t t)
{
    init(ticker);

    core_util_critical_section_enter();
    // lock only for the initial callback setup and this is not low power ticker
    if (!ticker->_function && ticker->_lock_deepsleep) {
        sleep_manager_lock_deep_sleep();
    }
    ticker->_function = func;
    setup(ticker, t, true);
    core_util_critical_section_exit();
}

void LowPowerTimeout_detach(ticker_t* ticker)
{
    init(ticker);

    core_util_critical_section_enter();
    timer_event_remove(&ticker->timer_event);
    // unlocked only if we were attached (we locked it) and this is not low power ticker
    if (ticker->_function && ticker->_lock_deepsleep) {
        sleep_manager_unlock_deep_sleep();
    }

    ticker->_function = 0;
    core_util_critical_section_exit();
}

us_timestamp_t LowPowerTimeout_read_us(ticker_t* ticker)
{
    init(ticker);

    return ticker_read_us(ticker->timer_event._ticker_data);
}

void LowPowerTimeout_waitUntil(ticker_t* ticker, us_timestamp_t ts)
{
    while (ticker_read_us(ticker->timer_event._ticker_data) < ts)
        asm("nop");
}
