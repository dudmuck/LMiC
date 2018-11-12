#include "timer_event.h"

void timer_event_irq(uint32_t id)
{
    timer_event_t* te = (timer_event_t*)id;
    te->handler(te->id);
}

void timer_event_initialize(timer_event_t* te)
{
    te->_ticker_data = get_lp_ticker_data();
    ticker_set_handler(te->_ticker_data, timer_event_irq);
}

void timer_event_insert_absolute(timer_event_t* te, us_timestamp_t timestamp, uint32_t id, void (*handler)(uint32_t))
{
    ticker_insert_event_us(te->_ticker_data, &te->event, timestamp, (uint32_t)te);
    te->id = id;
    te->handler = handler;
}

void timer_event_remove(timer_event_t* te)
{
    ticker_remove_event(te->_ticker_data, &te->event);
}
