#include "timer_event.h"

typedef struct {
    bool initialized;
    us_timestamp_t         _delay;  /**< Time delay (in microseconds) for re-setting the multi-shot callback. */
    void (*_function)(void);  /**< Callback. */
    bool          _lock_deepsleep;  /**< Flag which indicates if deep-sleep should be disabled. */

    timer_event_t     timer_event;
} ticker_t;

void LowPowerTimeout_attach_us(ticker_t*, void (*)(void), us_timestamp_t);
void LowPowerTimeout_attach_us_absolute(ticker_t*, void (*)(void), us_timestamp_t);
void LowPowerTimeout_detach(ticker_t*);
void LowPowerTimeout_waitUntil(ticker_t*, us_timestamp_t);

us_timestamp_t LowPowerTimeout_read_us(ticker_t*);
