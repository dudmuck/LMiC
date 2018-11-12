#include <stddef.h>
#include "lp_ticker_api.h"

typedef struct {
    ticker_event_t event;
    const ticker_data_t *_ticker_data;

    uint32_t id;
    void (*handler)(uint32_t);
} timer_event_t;

void timer_event_initialize(timer_event_t*);
void timer_event_remove(timer_event_t*);
void timer_event_insert_absolute(timer_event_t*, us_timestamp_t timestamp, uint32_t id, void (*)(uint32_t));

