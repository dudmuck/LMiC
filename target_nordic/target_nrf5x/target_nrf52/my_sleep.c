
#include "sleep_api.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_pwr_mgmt.h"

volatile bool deep_wake = false;
volatile us_timestamp_t deep_wake_at;

void hal_sleep(void)
{
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    nrf_pwr_mgmt_run();
}

void hal_deepsleep(void)
{
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    nrf_pwr_mgmt_run();
}
