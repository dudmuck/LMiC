#include "main.h"
#include "debug.h"
#include <stdio.h>


u1_t onEvent (ev_t ev, const void* args)
{
    u1_t stop = 0;
    static u1_t cnt;
    bit_t conf;
    debug_event(ev);

    switch(ev) {

        // network joined, session established
#ifdef JOINEUI
        case EV_JOINED:
            debug_val("netid = ", LMIC.netid);
            printf("devAddr:%08x\r\n", LMIC.devaddr);
            cnt = 0;
            //goto tx;
        case EV_JOIN_FAILED:
        case EV_REJOIN_FAILED:
            stop = 1;
            break;
        case EV_JOINING:
        {
            u1_t devEui[8];
            os_getDevEui(devEui);
            for (unsigned n = 0; n < 8; n++)
                printf("%02x ", devEui[n]);
            printf("\r\n");
            break;
        }
#endif /* JOINEUI */

            // scheduled data sent (optionally data received)
        case EV_TXCOMPLETE:
            printf("datalen:%u ",LMIC.dataLen);
            if(LMIC.dataLen) { // data received in rx slot after tx
                debug_buf("frame", LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                _ble_nus_data_send(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
            }
#if 0
            tx:
            // immediately prepare next transmission
            LMIC.frame[0] = LMIC.snr;
            // schedule transmission (port 1, datalen 1, no ack requested)
            conf = (cnt++ & 3) == 3;
            if (conf)
                printf("%u Conf ", cnt);
            else
                printf("%u Unconf ", cnt);
            LMIC_setTxData2(1, LMIC.frame, 1, conf);
            // (will be sent as soon as duty cycle permits)
#endif /* if 0 */
            break;
        case EV_SCAN_TIMEOUT:
        case EV_BEACON_FOUND:
        case EV_BEACON_MISSED:
        case EV_BEACON_TRACKED:
        case EV_RFU1:
        case EV_LOST_TSYNC:
        case EV_RESET:
        case EV_RXCOMPLETE:
        case EV_LINK_DEAD:
        case EV_LINK_ALIVE:
        case EV_SCAN_FOUND:
        case EV_TXSTART:
            printf("ch%u, %u  dr%u\r\n", LMIC.txChnl, LMIC.freq, LMIC.datarate);
            break;
    }

    return stop;
}

#if 0
ticker_t test;
#define WD_INTERVAL     500000

void wdcb()
{
    static u4_t cnt;
    LowPowerTimeout_attach_us(&test, wdcb, WD_INTERVAL);
    printf("wd %u\r\n", cnt++);
}
#endif /* if 0 */

void lorawan_app_init(void)
{
    bit_t ok;
    // initialize runtime env
    LMIC_init();

    // reset MAC state
    LMIC_reset();
#if defined(JOINEUI) && defined(ROOT_NWKKEY)
    // start joining
    ok = LMIC_startJoining();
    printf("joining: %u\r\n", ok);
#elif defined(DEVADDR)
    /* ABP */
    LMIC_setSession();

    printf("ABP start\r\n");

    /* start first uplink */
    LMIC.frame[0] = 0;
    LMIC_setTxData2(1, LMIC.frame, 1, false);
#endif

    //LowPowerTimeout_attach_us(&test, wdcb, WD_INTERVAL);
}

void lorawan_app_mainloop(void)
{
    LMIC_mainloop();
    //sleep_manager_sleep_auto();
}

