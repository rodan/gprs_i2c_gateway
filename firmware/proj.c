
//  weather station system based on a MSP430F5510 uC
//
//  author:          Petre Rodan <petre.rodan@simplex.ro>
//  available from:  https://github.com/rodan/
//  license:         GNU GPLv3

#include <stdio.h>
#include <string.h>

#include "proj.h"
#include "drivers/sys_messagebus.h"
#include "drivers/rtc.h"
#include "drivers/timer_a0.h"
#include "drivers/uart0.h"
#include "drivers/uart1.h"
#include "drivers/adc.h"
#include "drivers/sim900.h"
#include "drivers/flash.h"
#include "drivers/fm24.h"
#include "drivers/fm24_memtest.h"
#include "qa.h"

uint32_t rtca_set_next = 0;

uint32_t status_show_next = 0;

uint32_t adc_check_next = 0;
uint16_t adc_check_interval = 15;

uint32_t movement_next = 0;

static void parse_gprs(enum sys_message msg)
{
    uart0_tx_str((char *)uart1_rx_buf, uart1_p);
    sim900_parse_rx((char *)uart1_rx_buf, uart1_p);
}

static void parse_UI(enum sys_message msg)
{
    parse_user_input();

    uart0_p = 0;
    uart0_rx_enable = 1;
    LED_OFF;
}

static void schedule(enum sys_message msg)
{

    // battery related
    if (rtca_time.sys > adc_check_next) {
        adc_read();

        adc_check_next = rtca_time.sys + adc_check_interval;

        if ((stat.v_raw > 400) && (stat.v_raw < 550)) {
            if (CHARGING_STOPPED) {
                if (stat.should_charge) {
                    CHARGE_DISABLE;
                    stat.should_charge = false;
                    adc_check_next = rtca_time.sys + 300;
                } else {
                    if (stat.v_bat < 390) {
                        CHARGE_ENABLE;
                        stat.should_charge = true;
                        charge_start = rtca_time.sys;
                    }
                }
            } else {
                if (rtca_time.sys > charge_start + 36000) {
                    CHARGE_DISABLE;
                    stat.should_charge = false;
                    adc_check_next = rtca_time.sys + 3600;
                }
            }
        } else {
            CHARGE_DISABLE;
            stat.should_charge = false;
        }
    }

    if (rtca_time.sys > gps_trigger_next) {
        // state machine
    }

    // GPRS related
    // force the HTTP POST from time to time
    if (rtca_time.sys > gprs_tx_next) {
        if (gprs_tx_trig & TG_NOW_MOVING) {
            gprs_tx_next = rtca_time.sys + s.gprs_moving_tx_interval;
        } else {
            gprs_tx_next = rtca_time.sys + s.gprs_static_tx_interval;
        }
        sim900.flags |= TX_FIX_RDY;
    }

    if (sim900.flags & BLACKOUT) {
       if (rtca_time.sys > gprs_blackout_lift) {
           sim900.flags &= ~BLACKOUT;
       }
    }

    if (((rtca_time.sys > gprs_trigger_next) || (sim900.flags & TX_FIX_RDY)) && 
            !(sim900.flags & TASK_IN_PROGRESS)) {

        // time to act
        adc_read();
        gprs_trigger_next = rtca_time.sys + s.gprs_loop_interval;

        if (stat.v_bat > 350) {
            // if battery voltage is below ~3.4v
            // the sim will most likely lock up while trying to TX
            if (!(sim900.flags & BLACKOUT)) {
                sim900_exec_default_task();
            }
        }
    }
}

int main(void)
{
    main_init();
    rtca_init();
    timer_a0_init();
    uart0_init();
    sim900_init_messagebus();
    sim900.next_state = SIM900_OFF;

    settings_init(SEGMENT_B, VERSION_BASED);
    //settings_apply();

    m.e = 0x0;
    m.seg[0] = 0x0;
    m.seg_num = 1;

    stat.http_post_version = POST_VERSION;
    stat.fix_id = 1;

    sim900.imei[0] = 0;
    sim900.flags = 0;


    gps_trigger_next = 0;
    gprs_trigger_next = s.gprs_loop_interval;

    rtca_set_next = 0;
    rtc_not_set = 1;
    gps_next_state = MAIN_GPS_IDLE;

    if (s.gps_invalidate_interval > s.gps_loop_interval) {
        s.gps_invalidate_interval = s.gps_loop_interval;
    }
   
    gprs_tx_trig = 0;
    gprs_tx_next = s.gprs_static_tx_interval;

    gprs_blackout_lift = 0;

    uart0_tx_str("solar sensor\r\n", 14);
    display_menu();

    sys_messagebus_register(&schedule, SYS_MSG_RTC_SECOND);
    sys_messagebus_register(&parse_UI, SYS_MSG_UART0_RX);
    sys_messagebus_register(&parse_gprs, SYS_MSG_UART1_RX);

#ifdef FM24_HAS_SLEEP_MODE
    //fm24_sleep();
#endif

    // main loop
    while (1) {
        _BIS_SR(LPM3_bits + GIE);
        //wake_up();
#ifdef USE_WATCHDOG
        // reset watchdog counter
        WDTCTL = (WDTCTL & 0xff) | WDTPW | WDTCNTCL;
#endif
        // new messages can be sent from within a check_events() call, so 
        // parse the message linked list multiple times
        check_events();
        check_events();
        check_events();

#ifdef FM24_HAS_SLEEP_MODE
        // sleep
        //if (fm24_status & FM24_AWAKE) {
        //    fm24_sleep();
        //}
#endif

        // P4.0 and P4.1
        //P4SEL &= ~0x3;
        
        /*
        PMMCTL0_H = 0xA5;
        SVSMHCTL &= ~SVMHE;
        SVSMLCTL &= ~(SVSLE+SVMLE);
        PMMCTL0_H = 0x00;
        */
    }
}

void main_init(void)
{

    // watchdog triggers after 4 minutes when not cleared
#ifdef USE_WATCHDOG
    WDTCTL = WDTPW + WDTIS__8192K + WDTSSEL__ACLK + WDTCNTCL;
#else
    WDTCTL = WDTPW + WDTHOLD;
#endif
    //SetVCore(3);

    // enable LF crystal
    P5SEL |= BIT5 + BIT4;
    UCSCTL6 &= ~(XT1OFF | XT1DRIVE0);

    P1SEL = 0x0;
    P1DIR = 0x0;
    //P1REN = 0x2;
    // make sure CTS is pulled low so the software doesn't get stuck 
    // in case the sim900 is missing - or broken.
    P1REN = 0x0;
    P1OUT = 0x0;

    P2SEL = 0x0;
    P2DIR = 0x0;
    P2OUT = 0x0;

    P3SEL = 0x0;
    P3DIR = 0x1f;
    P3OUT = 0x0;

    P4DIR = 0x0;
    P4OUT = 0x0;

    // mappings
    PMAPPWD = 0x02D52;
    // uart interface
    P4MAP2 = PM_UCA0TXD;
    P4MAP3 = PM_UCA0RXD;
    P4SEL |= 0xc;
    // try to also send uart0 tx to 4.1 XXX
    //P4MAP1 = PM_UCA0TXD;
    //P4SEL |= 0x2;
    PMAPPWD = 0;

    //P5SEL is set above
    P5DIR = 0xf;
    P5OUT = 0x0;

    P6SEL = 0x0;
    P6DIR = 0x0;
    P6OUT = 0x0;

    PJDIR = 0xFF;
    PJOUT = 0x00;

    // disable VUSB LDO and SLDO
    USBKEYPID = 0x9628;
    USBPWRCTL &= ~(SLDOEN + VUSBEN);
    USBKEYPID = 0x9600;

}

void check_events(void)
{
    struct sys_messagebus *p = messagebus;
    enum sys_message msg = 0;

    // drivers/timer0a
    if (timer_a0_last_event) {
        msg |= timer_a0_last_event;
        timer_a0_last_event = 0;
    }
    // drivers/uart0
    if (uart0_last_event & UART0_EV_RX) {
        msg |= BITA;
        uart0_last_event = 0;
    }
    // drivers/uart1
    if (uart1_last_event & UART1_EV_RX) {
        msg |= BITB;
        uart1_last_event = 0;
    }
    // drivers/rtca
    if (rtca_last_event & RTCA_EV_SECOND) {
        msg |= BITF;
        rtca_last_event = 0;
    }
    while (p) {
        // notify listener if he registered for any of these messages
        if (msg & p->listens) {
            p->fn(msg);
        }
        p = p->next;
    }
}

void settings_init(uint8_t * addr, const uint8_t location)
{
    uint8_t *src_p, *dst_p;
    uint8_t i;

    src_p = addr;
    dst_p = (uint8_t *) & s;
    if (((*src_p) != FLASH_VER) || (location)) {
        src_p = (uint8_t *) & defaults;
    }
    for (i = 0; i < sizeof(s); i++) {
        *dst_p++ = *src_p++;
    }
}

void settings_apply()
{
}

void adc_read()
{
    uint16_t q_bat = 0, q_raw = 0;
    uint32_t v_bat, v_raw;

    adc10_read(3, &q_bat, REFVSEL_1);
    adc10_read(2, &q_raw, REFVSEL_1);
    v_bat = (uint32_t) q_bat *s.vref * DIV_BAT / 10000;
    v_raw = (uint32_t) q_raw *s.vref * DIV_RAW / 10000;
    stat.v_bat = v_bat;
    stat.v_raw = v_raw;
    adc10_halt();
}

void store_pkt()
{
}

void gps_enable(void)
{
    P6OUT |= BIT0;
    P4SEL |= 0xc;
}

void gps_disable(void)
{
    P6OUT &= ~BIT0;
    P4SEL &= ~0xc;
}

