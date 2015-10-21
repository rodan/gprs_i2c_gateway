
#include <stdio.h>
#include <string.h>

#include "drivers/uart0.h"
#include "drivers/sim900.h"
#include "drivers/uart1.h"
#include "drivers/timer_a0.h"
#include "drivers/flash.h"
#include "drivers/rtc.h"
#include "version.h"
#include "sensirion.h"
#include "serial_bitbang.h"
#include "hsc_ssc_i2c.h"
#include "ds3231.h"
#include "qa.h"

void display_memtest(const uint32_t start_addr, const uint32_t stop_addr, fm24_test_t test)
{
    uint32_t el;
    uint32_t rows_tested;

    snprintf(str_temp, STR_LEN, " \e[36;1m*\e[0m testing %lx - %lx with pattern #%d\t", start_addr, stop_addr, test);
    uart0_tx_str(str_temp, strlen(str_temp));

    el = fm24_memtest(start_addr, stop_addr, test, &rows_tested);

    if (el == 0) { 
        snprintf(str_temp, STR_LEN, "%lu bytes tested \e[32;1mok\e[0m\r\n", rows_tested * 8);
    } else {
        snprintf(str_temp, STR_LEN, "%lu bytes tested with \e[31;1m%lu errors\e[0m\r\n", rows_tested * 8, el );
    }
    uart0_tx_str(str_temp, strlen(str_temp));
}

void display_menu(void)
{
    snprintf(str_temp, STR_LEN,
            "\r\n --- ss build #%d\r\n  available commands:\r\n", BUILD);
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1m?\e[0m              - show menu\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1m!gprs [on/off]\e[0m - gprs power on/off\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1m!gprs init\e[0m     - gprs initial setup\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1m!gprs def\e[0m      - gprs start default task\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1m!sht\e[0m           - get SHT measurment\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1m!hsc\e[0m           - get HSC measurment\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1m!rtc\e[0m           - get RTC readout\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1m!mem store\e[0m     - store packet\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1m!mem test\e[0m      - memtest\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1m!mem read\e[0m      - read all external mem\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1m!flash read\e[0m    - read flash segment B\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1m!flash clear\e[0m   - clear flash segment B\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1m!chg [on/off]\e[0m  - charge on/off\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

    snprintf(str_temp, STR_LEN, " \e[33;1m!stat\e[0m          - system status\r\n" );
    uart0_tx_str(str_temp, strlen(str_temp));

}

void parse_user_input(void)
{
    char f = uart0_rx_buf[0];
    char *in = (char *) uart0_rx_buf;
    uint8_t *src_p;
    uint16_t i;
    uint8_t j;
    uint8_t row[8];
    uint8_t zeroes[128];
    struct ts t;

    if (f == '?') {
        display_menu();
    } else if (f == '!') {
        if (strstr(in, "gprs")) {
            if (strstr(in, "def")) {
            // gprs default task
                sim900_exec_default_task();
            } else if (strstr(in, "on")) {
            // gprs on
                sim900_start();
            } else if (strstr(in, "off")) {
            // gprs off
                sim900_halt();
            } else if (strstr(in, "init")) {
            // gprs init
                uart1_init(2400);
                sim900.cmd = CMD_FIRST_PWRON;
                sim900.next_state = SIM900_IDLE;
                timer_a0_delay_noblk_ccr2(SM_STEP_DELAY);
            }
        } else if (strstr(in, "rtcset")) {
            t.sec = inp2toi(in, 7);
            t.min = inp2toi(in, 9);
            t.hour = inp2toi(in, 11);
            t.wday = in[13] - 48;
            t.mday = inp2toi(in, 14);
            t.mon = inp2toi(in, 16);
            t.year = inp2toi(in, 18) * 100 + inp2toi(in, 20);
            if (DS3231_set(t) == EXIT_SUCCESS) {
                uart0_tx_str("ok\r\n", 4);
            } else {
                uart0_tx_str("fail\r\n", 6);
            }
        } else if (strstr(in, "rtc")) {

            if (DS3231_get(&t) == EXIT_FAILURE) {
                uart0_tx_str("fail\r\n", 6);
            }

#ifdef CONFIG_UNIXTIME
            snprintf(str_temp, STR_LEN, "%d.%02d.%02d %02d:%02d:%02d %ld\r\n", t.year,
                t.mon, t.mday, t.hour, t.min, t.sec, t.unixtime);
#else
            snprintf(str_temp, STR_LEN, "%d.%02d.%02d %02d:%02d:%02d\r\n", t.year,
                t.mon, t.mday, t.hour, t.min, t.sec);
#endif
            uart0_tx_str(str_temp, strlen(str_temp));
        } else if (strstr(in, "sht")) {
            uint16_t t2, rh;

            if (sht_get_meas(&t2, &rh) == EXIT_SUCCESS) {
                snprintf(str_temp, STR_LEN, "%d.%02ddC ", t2/100, t2%100);
                uart0_tx_str(str_temp, strlen(str_temp));
                snprintf(str_temp, STR_LEN, "%d.%02drH\r\n", rh/100, rh%100);
                uart0_tx_str(str_temp, strlen(str_temp));
            } else {
                uart0_tx_str("fail\r\n", 6);
            }
        } else if (strstr(in, "hsc")) {
            int16_t t;
            uint32_t p;
            struct cs_raw ps;
            uint8_t rv1;

            rv1 = ps_get_raw(PS_SLAVE_ADDR, &ps);
            if (rv1 == I2C_ACK) {
                ps_convert(ps, &p, &t, OUTPUT_MIN, OUTPUT_MAX, PRESSURE_MIN, PRESSURE_MAX);

                snprintf(str_temp, STR_LEN, "stat %d, ", ps.status);
                uart0_tx_str(str_temp, strlen(str_temp));
                snprintf(str_temp, STR_LEN, "%06ldp ", p);
                uart0_tx_str(str_temp, strlen(str_temp));
                snprintf(str_temp, STR_LEN, "%02d.%02ddC\r\n", t / 100, t % 100);
                uart0_tx_str(str_temp, strlen(str_temp));
            } else {
                snprintf(str_temp, STR_LEN, "err %d\r\n", rv1);
                uart0_tx_str(str_temp, strlen(str_temp));
            }
        } else if (strstr(in, "mem")) {
            if (strstr(in, "test")) {
            // mem test
                display_memtest(0, FM_LA, TEST_00);
                display_memtest(0, FM_LA, TEST_FF);
                display_memtest(0, FM_LA, TEST_AA);
                uart0_tx_str(" * roll over test\r\n", 19);
                display_memtest(FM_LA - 3, FM_LA + 5, TEST_FF);
            } else if (strstr(in, "store")) {
            // mem store
                adc_read();
                store_pkt();
            } else if (strstr(in, "read")) {
            // mem read
                for (i=0;i<(FM_LA+1)/8;i++) {
                    fm24_read_from(row, i * 8, 8);
                    for (j=0; j<8; j++) {
                        uart0_tx_str((char *)row + j, 1);
                    }
                }
            }
        } else if (strstr(in, "flash")) {
            if (strstr(in, "read")) {
            // flash read
                src_p = SEGMENT_B;
                for (i=0;i<128;i++) {
                    uart0_tx_str((char *)src_p + i, 1);
                }
            } else if (strstr(in, "clear")) {
                memset(zeroes, 0, 128);
                flash_save(SEGMENT_B, zeroes, 128);
            }
        } else if (strstr(in, "chg")) {
            if (strstr(in, "on")) {
                CHARGE_ENABLE;
                stat.should_charge = true;
            } else if (strstr(in, "off")) {
                CHARGE_DISABLE;
                stat.should_charge = false;
            }
        } else if (strstr(in, "foo")) {
            snprintf(str_temp, STR_LEN, "%d\r\n", fm24_sleep());
            uart0_tx_str(str_temp, strlen(str_temp));
        } else if (strstr(in, "bar")) {
            uint8_t data[] = "hello world";
            //fm24_write(data, 0, 11);
            fm24_read_from(data, 0, 11);
        } else if (strstr(in, "stat")) {

            adc_read();

            snprintf(str_temp, STR_LEN, "  Vbat %d.%02dV, Vraw %d.%02dV, charging ", stat.v_bat/100, stat.v_bat%100, stat.v_raw/100, stat.v_raw%100);
            uart0_tx_str(str_temp, strlen(str_temp));

            if (CHARGING_STOPPED) {
                uart0_tx_str("\e[31;1moff\e[0m ", 15);
            } else {
                uart0_tx_str("\e[32;1mon\e[0m ", 14);
            }

            uart0_tx_str("should be ", 10);

            if (stat.should_charge) {
                uart0_tx_str("\e[32;1mon\e[0m\r\n", 15);
            } else {
                uart0_tx_str("\e[31;1moff\e[0m\r\n", 16);
            }

            snprintf(str_temp, STR_LEN, "  tchg %lus\r\n", rtca_time.sys - charge_start);
            uart0_tx_str(str_temp, strlen(str_temp));
        }
    } else {
        sim900_tx_str((char *)uart0_rx_buf, uart0_p);
        sim900_tx_str("\r", 1);
    }
}

