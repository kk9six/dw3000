#include "at_dstwr/uwb.h"

#include "dw3000.h"

dwt_config_t config = {
    5,            /* Channel number. */
    DWT_PLEN_128, /* Preamble length. Used in TX only. */
    DWT_PAC8,     /* Preamble acquisition chunk size. Used in RX only. */
    9,            /* TX preamble code. Used in TX only. */
    9,            /* RX preamble code. Used in RX only. */
    1, /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2
          for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    DWT_PHRRATE_STD,  /* PHY header rate. */
    (129 + 8 - 8),    /* SFD timeout (preamble length + 1 + SFD length - PAC
                         size).    Used in RX only. */
    DWT_STS_MODE_OFF, /* STS disabled */
    DWT_STS_LEN_64,   /* STS length see allowed values in Enum
                       * dwt_sts_lengths_e
                       */
    DWT_PDOA_M0       /* PDOA mode off */
};
extern dwt_txconfig_t txconfig_options;

uint8_t tx_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, UID,
                    0,    0,    0, 0,    0,    0, 0, 0};
uint8_t rx_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, UID,
                    0,    0,    0, 0,    0,    0, 0, 0};
uint8_t frame_seq_nb = 0;
uint8_t rx_buffer[BUF_LEN];

uint32_t status_reg = 0;
bool wait_poll = true;
bool wait_final = false;
bool wait_ack = true;
bool wait_range = false;
int counter = 0;
int ret;

uint64_t poll_tx_ts, poll_rx_ts, range_tx_ts, ack_tx_ts, range_rx_ts;
uint32_t t_reply_1[NUM_NODES - 1];
uint64_t t_reply_2;
uint64_t t_round_1[NUM_NODES - 1];
uint32_t t_round_2[NUM_NODES - 1];
double tof, distance;
unsigned long previous_debug_millis = 0;
unsigned long current_debug_millis = 0;
int millis_since_last_serial_print;
uint32_t tx_time;
uint64_t tx_ts;

int target_uids[NUM_NODES - 1];

void set_target_uids() {
/*
 * U1 is the initiator, U2 - U6 are responders
 * U1 - U6 are the target UIDs
 */
#ifdef TAG
    switch (NUM_NODES) {
        case 6:
            target_uids[4] = U6;
        case 5:
            target_uids[3] = U5;
        case 4:
            target_uids[2] = U4;
        case 3:
            target_uids[1] = U3;
        case 2:
            target_uids[0] = U2;
        default:
            break;
    }
#elif defined(ANCHOR_U2)
    switch (NUM_NODES) {
        case 6:
            target_uids[4] = U6;
        case 5:
            target_uids[3] = U5;
        case 4:
            target_uids[2] = U4;
        case 3:
            target_uids[1] = U3;
        case 2:
            target_uids[0] = U1;
        default:
            break;
    }
#elif defined(ANCHOR_U3)
    switch (NUM_NODES) {
        case 6:
            target_uids[4] = U6;
        case 5:
            target_uids[3] = U5;
        case 4:
            target_uids[2] = U4;
        case 3:
            target_uids[1] = U2;
        case 2:
            target_uids[0] = U1;
        default:
            break;
    }
#elif defined(ANCHOR_U4)
    switch (NUM_NODES) {
        case 6:
            target_uids[4] = U6;
        case 5:
            target_uids[3] = U5;
        case 4:
            target_uids[2] = U3;
        case 3:
            target_uids[1] = U2;
        case 2:
            target_uids[0] = U1;
        default:
            break;
    }
#elif defined(ANCHOR_U5)
    switch (NUM_NODES) {
        case 6:
            target_uids[4] = U6;
        case 5:
            target_uids[3] = U4;
        case 4:
            target_uids[2] = U3;
        case 3:
            target_uids[1] = U2;
        case 2:
            target_uids[0] = U1;
        default:
            break;
    }
#elif defined(ANCHOR_U6)
    switch (NUM_NODES) {
        case 6:
            target_uids[4] = U5;
        case 5:
            target_uids[3] = U4;
        case 4:
            target_uids[2] = U3;
        case 3:
            target_uids[1] = U2;
        case 2:
            target_uids[0] = U1;
        default:
            break;
    }
#endif
}

void start_uwb() {
    while (!dwt_checkidlerc()) {
        UART_puts("IDLE FAILED\r\n");
        while (1);
    }

    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        UART_puts("INIT FAILED\r\n");
        while (1);
    }

    dwt_setleds(DWT_LEDS_DISABLE);

    if (dwt_configure(&config)) {
        UART_puts("CONFIG FAILED\r\n");
        while (1);
    }

    dwt_configuretxrf(&txconfig_options);
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    dwt_setrxaftertxdelay(TX_TO_RX_DLY_UUS);
#ifdef TAG
    dwt_setrxtimeout(RX_TIMEOUT_UUS);
#else
    dwt_setrxtimeout(0);
#endif
    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    set_target_uids();
    Serial.println(APP_NAME);
    Serial.println(UID);
    Serial.println("Setup over........");
}

void initiator() {
    if (!wait_ack && !wait_final && (counter == 0)) {
        wait_ack = true;
        tx_msg[MSG_SN_IDX] = frame_seq_nb;
        tx_msg[MSG_FUNC_IDX] = FUNC_CODE_POLL;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        dwt_writetxdata((uint16_t)(MSG_LEN), tx_msg, 0);
        dwt_writetxfctrl((uint16_t)(MSG_LEN), 0, 1);
        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    } else {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
    }

    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO |
              SYS_STATUS_ALL_RX_ERR))) {
    };

    /* receive ack msg or final msg */
    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        dwt_readrxdata(rx_buffer, BUF_LEN, 0);
        if (rx_buffer[MSG_SID_IDX] != target_uids[counter]) {
            dwt_write32bitreg(SYS_STATUS_ID,
                              SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            counter = 0;
            wait_ack = false;
            wait_final = false;
            return;
        }
        if (wait_ack) {
            poll_tx_ts = get_tx_timestamp_u64();
            t_round_1[counter] = get_rx_timestamp_u64() - poll_tx_ts;
            resp_msg_get_ts(&rx_buffer[MSG_T_REPLY_IDX], &t_reply_1[counter]);
            ++counter;
        } else {
            resp_msg_get_ts(&rx_buffer[MSG_T_REPLY_IDX], &t_round_2[counter]);
            ++counter;
        }
    } else { /* timeout or error, reset, send ack*/
        tx_msg[MSG_SN_IDX] = frame_seq_nb;
        tx_msg[MSG_FUNC_IDX] = FUNC_CODE_RESET;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        dwt_writetxdata((uint16_t)(MSG_LEN), tx_msg, 0);
        dwt_writetxfctrl((uint16_t)(MSG_LEN), 0, 1);
        dwt_starttx(DWT_START_TX_IMMEDIATE);
        dwt_write32bitreg(SYS_STATUS_ID,
                          SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
        wait_ack = false;
        wait_final = false;
        counter = 0;
        Sleep(1);
        return;
    }
    if (wait_ack && (counter == NUM_NODES - 1)) { /* received all ack msg */
        /* send range msg */
        tx_time =
            (get_rx_timestamp_u64() + (RX_TO_TX_DLY_UUS * UUS_TO_DWT_TIME)) >>
            8;
        tx_ts = (((uint64_t)(tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
        dwt_setdelayedtrxtime(tx_time);
        tx_msg[MSG_SN_IDX] = frame_seq_nb;
        tx_msg[MSG_FUNC_IDX] = FUNC_CODE_RANGE;
        dwt_writetxdata((uint16_t)(BUF_LEN), tx_msg, 0);
        dwt_writetxfctrl((uint16_t)(BUF_LEN), 0, 1);
        ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
        if (ret == DWT_SUCCESS) {
            while (!(dwt_read32bitreg(SYS_STATUS_ID) &
                     SYS_STATUS_TXFRS_BIT_MASK)) {
            };
            wait_ack = false;
            wait_final = true;
            counter = 0;
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
        }
        return;
    }
    if (wait_final && (counter == NUM_NODES - 1)) { /* received all final msg */
        range_tx_ts = get_tx_timestamp_u64();
        current_debug_millis = millis();
        Serial.print(current_debug_millis - previous_debug_millis);
        Serial.print("ms\t");
        for (int i = 0; i < counter; i++) {
            t_reply_2 = range_tx_ts - (t_round_1[i] + poll_tx_ts);
            tof = (t_round_1[i] * t_round_2[i] - t_reply_1[i] * t_reply_2) /
                  (t_round_1[i] + t_round_2[i] + t_reply_1[i] + t_reply_2) *
                  DWT_TIME_UNITS;
            distance = tof * SPEED_OF_LIGHT;
            snprintf(dist_str, sizeof(dist_str), "%3.3f m\t", distance);
            Serial.print(target_uids[i]);
            Serial.print("\t");
            Serial.print(dist_str);
        }
        Serial.println();
        previous_debug_millis = current_debug_millis;
        counter = 0;
        wait_ack = false;
        wait_final = false;
        ++frame_seq_nb;
        Sleep(INTERVAL);
    }
}

void responder() {
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) &
             (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR))) {
    };
    if (status_reg & SYS_STATUS_RXFCG_BIT_MASK) { /* receive msg */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
        dwt_readrxdata(rx_buffer, BUF_LEN, 0);
        if (rx_buffer[MSG_FUNC_IDX] == FUNC_CODE_RESET) {
            wait_poll = true;
            wait_range = false;
            counter = 0;
            return;
        }
        if (rx_buffer[MSG_SID_IDX] != target_uids[counter]) {
            dwt_write32bitreg(SYS_STATUS_ID,
                              SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);
            wait_poll = true;
            wait_range = false;
            counter = 0;
            return;
        }
        if (wait_poll) {  // received poll from U1
            if (counter == 0) {
                poll_rx_ts = get_rx_timestamp_u64();
            }
            ++counter;
        } else if (wait_range) {
            if (counter == 0) {
                range_rx_ts = get_rx_timestamp_u64();
            }
            ++counter;
        }
    } else {
        wait_poll = true;
        wait_range = false;
        counter = 0;
        dwt_write32bitreg(SYS_STATUS_ID,
                          SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO);
        return;
    }
    if (wait_poll) {
        if (counter == WAIT_NUM) {
            tx_time = (get_rx_timestamp_u64() +
                       (RX_TO_TX_DLY_UUS * UUS_TO_DWT_TIME)) >>
                      8;
            tx_ts = (((uint64_t)(tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
            dwt_setdelayedtrxtime(tx_time);
            tx_msg[MSG_SN_IDX] = frame_seq_nb;
            tx_msg[MSG_FUNC_IDX] = FUNC_CODE_ACK;
            resp_msg_set_ts(&tx_msg[MSG_T_REPLY_IDX], tx_ts - poll_rx_ts);
            dwt_writetxdata((uint16_t)(BUF_LEN), tx_msg, 0);
            dwt_writetxfctrl((uint16_t)(BUF_LEN), 0, 1);
            ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
            if (ret == DWT_SUCCESS) {
                while (!(dwt_read32bitreg(SYS_STATUS_ID) &
                         SYS_STATUS_TXFRS_BIT_MASK)) {
                };
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
            }
        }
        if (counter == NUM_NODES - 1) { /* all anchors sent the ack msgs */
            counter = 0;
            wait_poll = false;
            wait_range = true;
            return;
        }
    }
    if (wait_range) {
        if (counter == WAIT_NUM) {
            /* send final msg */
            ack_tx_ts = get_tx_timestamp_u64(); /* ack tx */
            tx_time = (get_rx_timestamp_u64() +
                       (RX_TO_TX_DLY_UUS * UUS_TO_DWT_TIME)) >>
                      8;
            tx_ts = (((uint64_t)(tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
            dwt_setdelayedtrxtime(tx_time);
            resp_msg_set_ts(&tx_msg[MSG_T_REPLY_IDX], range_rx_ts - ack_tx_ts);
            tx_msg[MSG_SN_IDX] = frame_seq_nb;
            tx_msg[MSG_FUNC_IDX] = FUNC_CODE_FINAL;
            dwt_writetxdata((uint16_t)(BUF_LEN), tx_msg, 0);
            dwt_writetxfctrl((uint16_t)(BUF_LEN), 0, 1);
            ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
            if (ret == DWT_SUCCESS) {
                while (!(dwt_read32bitreg(SYS_STATUS_ID) &
                         SYS_STATUS_TXFRS_BIT_MASK)) {
                };
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
            }
        }
        if (counter == NUM_NODES - 1) {
            /* all anchors sent the final msgs */
            counter = 0;
            wait_poll = true;
            wait_range = false;
            ++frame_seq_nb;
            return;
        }
    }
}
