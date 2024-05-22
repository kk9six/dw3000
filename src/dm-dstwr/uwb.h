#pragma once

#ifndef UWB_H
#define UWB_H

#include "dw3000.h"

#define NUM_NODES 4
#define INTERVAL 5 /* MAX FPS = 1000 / INTERVAL */
#define UWB_RST 27
#define UWB_IRQ 34
#define UWB_SS 4

#define FUNC_CODE_INTER 0xE2
#define FUNC_CODE_RESET 0xE3

#define U0 0
#define U1 14
#define U2 18
#define U3 22
#define U4 26
#define U5 30
#define U6 34

// #define U1 0x30
// #define U2 0x31
// #define U3 0x32
// #define U4 0x33
// #define U5 0x34
// #define U6 0x35

#ifdef MAIN_U1
#define APP_NAME "UWB DW3000 U1"
#define UID U1
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define POLL_NUM 0
#define RANGE_NUM 0
#define INITIATOR
#define REPORT_DISTANCE_FROM 0
#elif defined(MAIN_U2)
#define APP_NAME "UWB DW3000 U2"
#define UID U2
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define POLL_NUM 1
#define RANGE_NUM 1
#define REPORT_DISTANCE_FROM 1
#elif defined(MAIN_U3)
#define APP_NAME "UWB DW3000 U3"
#define UID U3
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define POLL_NUM 2
#define RANGE_NUM 2
#define REPORT_DISTANCE_FROM 2
#elif defined(MAIN_U4)
#define APP_NAME "UWB DW3000 U4"
#define UID U4
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define POLL_NUM 3
#define RANGE_NUM 3
#define REPORT_DISTANCE_FROM 3
#elif defined(MAIN_U5)
#define APP_NAME "UWB DW3000 U5"
#define UID U5
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define POLL_NUM 4
#define RANGE_NUM 4
#define REPORT_DISTANCE_FROM 4
#elif defined(MAIN_U6)
#define APP_NAME "UWB DW3000 U6"
#define UID U6
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define POLL_NUM 5
#define RANGE_NUM 5
#define REPORT_DISTANCE_FROM 5
#else
#define APP_NAME "UWB DW3000 UNKNOWN"
#define UID U0
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define POLL_NUM 0
#define RANGE_NUM 0
#define REPORT_DISTANCE_FROM 0
#endif

#define MSG_LEN 40 /* message length */
#define BUF_LEN 40 /* buffer length */
#define MSG_SN_IDX 2   /* sequence number */
#define MSG_SID_IDX 7  /* source id */
#define MSG_FUNC_IDX 9 /* func code*/
#define TX_TS_IDX 10 /* byte index of transmitter ts */
#define RESP_MSG_TS_LEN 4
#define TX_TO_RX_DLY_UUS 100
#define RX_TO_TX_DLY_UUS 1000
#define RX_TIMEOUT_UUS 400000

extern uint8_t tx_msg[], rx_msg[];
extern uint8_t frame_seq_nb;
extern uint8_t rx_buffer[NUM_NODES - 1][BUF_LEN];
extern int target_uids[NUM_NODES - 1];
extern uint32_t status_reg;
extern uint64_t poll_tx_ts, range_tx_ts;
extern uint32_t ack_tx_ts, range_rx_ts;
extern uint64_t ack_rx_ts[NUM_NODES - 1];
extern uint64_t range_rx_from_others_ts[NUM_NODES - 1];
extern uint64_t poll_rx_from_others_ts[NUM_NODES - 1];
extern uint32_t poll_rx_ts[NUM_NODES - 1];
extern double t_round_1, t_reply_1, t_round_2, t_reply_2;
extern double tof, distance;
extern bool wait_poll, wait_ack, wait_range, wait_final;
extern int counter;
extern int ret;
extern unsigned long previous_debug_millis, current_debug_millis;
extern int millis_since_last_serial_print;
extern uint32_t tx_time;
extern uint64_t tx_ts;
extern float clockOffsetRatioAck, clockOffsetRatioFinal;

void start_uwb();
void initiator();
void responder();

#endif
