#pragma once
#ifndef UWB_H

#include "dw3000.h"

#define NUM_NODES 3
#define INTERVAL 5
#define UWB_RST 27
#define UWB_IRQ 34
#define UWB_SS 4

#define FUNC_CODE_TAG 0xE2
#define FUNC_CODE_ANCHOR 0xE3
#define FUNC_CODE_RESET 0xE4

#define U0 0
#define U1 10
#define U2 14
#define U3 18
#define U4 22
#define U5 26
#define U6 30

#ifdef TAG
#define APP_NAME "UWB DW3000 U1"
#define UID U1
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#elif defined(ANCHOR_U2)
#define APP_NAME "UWB DW3000 U2"
#define UID U2
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ANCHOR
#define RX_TO_TX_DLY_UUS 580
#elif defined(ANCHOR_U3)
#define APP_NAME "UWB DW3000 U3"
#define UID U3
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ANCHOR
#define RX_TO_TX_DLY_UUS 1160
#elif defined(ANCHOR_U4)
#define APP_NAME "UWB DW3000 U4"
#define UID U4
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ANCHOR
#define RX_TO_TX_DLY_UUS 1740
#elif defined(ANCHOR_U5)
#define APP_NAME "UWB DW3000 U5"
#define UID U5
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ANCHOR
#define RX_TO_TX_DLY_UUS 2320
#elif defined(ANCHOR_U6)
#define APP_NAME "UWB DW3000 U6"
#define UID U6
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ANCHOR
#define RX_TO_TX_DLY_UUS 2900
#else
#define APP_NAME "UWB DW3000 UNKNOWN"
#define UID U0
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define ANCHOR
#define RX_TO_TX_DLY_UUS 1000
#endif

#define MSG_LEN 16         /* message length */
#define BUF_LEN MSG_LEN    /* buffer length */
#define MSG_SN_IDX 2       /* sequence number */
#define MSG_SID_IDX 7      /* source id */
#define MSG_FUNC_IDX 9     /* func code*/
#define MSG_T_REPLY_IDX 10 /* t_reply */
#define RESP_MSG_TS_LEN 4
#define TX_TO_RX_DLY_UUS 100
#define RX_TIMEOUT_UUS 400000

extern dwt_config_t config;

extern uint8_t frame_seq_nb;
extern uint8_t tx_msg[], rx_msg[];

#ifdef TAG
extern uint8_t rx_buffer[NUM_NODES - 1][BUF_LEN];
#else
extern uint8_t rx_buffer[BUF_LEN];
#endif

extern uint32_t status_reg;
extern int counter;
extern int ret;

#ifdef TAG
extern uint64_t rx_ts[NUM_NODES - 1];
#else
extern uint64_t rx_ts;
#endif

extern uint64_t tx_ts;
extern uint32_t tx_time;
extern uint32_t t_reply;
extern uint64_t t_round;
extern double tof, distance;
extern unsigned long previous_debug_millis, current_debug_millis;
extern int millis_since_last_serial_print;

#ifdef TAG
extern int target_uids[NUM_NODES - 1];
#endif

void start_uwb();
#ifdef TAG
void initiator();
#else
void responder();
#endif

#endif
