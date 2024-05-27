// SPI
// #define USE_VSPI
// #define USE_HSPI
// #define USE_I2C
#pragma once
#ifndef IMU_H
#define IMU_H

#include "SparkFun_BNO08x_Arduino_Library.h"

#define IMU_INT 26
#define IMU_RST 27
#define IMU_ADDR 0x4B

/* avoid using VSPI when integrating UWB and IMU. infinite loop of uwb will
 * block the connection */
#ifdef USE_VSPI
#define IMU_CS 5  // SS
// #define IMU_SCK 18  // SCK
// #define IMU_SO 19   // MISO
// #define IMU_SI 23   // MOSI
#elif defined(USE_HSPI)
#define IMU_SCK 14  // SCK
#define IMU_SO 25   // MISO
#define IMU_SI 13   // MOSI
#define IMU_CS 15   // SS
#elif defined(USE_I2C)
#define IMU_SDA 21
#define IMU_SCL 22
#else
/* Please define USE_VSPI/HSPI/I2C to use. */
#endif

extern BNO08x imu;

extern unsigned long previous_millis_imu;
extern unsigned long current_millis_imu;
extern int millis_since_last_ts;

extern uint64_t ts_acc, ts_ori;

extern uint8_t report_id;
extern float ax, ay, az;
extern float qx, qy, qz, qw;

#ifdef USE_HSPI
extern SPIClass spi_imu;
#endif

extern bool hasAcc, hasOri;

void start_imu();
void enable_reading();
void imu_handler();

#endif
