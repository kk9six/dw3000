#pragma once

#ifndef GENERAL_H
#define GENERAL_H

#define IMU_INT 26
#define IMU_RST 27
#define IMU_ADDR 0x4B
// I2C
#define IMU_SDA 21
#define IMU_SCL 22
// SPI
#define IMU_SCK 19  // SCK
#define IMU_SO 19   // MISO
#define IMU_SI 23   // MOSI
#define IMU_CS 15   // SS

#define INTERVAL 5     /* FPS = 1003 / INTERVAL */

#endif
