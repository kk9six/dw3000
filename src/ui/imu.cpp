#include "ui/imu.h"

BNO08x imu;

unsigned long previous_millis_imu = 0;
unsigned long current_millis_imu = 0;
int millis_since_last_ts;

uint64_t ts_acc = 0;
uint64_t ts_ori = 0;

uint8_t report_id = 0x00;
float ax = 0.0;
float ay = 0.0;
float az = 0.0;
float qx = 0.0;
float qy = 0.0;
float qz = 0.0;
float qw = 0.0;

#ifdef USE_HSPI
SPIClass spi_imu = SPIClass();
#endif

bool hasAcc, hasOri = false;

void enable_reading() {
    Serial.println("Setting desired reports");
    imu.enableARVRStabilizedGameRotationVector(12);
    imu.enableLinearAccelerometer(12);
    delay(100);
}

void start_imu() {
    Serial.println("Starting Sparkfun BNO086");
#ifdef USE_I2C
    Wire.flush();
    Wire.begin(IMU_SDA, IMU_SCL, 400000);
    delay(1000);
    if (imu.begin(IMU_ADDR, Wire, IMU_INT, IMU_RST) == false) {
        Serial.println(
            "BNO08x not detected at default I2C address. Check your "
            "jumpers and the hookup guide. Freezing...");
        while (1);
    }
#elif defined(USE_VSPI)
    if (imu.beginSPI(IMU_CS, IMU_INT, IMU_RST, 3000000) == false) {
        Serial.println(
            "BNO08x not detected at default SPI address. Check your "
            "jumpers and the hookup guide. Freezing...");
        while (1);
    }
#elif defined(USE_HSPI)
    spi_imu.begin(IMU_SCK, IMU_SO, IMU_SI, IMU_CS);
    if (imu.beginSPI(IMU_CS, IMU_INT, IMU_RST, 3000000, spi_imu) == false) {
        Serial.println(
            "BNO08x not detected at default SPI address. Check your "
            "jumpers and the hookup guide. Freezing...");
        while (1);
    }
#else /* lack necessary defination */
    Serial.println("Please define USE_VSPI/HSPI/I2C to use. Freezing...");
    while (1);

#endif
    Serial.println("BNO086 started");
    enable_reading();
}

void imu_handler() {
    if (imu.wasReset()) {
        Serial.println("IMU was reset");
        enable_reading();
    }
    while (!hasAcc | !hasOri) {
        if (imu.getSensorEvent() == true) {
            report_id = imu.getSensorEventID();
            if (report_id == SENSOR_REPORTID_LINEAR_ACCELERATION) {
                ax = imu.getLinAccelX();
                ay = imu.getLinAccelY();
                az = imu.getLinAccelZ();
                hasAcc = true;
            }
            if (report_id ==
                SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR) {
                qx = imu.getGameQuatI();
                qy = imu.getGameQuatJ();
                qz = imu.getGameQuatK();
                qw = imu.getGameQuatReal();
                hasOri = true;
            }
        }
        delayMicroseconds(100);
    }
    hasAcc = false;
    hasOri = false;

    current_millis_imu = millis();
    millis_since_last_ts = (current_millis_imu - previous_millis_imu);
    Serial.print("\t");
    Serial.print(millis_since_last_ts);
    Serial.print("\t");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.print("\t");
    Serial.print(qw);
    Serial.print(",");
    Serial.print(qx);
    Serial.print(",");
    Serial.print(qy);
    Serial.print(",");
    Serial.print(qz);
    Serial.print(",");
    Serial.println();
    previous_millis_imu = current_millis_imu;
    delay(5);
    // delayMicroseconds(500);
}
