#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <Wire.h>

#include "SparkFun_BNO08x_Arduino_Library.h"
#include "dw3000.h"
#include "ui/imu.h"
#include "ui/uwb.h"

TaskHandle_t imuTaskHandle;

void imu_loop(void *pvParameters);

void print_mac_addr() {
    Serial.print("ESP Board MAC Address:  ");
    Serial.println(WiFi.macAddress());
}

void setup() {
    UART_init();
    spiBegin(UWB_IRQ, UWB_RST);
    spiSelect(UWB_SS);
    delay(2);
    start_uwb();
    start_imu();

    xTaskCreatePinnedToCore(imu_loop, "IMU Handler",
                            CONFIG_ARDUINO_LOOP_STACK_SIZE, NULL, 1,
                            &imuTaskHandle, 0);
    delay(200);
}

void imu_loop(void *pvParameters) {
    for (;;) {
        imu_handler();
    }
    vTaskDelete(NULL);
}

void loop() {
#ifdef INITIATOR
    initiator();
#else
    responder();
#endif
}
