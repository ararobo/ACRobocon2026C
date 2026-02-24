#include <Arduino.h>
#include <PS4Controller.h>
#include <gn10_can/core/can_bus.hpp>
#include <gn10_can/devices/motor_driver_client.hpp>
#include "esp32_can_driver.hpp"

ESP32CANDriver can_driver;
gn10_can::CANBus can_bus(can_driver);
gn10_can::devices::MotorDriverClient motor_wheel_fr(can_bus, 0);
gn10_can::devices::MotorDriverClient motor_wheel_fl(can_bus, 1);
gn10_can::devices::MotorDriverClient motor_wheel_rr(can_bus, 2);
gn10_can::devices::MotorDriverClient motor_wheel_rl(can_bus, 3);

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        ;  // シリアルポートが接続されるのを待つ
    }
    can_driver.setpin(4, 5);  // RXピンをGPIO4、TXピンをGPIO5に設定
    if (!can_driver.begin(100E3)) {
        Serial.println("failed to initialize CAN driver!");
        while (true) {
            delay(1000);  // 無限ループで停止
        }
    }
    Serial.println("CAN driver initialized successfully.");

    PS4.begin();  // PS4コントローラーの初期化
    // PS4コントローラーの接続を待つ
    Serial.println("Waiting for PS4 controller connection...");
    while (!PS4.isConnected()) {
        delay(1000);
    }
    Serial.println("PS4 controller connected.");

}

void loop() {
    if (PS4.isConnected()) {
    }
}