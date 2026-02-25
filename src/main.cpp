#include <Arduino.h>
#include <PS4Controller.h>
#include <gn10_can/core/can_bus.hpp>
#include <gn10_can/devices/motor_driver_client.hpp>
#include "esp32_can_driver.hpp"
#include "mecanum_wheel.hpp"

MecanumWheel mecanum(0.3f, 0.4f, 0.05f, 45.0f); // 車体幅30cm、車体長40cm、車輪半径5cm、メカナムホイール角度45度
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
    can_driver.setpin(32, 33);
    if (!can_driver.begin(100E3)) {
        Serial.println("failed to initialize CAN driver!");
        while (true) {
            delay(1000);  // 無限ループで停止
        }
    }
    Serial.println("CAN driver initialized successfully.");

    PS4.begin("c0:5d:89:88:5e:44");  // PS4コントローラーの初期化
    // PS4コントローラーの接続を待つ
    Serial.println("Waiting for PS4 controller connection...");
    while (!PS4.isConnected()) {
        delay(1000);
    }
    Serial.println("PS4 controller connected.");

}

void loop() {
    if (PS4.isConnected()) {
        float vx = (PS4.LStickX() / 128.0f) * 1.0f; // 左スティックのX軸を速度に変換
        float vy = (PS4.LStickY() / 128.0f) * 1.0f; // 左スティックのY軸を速度に変換
        float omega = (PS4.RStickX() / 128.0f) * 1.0f; // 右スティックのX軸を回転速度
        // メカナムホイールの速度を計算
        mecanum.calculate_wheel_speed(vx, vy, omega);
        float fr, fl, rr, rl;
        // 車輪の角速度を取得
        mecanum.get_wheel_angular_velocity(&fr, &fl, &rl, &rr);
        // 速度をモータードライバーに送信
        motor_wheel_fr.set_target(fr);
        motor_wheel_fl.set_target(fl);
        motor_wheel_rr.set_target(rr);
        motor_wheel_rl.set_target(rl);
    }
    delay(10); // 100Hzで制御ループを回す
}