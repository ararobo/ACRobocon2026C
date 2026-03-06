#include <Arduino.h>
#include <PS4Controller.h>

#include <gn10_can/core/can_bus.hpp>
#include <gn10_can/devices/motor_driver_client.hpp>
#include <gn10_can/devices/servo_motor_client.hpp>

#include "esp32_can_driver.hpp"
#include "mecanum_wheel.hpp"

/// PS4スティックのデッドゾーン比率 (10%)
constexpr float DEADZONE_RATIO = 0.1f;

MecanumWheel mecanum(
    0.3f,
    0.4f,
    3.0f,
    45.0f);  // 車体幅30cm、車体長40cm、車輪半径は無視して出力値の制限として利用、メカナムホイール角度45度
ESP32CANDriver can_driver;
gn10_can::CANBus can_bus(can_driver);
gn10_can::devices::MotorDriverClient motor_wheel_fr(can_bus, 0);
gn10_can::devices::MotorDriverClient motor_wheel_fl(can_bus, 1);
gn10_can::devices::MotorDriverClient motor_wheel_rr(can_bus, 2);
gn10_can::devices::MotorDriverClient motor_wheel_rl(can_bus, 3);
gn10_can::devices::MotorDriverClient motor_wheel_right_arm_up(can_bus, 4);
gn10_can::devices::MotorDriverClient motor_wheel_left_arm_up(can_bus, 5);
gn10_can::devices::MotorDriverClient motor_wheel_power(can_bus, 6);
gn10_can::devices::MotorDriverClient motor_wheel_l_fry(can_bus, 7);
gn10_can::devices::MotorDriverClient motor_wheel_r_fry(can_bus, 8);

gn10_can::devices::MotorConfig motor_config_wheel;
gn10_can::devices::MotorConfig motor_config_arm;
gn10_can::devices::MotorConfig motor_config_power;
gn10_can::devices::MotorConfig motor_config_fry;

gn10_can::devices::ServoMotorClient servo_left(can_bus, 0);
gn10_can::devices::ServoMotorClient servo_right(can_bus, 1);
gn10_can::devices::ServoMotorClient servo_center(can_bus, 2);

float servo_left_angle   = PI / 2.0f;  // 左サーボの角度 (ラジアン)
float servo_right_angle  = PI / 2.0f;  // 右サーボの角度 (ラジアン)
float servo_center_angle = PI / 2.0f;  // 中央サーボの角度 (ラジアン)

float turn_multiplier = 4.0f;  // 旋回速度の倍率

/**
 * @brief PS4スティック入力にデッドゾーンを適用し、正規化された値を返す
 * @param raw    PS4スティックの生値 (-128 ~ 127)
 * @param deadzone_ratio デッドゾーンの比率 (0.0 ~ 1.0)
 * @return デッドゾーン適用済みの正規化値 (-1.0 ~ 1.0)
 */
static float apply_deadzone(int8_t raw,
                            float deadzone_ratio) {  // もしデッドゾーンだったら値を０にする
    const float normalized = raw / 128.0f;
    if (fabsf(normalized) < deadzone_ratio) {
        return 0.0f;
    }
    return normalized;
}

void setup() {
    // モータードライバ足回りの設定
    motor_config_wheel.set_max_duty_ratio(1.0f);
    motor_config_wheel.set_accel_ratio(1.0f);
    motor_config_wheel.set_encoder_type(gn10_can::devices::EncoderType::None);
    motor_config_wheel.set_feedback_cycle(100);
    // モータードライバアームの設定
    motor_config_arm.set_max_duty_ratio(0.1f);
    motor_config_arm.set_accel_ratio(1.0f);
    motor_config_arm.set_encoder_type(gn10_can::devices::EncoderType::None);
    // モータードライバパワーの設定
    motor_config_power.set_max_duty_ratio(0.2f);
    motor_config_power.set_accel_ratio(1.0f);
    motor_config_power.set_encoder_type(gn10_can::devices::EncoderType::None);
    // モータードライバ射出の設定
    motor_config_fry.set_max_duty_ratio(0.2f);
    motor_config_fry.set_accel_ratio(1.0f);
    motor_config_fry.set_encoder_type(gn10_can::devices::EncoderType::None);

    Serial.begin(115200);
    while (!Serial) {
        ;  // シリアルポートが接続されるのを待つ
    }
    can_driver.setpin(32, 33);
    if (!can_driver.begin(1000E3)) {
        Serial.println("failed to initialize CAN driver!");
        while (true) {
            delay(1000);  // 無限ループで停止
        }
    }
    Serial.println("CAN driver initialized successfully.");
    // モータードライバーの初期化コマンドを送信
    motor_wheel_fr.set_init(motor_config_wheel);
    motor_wheel_fl.set_init(motor_config_wheel);
    motor_wheel_rr.set_init(motor_config_wheel);
    motor_wheel_rl.set_init(motor_config_wheel);
    motor_wheel_right_arm_up.set_init(motor_config_arm);
    motor_wheel_left_arm_up.set_init(motor_config_arm);
    motor_wheel_power.set_init(motor_config_power);
    motor_wheel_l_fry.set_init(motor_config_fry);
    motor_wheel_r_fry.set_init(motor_config_fry);
    servo_left.set_init(500, 2500);    // サーボの初期化 (最小500us、最大2500us)
    servo_right.set_init(500, 2500);   // サーボの初期化 (最小500us、最大2500us)
    servo_center.set_init(500, 2500);  // サーボの初期化 (最小500us、最大2500us)
    Serial.println("Motor drivers initialized.");

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
        // デッドゾーン適用済みの正規化スティック入力を取得
        float vx = apply_deadzone(PS4.LStickX(), DEADZONE_RATIO);  // 左スティックX軸 → 横移動速度
        float vy = apply_deadzone(PS4.LStickY(), DEADZONE_RATIO);  // 左スティックY軸 → 縦移動速度
        float omega = apply_deadzone(PS4.RStickX(), DEADZONE_RATIO);  // 右スティックX軸 → 旋回速度
        // メカナムホイールの速度を計算
        mecanum.calculate_wheel_speed(vx, vy, omega * turn_multiplier);
        float fr, fl, rr, rl;
        // 車輪の角速度を取得
        mecanum.get_wheel_angular_velocity(&fr, &fl, &rl, &rr);
        // 速度をモータードライバーに送信
        motor_wheel_fr.set_target(fr);
        motor_wheel_fl.set_target(fl);
        motor_wheel_rr.set_target(rr);
        motor_wheel_rl.set_target(rl);

        if (PS4.Triangle()) {
            motor_wheel_right_arm_up.set_target(-1.0f);  // 右アームを正転
        } else if (PS4.Cross()) {
            motor_wheel_right_arm_up.set_target(1.0f);  // 右アームを逆転
        } else {
            motor_wheel_right_arm_up.set_target(0.0f);  // 右アームを停止
        };

        if (PS4.Up()) {
            motor_wheel_left_arm_up.set_target(1.0f);  // 左アームを正転
        } else if (PS4.Down()) {
            motor_wheel_left_arm_up.set_target(-1.0f);  // 左アームを逆転
        } else {
            motor_wheel_left_arm_up.set_target(0.0f);  // 左アームを停止
        };

        if (PS4.R1()) {
            motor_wheel_power.set_target(1.0f);  // 巻き取る
        } else if (PS4.L1()) {
            motor_wheel_power.set_target(-1.0f);  // 繰り出す
        } else {
            motor_wheel_power.set_target(0.0f);  // 停止
        };

        if (PS4.Touchpad()) {
            motor_wheel_l_fry.set_target(1.0f);  // 回す
        } else {
            motor_wheel_l_fry.set_target(0.0f);  // 停止
        };

        if (PS4.Touchpad()) {
            motor_wheel_r_fry.set_target(-1.0f);  // 回す
        } else {
            motor_wheel_r_fry.set_target(0.0f);  // 停止
        };

        if (PS4.Square()) {
            servo_left_angle -= 0.01f;
        } else if (PS4.Circle()) {
            servo_left_angle += 0.01f;
        }

        if (PS4.Up()) {
            servo_right_angle += 0.01f;
        } else if (PS4.Down()) {
            servo_right_angle -= 0.01f;
        }

        if (PS4.R2()) {
            servo_center_angle += 0.01f;
        } else if (PS4.L2()) {
            servo_center_angle -= 0.01f;
        }

        Serial.printf("Servo Angles (deg) - L: %.1f, R: %.1f, C: %.1f\n",
                      servo_left_angle * 180.0f / PI,
                      servo_right_angle * 180.0f / PI,
                      servo_center_angle * 180.0f / PI);

        // サーボの角度を送信
        servo_left.set_angle_rad(servo_left_angle);
        servo_right.set_angle_rad(servo_right_angle);
        servo_center.set_angle_rad(servo_center_angle);
        delay(10);  // 100Hzで制御ループを回す
    }
}