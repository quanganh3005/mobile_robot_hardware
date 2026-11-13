// Copyright (c) 2021 Juan Miguel Jimeno
// Copyright (c) 2023 Thomas Chou
// Licensed under the Apache License, Version 2.0

#include <Arduino.h>
#include <stdio.h>

#include "config.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"

// --- KHÔNG DÙNG INTERRUPT (Dùng PCNT Hardware của ESP32) ---
// #define ENCODER_USE_INTERRUPTS
// #define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"

#ifndef BAUDRATE
#define BAUDRATE 115200
#endif

// --- CHỈ KHAI BÁO 2 MOTOR ---
Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);

// PID khai báo để giữ cấu trúc, dù bài test này chạy max speed (Open Loop) nhưng có thể cần logic PID nền
PID motor1_pid(PWM_MIN, PWM_MAX, K_P_1, K_I_1, K_D_1);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P_2, K_I_2, K_D_2);

Kinematics kinematics(
    Kinematics::LINO_BASE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE
);

unsigned total_motors = 2; // CỐ ĐỊNH 2 MOTOR

void setup()
{
#ifdef BOARD_INIT 
    BOARD_INIT;
#endif

    Serial.begin(BAUDRATE);
    pinMode(LED_PIN, OUTPUT);
    
    // --- KHÔNG INIT I2C, IMU, MAG, BATTERY, WIFI, OTA ---
    // Chỉ reset Encoder
    motor1_encoder.getRPM();
    motor2_encoder.getRPM();
    
    Serial.println("--- MOTOR TEST FIRMWARE START ---");
    Serial.println("Testing Motor 1 and Motor 2...");
}

void loop() {
    static unsigned tk = 0; // tick counter
    const unsigned run_time = 8; // thời gian chạy mỗi motor (giây)
    const unsigned cycle = run_time * total_motors;
    
    // Logic xác định motor nào đang chạy
    unsigned current_motor = tk / run_time % total_motors; 
    unsigned direction = tk / cycle % 2; // 0: Quay tới, 1: Quay lui
    
    const int pwm_max = ((1 << PWM_BITS) - 1) * MAX_RPM_RATIO;
    static float max_rpm, stopping;

    // Đèn LED báo trạng thái chiều quay
    digitalWrite(LED_PIN, direction ? LOW : HIGH);

    // Điều khiển Motor: Chỉ chạy M1 hoặc M2, con kia dừng
    motor1_controller.spin((current_motor == 0) ? (direction ? -pwm_max : pwm_max) : 0);
    motor2_controller.spin((current_motor == 1) ? (direction ? -pwm_max : pwm_max) : 0);

    delay(1000); // Mỗi giây update 1 lần

    // Đọc RPM
    float current_rpm1 = motor1_encoder.getRPM();
    float current_rpm2 = motor2_encoder.getRPM();

    // Logic lưu giá trị max speed và stopping distance
    if (current_motor == 0 && tk % run_time == run_time - 1) max_rpm = current_rpm1;
    if (current_motor == 1 && tk % run_time == 0) stopping = current_rpm1;
    if (current_motor == 1 && tk % run_time == run_time - 1) max_rpm = current_rpm2;
    if (current_motor == 0 && tk % run_time == 0) stopping = current_rpm2;

    // In thông số tổng kết sau mỗi chu kỳ test xong 1 motor
    if (tk && tk % run_time == 0) {
        Kinematics::velocities max_linear = kinematics.getVelocities(max_rpm, max_rpm, max_rpm, max_rpm);
        Kinematics::velocities max_angular = kinematics.getVelocities(-max_rpm, max_rpm, -max_rpm, max_rpm);
        
        char buffer[150];
        sprintf(buffer, "--- REPORT MOTOR %d ---\n Speed: %6.2f m/s | %6.2f rad/s\n Stop Dist: %6.3f m", 
                current_motor ? current_motor : total_motors, // Fix logic hiển thị số thứ tự
                max_linear.linear_x, 
                max_angular.angular_z, 
                max_linear.linear_x * stopping / max_rpm);
        Serial.println(buffer);
    }

    // In thông số realtime
    char log_buffer[100];
    sprintf(log_buffer, "[%s] M%d Running | M1 RPM: %8.1f | M2 RPM: %8.1f",
       direction ? "REV" : "FWD",
       current_motor + 1,
       current_rpm1, current_rpm2);
    Serial.println(log_buffer);

    tk++;
}