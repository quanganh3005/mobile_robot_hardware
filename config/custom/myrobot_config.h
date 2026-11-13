// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MYROBOT_CONFIG_H
#define MYROBOT_CONFIG_H

#define LED_PIN 2
#ifndef ESP32
    #define ESP32
#endif

// Use this or ENCODER_USE_INTERRUPTS, not both
// #define SOC_PCNT_SUPPORTED 
// This is replaced by ESP32Encoder library (PCNT hardware) in "encoder.h"

#define BAUDRATE 115200
#define LINO_BASE DIFFERENTIAL_DRIVE 

#define USE_GENERIC_2_IN_MOTOR_DRIVER // L298N motor driver
// #define USE_SHORT_BRAKE // for shorter stopping distance, only use when your robot requires high accuracy when stopping, slow or medium movement speed, and is lightweight

#define USE_MPU6050_IMU
#define SDA_PIN 21 // specify I2C pins for esp32
#define SCL_PIN 22

// #define ACCEL_COV { 0.01, 0.01, 0.01 }
// #define GYRO_COV { 0.001, 0.001, 0.001 }
// #define ORI_COV { 0.01, 0.01, 0.01 }
// #define POSE_COV { 0.001, 0.001, 0.001, 0.001, 0.001, 0.001 }
// #define TWIST_COV { 0.001, 0.001, 0.001, 0.003, 0.003, 0.003 }

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2
         BACK
*/
                           

// PID parameters for MOTOR 1 (Cập nhật từ file tune PID)
#define K_P_1 0.8
#define K_I_1 3.0
#define K_D_1 0.02
// PID parameters for MOTOR 2
#define K_P_2 0.8
#define K_I_2 3.0
#define K_D_2 0.02

#define MOTOR_MAX_RPM 280       
#define MAX_RPM_RATIO 0.85
#define MOTOR_OPERATING_VOLTAGE 11.8
#define MOTOR_POWER_MAX_VOLTAGE 11.8 // Max voltage between OUT1 and OUT2 (OR OUT3 and OUT4) of L298N
#define MOTOR_POWER_MEASURED_VOLTAGE 11.8 // Current voltage between OUT1 and OUT2 (OR OUT3 and OUT4) of L298N
#define COUNTS_PER_REV1 467 // 515 if use encoder_.attachSingleEdge(pin1, pin2); in "encoder.h"
#define COUNTS_PER_REV2 467 // 515 if use encoder_.attachSingleEdge(pin1, pin2); in "encoder.h"
#define COUNTS_PER_REV3 0 // set to 0 to avoid undefined attach PCNT to pin -1
#define COUNTS_PER_REV4 0 //  set to 0 to avoid undefined attach PCNT to pin -1
#define WHEEL_DIAMETER 0.075         // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.180// distance between left and right wheels

#define PWM_BITS 8                         
#define PWM_FREQUENCY 250

// Fixed pin numbers for ESP32-WROOM-32D 30 PIN VERSION
/// ENCODER PINS
#define MOTOR1_ENCODER_A 36
#define MOTOR1_ENCODER_B 39 
#define MOTOR1_ENCODER_INV true

#define MOTOR2_ENCODER_A 34
#define MOTOR2_ENCODER_B 35     
#define MOTOR2_ENCODER_INV false

#define MOTOR3_ENCODER_A -1
#define MOTOR3_ENCODER_B -1 
#define MOTOR3_ENCODER_INV true 

#define MOTOR4_ENCODER_A -1
#define MOTOR4_ENCODER_B -1
#define MOTOR4_ENCODER_INV false 

// Motor Pins
#define MOTOR1_PWM 25
#define MOTOR1_IN_A 33
#define MOTOR1_IN_B 32 
#define MOTOR1_INV false

#define MOTOR2_PWM 26
#define MOTOR2_IN_A 27
#define MOTOR2_IN_B 14 
#define MOTOR2_INV false

#define MOTOR3_PWM -1
#define MOTOR3_IN_A -1
#define MOTOR3_IN_B -1
#define MOTOR3_INV false

#define MOTOR4_PWM -1
#define MOTOR4_IN_A -1
#define MOTOR4_IN_B -1
#define MOTOR4_INV false

#define PWM_MAX ((1 << PWM_BITS) - 1)
#define PWM_MIN (-PWM_MAX)
// #define USE_WIFI_TRANSPORT
#define AGENT_IP { 10, 107, 89, 22 }  // eg IP of the desktop computer
#define AGENT_PORT 8888
// Enable WiFi with null terminated list of multiple APs SSID and password
#define WIFI_AP_LIST {{"WIFI_SSID", "WIFI_PASSWORD"}, {NULL}}
#define WIFI_MONITOR 2 // min. period to send wifi signal strength to syslog
// #define USE_ARDUINO_OTA
// #define USE_SYSLOG
#define SYSLOG_SERVER { 10, 107, 89, 22 }  // eg IP of the desktop computer
#define SYSLOG_PORT 514
#define DEVICE_HOSTNAME "linorobot2"
#define APP_NAME "hardware"
#endif

