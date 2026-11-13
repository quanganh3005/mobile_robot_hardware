#include <Arduino.h>
#include "config.h"

#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "kinematics.h"

/* ========= HARDWARE OBJECTS ========= */

// Encoder (2 bánh)
Encoder left_encoder(
    MOTOR1_ENCODER_A,
    MOTOR1_ENCODER_B,
    COUNTS_PER_REV1,
    MOTOR1_ENCODER_INV
);

Encoder right_encoder(
    MOTOR2_ENCODER_A,
    MOTOR2_ENCODER_B,
    COUNTS_PER_REV2,
    MOTOR2_ENCODER_INV
);

// Motor driver (2 bánh)
Motor left_motor(
    PWM_FREQUENCY,
    PWM_BITS,
    MOTOR1_INV,
    MOTOR1_PWM,
    MOTOR1_IN_A,
    MOTOR1_IN_B
);

Motor right_motor(
    PWM_FREQUENCY,
    PWM_BITS,
    MOTOR2_INV,
    MOTOR2_PWM,
    MOTOR2_IN_A,
    MOTOR2_IN_B
);

// PID parameters for MOTOR 1
#define K_P_1 0.8
#define K_I_1 3.0
#define K_D_1 0.02

// PID parameters for MOTOR 2
#define K_P_2 0.8
#define K_I_2 3.0
#define K_D_2 0.02

PID left_pid(PWM_MIN, PWM_MAX, K_P_1, K_I_1, K_D_1);
PID right_pid(PWM_MIN, PWM_MAX, K_P_2, K_I_2, K_D_2);

// Kinematics (DIFFERENTIAL)
Kinematics kinematics(
    Kinematics::DIFFERENTIAL_DRIVE,
    MOTOR_MAX_RPM,
    MAX_RPM_RATIO,
    MOTOR_OPERATING_VOLTAGE,
    MOTOR_POWER_MAX_VOLTAGE,
    WHEEL_DIAMETER,
    LR_WHEELS_DISTANCE
);

/* ========= CONTROL PARAM ========= */

float target_linear_x = 0.25;   // m/s
float target_angular_z = 0.0;  // rad/s

unsigned long last_loop = 0;
const unsigned long CONTROL_PERIOD_US = 100000; // 100 Hz

/* ========= SETUP ========= */

void setup()
{
    Serial.begin(115200);
    delay(1000);

    // Removed text printouts so that PlotJuggler only receives JSON

    left_pid.reset();
    right_pid.reset();

    left_encoder.write(0);
    right_encoder.write(0);

    last_loop = micros();
}

/* ========= LOOP ========= */

void loop()
{
    unsigned long now = micros();
    if (now - last_loop < CONTROL_PERIOD_US)
        return;

    last_loop = now;

    // 1️⃣ Desired RPM from kinematics
    Kinematics::rpm target_rpm = kinematics.getRPM(
        target_linear_x,
        0.0,
        target_angular_z
    );

    float left_target_rpm  = target_rpm.motor1;
    float right_target_rpm = target_rpm.motor2;

    // 2️⃣ Measured RPM from encoders
    float left_measured_rpm  = left_encoder.getRPM();
    float right_measured_rpm = right_encoder.getRPM();

    // 3️⃣ PID → PWM
    int left_pwm = left_pid.compute(left_target_rpm, left_measured_rpm);
    int right_pwm = right_pid.compute(right_target_rpm, right_measured_rpm);

    Kinematics::velocities current_vel = kinematics.getVelocities(left_measured_rpm, right_measured_rpm, 0, 0);

    // 4️⃣ Drive motors
    left_motor.spin(left_pwm);
    right_motor.spin(right_pwm);

    // 5️⃣ Debug output (JSON format for PlotJuggler)
    Serial.print("{\"L_Target\":"); Serial.print(left_target_rpm);
    Serial.print(",\"L_Meas\":"); Serial.print(left_measured_rpm);
    Serial.print(",\"L_PWM\":"); Serial.print(left_pwm);
    Serial.print(",\"R_Target\":"); Serial.print(right_target_rpm);
    Serial.print(",\"R_Meas\":"); Serial.print(right_measured_rpm);
    Serial.print(",\"R_PWM\":"); Serial.print(right_pwm);
    Serial.println("}");
}

