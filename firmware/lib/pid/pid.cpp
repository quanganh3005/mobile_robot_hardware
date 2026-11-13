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

#include "Arduino.h"
#include "pid.h"

PID::PID(float min_val, float max_val, float kp, float ki, float kd):
    min_val_(min_val),
    max_val_(max_val),
    kp_(kp),
    ki_(ki),
    kd_(kd),
    integral_(0),
    derivative_(0),
    prev_error_(0),
    prev_time_(0)
{
}

void PID::reset()
{
    integral_ = 0;
    derivative_ = 0;
    prev_error_ = 0;
    prev_time_ = 0;
}

double PID::compute(float setpoint, float measured_value)
{
    double error = setpoint - measured_value;

    // 1. Dừng hẳn khi setpoint = 0
    const float stop_threshold = 1e-3;
    if (fabs(setpoint) < stop_threshold)
    {
        integral_ = 0;
        derivative_ = 0;
        prev_error_ = 0;
        prev_time_ = micros();
        return 0;
    }

    // 2. Tính dt thực tế
    unsigned long now = micros();
    double dt = 0.02; // Mặc định 20ms (tương ứng 50Hz của Linorobot2)
    if (prev_time_ != 0)
    {
        dt = (now - prev_time_) / 1.0e6;
        if (dt <= 0.0 || dt > 0.5)
        {
            dt = 0.02;
        }
    }
    prev_time_ = now;

    // 3. Khâu tích phân (Integral) có dt & Chống bão hòa (Anti-windup theo file tune)
    integral_ += error * dt;
    double i_limit = (ki_ > 0.0001) ? (max_val_ / ki_) : max_val_;
    integral_ = constrain(integral_, -i_limit, i_limit);

    // 4. Khâu vi phân (Derivative) có chia dt
    derivative_ = (error - prev_error_) / dt;
    prev_error_ = error;

    // 5. Tính PID tổng: Kp*e + Ki*integral + Kd*derivative
    double pid_out = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative_);

    return constrain(pid_out, min_val_, max_val_);
}

void PID::updateConstants(float kp, float ki, float kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}
