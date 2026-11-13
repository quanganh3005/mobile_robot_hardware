// Copyright (c) 2026 Gemini AI Thought Partner
// Code hiệu chuẩn IMU cho Linorobot2 - MPU6050

#include <Arduino.h>
#include <Wire.h>
#include <sensor_msgs/msg/imu.h> 
#include "config.h"
#include "odometry.h"
#include "imu.h"

// --- CẤU HÌNH ---
#define BAUDRATE 115200
#define CALIBRATION_SAMPLES 1000  // Số mẫu để lấy trung bình

// --- BIẾN TOÀN CỤC ---
IMU imu;
sensor_msgs__msg__Imu imu_msg;

float gyro_bias_x = 0;
float gyro_bias_y = 0;
float gyro_bias_z = 0;

// --- HÀM HIỆU CHUẨN (CALIBRATION) ---
void calibrateIMU() {
    Serial.println("\n[BƯỚC 1] BẮT ĐẦU HIỆU CHUẨN...");
    Serial.println(">> CẢNH BÁO: ĐỂ ROBOT ĐỨNG YÊN TUYỆT ĐỐI TRÊN MẶT PHẲNG!");
    
    delay(3000); // Đợi 3 giây để người dùng bỏ tay khỏi robot

    float sum_x = 0, sum_y = 0, sum_z = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        imu_msg = imu.getData();
        sum_x += imu_msg.angular_velocity.x;
        sum_y += imu_msg.angular_velocity.y;
        sum_z += imu_msg.angular_velocity.z;

        if (i % 100 == 0) Serial.print("."); // Hiển thị tiến độ
        delay(5);
    }

    // Tính toán sai số trung bình (Bias)
    gyro_bias_x = sum_x / CALIBRATION_SAMPLES;
    gyro_bias_y = sum_y / CALIBRATION_SAMPLES;
    gyro_bias_z = sum_z / CALIBRATION_SAMPLES;

    Serial.println("\n\n[BƯỚC 2] KẾT QUẢ HIỆU CHUẨN THÀNH CÔNG!");
    Serial.println("-------------------------------------------");
    Serial.printf("Offset X: %10.6f\n", gyro_bias_x);
    Serial.printf("Offset Y: %10.6f\n", gyro_bias_y);
    Serial.printf("Offset Z: %10.6f (Quan trọng nhất)\n", gyro_bias_z);
    Serial.println("-------------------------------------------");
    Serial.println(">> Hãy lưu 3 số này lại để chèn vào code chính.");
    delay(2000);
}

void setup() {
    // Khởi tạo Serial
    Serial.begin(BAUDRATE);
    while(!Serial); // Đợi Serial mở

    // Khởi tạo I2C
#ifdef SDA_PIN 
    #ifdef ESP32
        Wire.begin(SDA_PIN, SCL_PIN);
    #else 
        Wire.setSDA(SDA_PIN);
        Wire.setSCL(SCL_PIN);
    #endif
#endif

    // Khởi tạo cảm biến
    imu.init();
    Serial.println("IMU Initialized FORCEFULLY (Skipped ID Check)");
    
    // Chạy hàm hiệu chuẩn
    calibrateIMU();
}

void loop() {
    // 1. Đọc dữ liệu thô (Raw)
    imu_msg = imu.getData();

    // 2. Trừ đi sai số đã tính được (Clean)
    float clean_gx = imu_msg.angular_velocity.x - gyro_bias_x;
    float clean_gy = imu_msg.angular_velocity.y - gyro_bias_y;
    float clean_gz = imu_msg.angular_velocity.z - gyro_bias_z;

    // 3. In kết quả so sánh để kiểm tra độ "trôi"
    // Nếu CLEAN_GZ loanh quanh mức 0.00xxx là cực đẹp
    Serial.printf("RAW_Z: %7.4f | CLEAN_Z: %7.4f | TRẠNG THÁI: %s\n", 
        imu_msg.angular_velocity.z, 
        clean_gz,
        (abs(clean_gz) < 0.001) ? "ỔN ĐỊNH" : "VẪN TRÔI"
    );

    delay(200); // Cập nhật 5 lần/giây cho dễ nhìn
}