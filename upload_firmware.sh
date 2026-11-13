#!/bin/bash
# Script để nạp code cho ESP32 (Firmware chính của robot)
# Đã tắt cấu hình IMU trong config/custom/myrobot_config.h

echo "Đang chuyển đến thư mục firmware..."
cd "$(dirname "$0")/firmware" || exit

echo "Bắt đầu nạp code cho ESP32..."
pio run -e myrobot -t upload

echo "Mở monitor theo dõi log (Ctrl+C để thoát)..."
pio device monitor -b 115200
