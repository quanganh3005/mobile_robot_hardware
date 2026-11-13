# Hướng dẫn Tune PID 2 bánh Linorobot2 trên sàn qua Wi-Fi

## 1. Nạp code vào ESP32
Mở terminal và chạy lệnh:
```bash
cd ~/my_linorobot2_hardware/pid_tune_web
pio run -e myrobot -t upload
```

## 2. Kết nối điều khiển bằng điện thoại / Laptop
ESP32 hỗ trợ cả 2 chế độ Wi-Fi:

* **Cách 1 (Tiện nhất khi chạy sàn):**
  1. Dùng điện thoại tìm mạng Wi-Fi tên: **`Linorobot2_PID_Tune`**
  2. Mật khẩu: **`12345678`**
  3. Mở trình duyệt web trên điện thoại gõ: **`http://192.168.4.1`**

* **Cách 2 (Mạng nội bộ Lab):**
  1. Nếu ESP32 vào được Wi-Fi `M625 Mechatronic Lab`, mở Serial Monitor (115200) để xem IP (ví dụ: `192.168.1.xxx`).
  2. Truy cập: `http://<IP_ESP32>`

---

## 3. Các bước tune PID khi có tải (1.3 kg dưới sàn)

1. **Đặt xe trên sàn trống:** Chọn khu vực sàn phẳng, không có chướng ngại vật phía trước.
2. **Khởi đầu với Step Test an toàn:**
   * Chọn tốc độ: khoảng **60 - 80 RPM** trên thanh trượt.
   * Bấm nút **`TEST BƯỚC 2 GIÂY`** hoặc **`TEST BƯỚC 4 GIÂY`**: Xe sẽ tự động chạy tiến đúng 2s (hoặc 4s) rồi tự dừng hẳn $\rightarrow$ hoàn toàn không lo xe đâm vào tường.
3. **Quan sát đồ thị thời gian thực:**
   * **Đường Vàng:** Tốc độ đặt mong muốn.
   * **Đường Xanh lá:** Tốc độ bánh trái.
   * **Đường Xanh lơ:** Tốc độ bánh phải.
4. **Quy tắc tinh chỉnh:**
   * **Nếu xe khởi động bị ỳ / không chạy nổi:** Tăng dần **$K_p$** (ví dụ từ `0.8` lên `1.2` $\rightarrow$ `1.8` $\rightarrow$ `2.2`).
   * **Nếu 2 bánh bám tốt nhưng xe vẫn chạy chậm hơn số đặt:** Tăng nhẹ **$K_i$** (từ `3.0` lên `3.5` $\rightarrow$ `4.5`).
   * **Nếu xe bị giật rung hoặc vọt lố:** Giảm bớt $K_i$ hoặc tăng nhẹ $K_d$ (`0.02` $\rightarrow$ `0.04`).
   * **Nếu 2 đường xanh lá và xanh lơ đè sát đường màu vàng và xe chạy thẳng tắp:** Đạt kết quả tối ưu!
5. **Lưu kết quả vào firmware chính:**
   * Khi tìm được bộ số ưng ý trên Web, ghi lại 3 số $K_p, K_i, K_d$ đó.
   * Điền vào file [`config/custom/myrobot_config.h`](file:///home/picasso/my_linorobot2_hardware/config/custom/myrobot_config.h).
   * Nạp lại firmware chính: `cd ~/my_linorobot2_hardware/firmware && pio run -e myrobot -t upload`.

