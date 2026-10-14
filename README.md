# 🚁 UAV Quadcopter Flight Controller & Telemetry System

Hệ thống điều khiển bay (Flight Controller) và trạm thu phát dữ liệu từ xa (Ground Station) hoàn chỉnh cho máy bay không người lái (UAV Quadcopter), bao gồm đầy đủ **Phần cứng (PCB KiCad)**, **Firmware điều khiển bay (STM32CubeIDE)**, và **Firmware trạm thu dữ liệu (ESP32-C3 ESP-IDF)**.

---

## 📌 Cấu Trúc Dự Án (Repository Structure)

```text
uav_ground_station/
├── mach-UAV-main/                    # Firmware STM32CubeIDE cho máy bay
│   └── firmware/
│       ├── uav_complete_fc/          # >> Firmware điều khiển bay hoàn chỉnh (Quad Flight Controller)
│       ├── doccambien/               # Module đọc và kiểm thử cảm biến (IMU MPU6050, Khí áp, La bàn)
│       ├── DocRF/                    # Module giao tiếp vô tuyến nRF24L01+
│       └── PWM_ADC_TIMER/            # Module đọc PWM tay cầm, đo điện áp pin ADC và Timer
├── thiet_ke_mach/                    # Thiết kế phần cứng mạch PCB (KiCad 10)
│   ├── mach_uav.kicad_sch            # Sơ đồ nguyên lý (Schematic)
│   ├── mach_uav.kicad_pcb            # Mạch in hoàn chỉnh (Layout 92x92mm)
│   ├── HUONG_DAN_DI_DAY_NGOAI.md     # Sơ đồ cầu nối ngoài (W1-W6) cho mạch 1 lớp
│   └── work/review/                  # Thuật toán A* Routing tự động hóa (C++ DLL & Python pcbnew)
└── main/                             # Firmware trạm mặt đất Ground Station (ESP32-C3)
    ├── main.c                        # FreeRTOS tasks giải mã telemetry và log Serial Monitor
    ├── nrf24_rx.c / nrf24_rx.h       # Driver SPI cho nRF24L01+ ở tần số 2.480 GHz
    └── telemetry_protocol.h          # Giao thức truyền tin nhị phân 32 byte + CRC-8
```

---

## 1. Firmware Điều Khiển Bay STM32 (`mach-UAV-main/firmware/uav_complete_fc`)

Dự án phát triển trên **STM32CubeIDE** cho vi điều khiển **STM32F103C8T6 (ARM Cortex-M3 @ 72 MHz HSE)**:

- **Chế độ bay (Flight Mode):** Cố định chế độ cân bằng góc (`ANGLE Mode`) kết hợp tự động bù góc nghiêng.
- **Vòng lặp điều khiển PID:** Bộ điều khiển PID độc lập cho 3 trục Roll, Pitch, Yaw với:
  - Tích hợp bộ lọc đạo hàm (`d_filter`).
  - Khử bão hòa tích phân (Anti-windup Integrator Limit).
  - Giới hạn ngõ ra an toàn (`output_limit`).
- **Xử lý cảm biến góc nghiêng & La bàn (GY-87):**
  - Giao tiếp cảm biến `MPU6050` (Gyro + Accel) qua **Hardware I2C1 (`PB6/PB7`)** được cấu hình thanh ghi trực tiếp, tránh trễ và lỗi bus của thư viện HAL.
  - Tích hợp la bàn số `HMC5883L` sửa trôi góc Yaw theo thời gian thực (loại bỏ nhiễu từ trường khi dòng tải động cơ thay đổi đột ngột).
- **Giữ hướng bay (Heading Hold):** Khóa hướng tham chiếu (`REF`) bằng tích phân con quay hồi chuyển, duy trì mũi máy bay ổn định khi buông cần lái Yaw.
- **Đọc tín hiệu điều khiển RC (PWM Input Capture):**
  - 4 kênh PWM trên Timer `TIM3` (Roll: `PA6`, Pitch: `PA7`, Throttle: `PB0`, Yaw: `PB1`).
  - Khóa/Mở khóa động cơ bằng tổ hợp cần lái an toàn:
    - **Arm:** `Ga thấp + Gạt hết Yaw sang Phải trong 3s`.
    - **Disarm:** `Ga thấp + Gạt hết Yaw sang Trái trong 3s`.
- **Ngõ ra điều khiển động cơ (ESC Outputs):**
  - 4 kênh xung PWM độc lập (1000 - 2000 µs) trên Timer `TIM2` (`PA0`, `PA1`, `PA2`, `PA3`) bố trí theo layout quad X (`M1-M4`).
- **Cơ chế an toàn Failsafe đa cấp (`FailsafeMode`):**
  - Tự động phát hiện mất sóng vô tuyến hoặc mất xung PWM máy thu: Chuyển sang giữ thăng bằng (`HOLD`) $\rightarrow$ hạ độ cao từ từ (`DESCEND`) $\rightarrow$ phục hồi nếu có lại sóng (`RECOVER`).
  - Tự động ngắt khẩn cấp (Emergency Disarm) nếu cảm biến IMU bị lỗi liên tục.

---

## 2. Thiết Kế Mạch PCB Flight Controller (`thiet_ke_mach/`)

- **Kích thước & Thiết kế:** Bo mạch tiêu chuẩn **92 × 92 mm**, thiết kế trên **KiCad 10**.
- **Khối nguồn:** Bộ nguồn xung buck converter **MP1584EN** hạ áp từ pin LiPo cấp dòng khỏe **5V BEC**, kết hợp LDO **3.3V** riêng cho cảm biến và radio để chống sụt áp khi 4 động cơ hoạt động hết công suất.
- **Tối ưu hóa mạch 1 lớp & Thuật toán A* Pathfinder:**
  - Giải pháp mạch 1 lớp (`B.Cu`) không dùng via xuyên tầng, phục vụ gia công CNC tại chỗ.
  - Xây dựng **thuật toán tìm đường A\* bằng C++** ([`pathfinder.cpp`](file:///C:/uav_ground_station/thiet_ke_mach/work/review/pathfinder.cpp)) biên dịch sang DLL kết hợp script Python `pcbnew` tự động định tuyến đường đồng theo kích thước an toàn (Power: 30–35 mil, Signal: 14 mil, Clearance: 0.20 mm).
  - Định tuyến tối ưu 6 cầu nối ngoài (**W1 – W6**) có in nhãn rõ ràng trên lớp lụa `F.SilkS`.
  - Phủ đồng mass `GND` diện tích lớn giúp tản nhiệt và chống nhiễu vô tuyến.

---

## 3. Firmware Trạm Thu Mặt Đất ESP32-C3 (`main/`)

- **Framework & RTOS:** Viết bằng native **ESP-IDF (v5.x/v6.x)** chạy trên nhân RISC-V của **ESP32-C3** với hệ điều hành thời gian thực **FreeRTOS**.
- **Giao thức Telemetry tốc độ cao (100 Hz):**
  - Gói tin nhị phân định dạng cố định **32 byte**, bảo vệ toàn vẹn bằng **CRC-8** (đa thức $0x07$).
  - Truyền nhận các khung: Góc nghiêng Roll/Pitch/Heading (`0xA1`), Vị trí cần lái RC (`0xA2`), Trạng thái vi điều khiển & chu kỳ vòng lặp PID (`0xA3`), Cân chỉnh la bàn (`0xA4`), Cấu hình PID (`0xA5`).
- **Driver vô tuyến nRF24L01+:**
  - Tần số **2.480 GHz (Kênh 80)**, tốc độ 1 Mbps, tránh hoàn toàn dải tần WiFi gia đình để thu tín hiệu ổn định.
- **Hiển thị dữ liệu qua Serial Monitor:**
  - Xuất log trực quan chu kỳ 1 giây: Tần số nhận gói (Hz), tỷ lệ mất gói (loss count), chất lượng link (%), trạng thái Arm/Failsafe, độ rộng xung 4 động cơ và góc nghiêng máy bay.

---

## 4. Hướng Dẫn Biên Dịch & Nạp Code

### 4.1 Nạp Firmware STM32 Flight Controller:
1. Mở phần mềm **STM32CubeIDE**.
2. Chọn `File` $\rightarrow$ `Open Projects from File System...` $\rightarrow$ trỏ đến thư mục `mach-UAV-main/firmware/uav_complete_fc`.
3. Kết nối mạch STM32 qua mạch nạp ST-Link (SWDIO `PA13`, SWCLK `PA14`).
4. Bấm **Build** (búa) và **Run / Flash** để nạp chương trình.

### 4.2 Nạp Firmware ESP32-C3 Ground Station:
```bash
# Đứng tại thư mục gốc uav_ground_station
idf.py set-target esp32c3
idf.py build
idf.py -p COMx flash monitor
```

---

## 5. Kỹ Năng Kỹ Thuật Đạt Được (Portfolio Highlights)

- **Lập trình Vi Điều Khiển & Nhúng Chuyên Sâu:** C/C++, ARM Cortex-M (STM32F103) trần (Bare-metal) & HAL, ESP32-C3 RISC-V với FreeRTOS.
- **Hệ Thống Điều Khiển Tự Động (Control Systems):** Thiết kế bộ điều khiển PID góc nghiêng máy bay, bộ lọc số (Digital Filter), thuật toán khóa hướng Heading Hold, xử lý an toàn Failsafe.
- **Thiết Kế Phần Cứng & PCB:** Thiết kế mạch nguyên lý, phân tách vùng nguồn số/tương tự/động lực, tính toán tiết diện đường nguồn công suất cao, thiết kế footprint trên KiCad 10.
- **Thuật Toán Tối Ưu Hóa (Algorithms):** Xây dựng thuật toán định tuyến tự động A* bằng C++ DLL kết hợp Python Scripting (`pcbnew`) trên hình học lưới PCB.
- **Truyền Thông Không Dây (RF Communications):** Giao thức truyền tin nhị phân tốc độ cao qua sóng 2.4 GHz nRF24L01+, kiểm tra toàn vẹn CRC-8.
