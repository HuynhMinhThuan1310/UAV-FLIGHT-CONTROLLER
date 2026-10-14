# 🚁 UAV Flight Controller & Real-Time Telemetry Ground Station

Hệ thống điều khiển bay và trạm thu phát dữ liệu từ xa (Telemetry Ground Station) hoàn chỉnh cho máy bay không người lái (UAV / Drone), bao gồm:
1. **Phần Cứng (Hardware / PCB):** Bo mạch Flight Controller tự thiết kế trên KiCad dựa trên vi điều khiển STM32F103, tích hợp cảm biến IMU, Barometer, La bàn, GPS, điều khiển 4 ESC động cơ, và bộ chuyển nguồn BEC hiệu suất cao. Đặc biệt có bộ công cụ tự động định tuyến mạch 1 lớp (Single-Layer Routing) bằng thuật toán A* (C++ / Python).
2. **Phần Mềm (Firmware & Ground Station):** Trạm mặt đất nhúng trên vi điều khiển **ESP32-C3** chạy **ESP-IDF v5/v6** & **FreeRTOS**, kết nối module RF **nRF24L01+** tốc độ cao (100 Hz), phát **WiFi SoftAP** nội bộ và cung cấp **Web Dashboard thời gian thực** (chân trời nhân tạo, la bàn số, đo xung động cơ, vị trí cần điều khiển) mà không cần mạng Internet.

---

## 📌 Mục Lục
- [1. Kiến Trúc Tổng Thể](#1-kiến-trúc-tổng-thể)
- [2. Thiết Kế Mạch PCB Flight Controller](#2-thiết-kế-mạch-pcb-flight-controller)
  - [2.1 Sơ đồ nguyên lý & Ngoại vi](#21-sơ-đồ-nguyên-lý--ngoại-vi)
  - [2.2 Tối ưu hóa mạch 1 lớp & Thuật toán A* Routing](#22-tối-ưu-hóa-mạch-1-lớp--thuật-toán-a-routing)
  - [2.3 Hướng dẫn cầu nối và đi dây ngoài](#23-hướng-dẫn-cầu-nối-và-đi-dây-ngoài)
- [3. Phần Mềm Trạm Mặt Đất (ESP32-C3 Ground Station)](#3-phần-mềm-trạm-mặt-đất-esp32-c3-ground-station)
  - [3.1 Driver nRF24L01+ & RF 2.480 GHz](#31-driver-nrf24l01--rf-2480-ghz)
  - [3.2 Giao thức Telemetry nhị phân (Binary Protocol)](#32-giao-thức-telemetry-nhị-phân-binary-protocol)
  - [3.3 Kiến trúc đa luồng FreeRTOS](#33-kiến-trúc-đa-luồng-freertos)
  - [3.4 Web Dashboard thời gian thực](#34-web-dashboard-thời-gian-thực)
- [4. Sơ Đồ Nối Dây ESP32-C3 và nRF24L01](#4-sơ-đồ-nối-dây-esp32-c3-và-nrf24l01)
- [5. Hướng Dẫn Biên Dịch & Nạp Code](#5-hướng-dẫn-biên-dịch--nạp-code)
- [6. Kỹ Năng Kỹ Thuật Đạt Được (Portfolio Highlights)](#6-kỹ-năng-kỹ-thuật-đạt-được-portfolio-highlights)

---

## 1. Kiến Trúc Tổng Thể

```
+-------------------------------------------------------------+
|              UAV FLIGHT CONTROLLER (STM32F103)              |
|  - Cảm biến IMU (MPU6050), Khí áp Baro, La bàn số, GPS      |
|  - Điều khiển 4 ESC động cơ không chổi than (PWM 1-2 ms)    |
|  - Module nRF24L01+ truyền Telemetry 100 Hz (Kênh 80)       |
+------------------------------+------------------------------+
                               |
                   Sóng RF 2.480 GHz (1 Mbps)
                   Giao thức nhị phân 32 byte + CRC-8
                               |
                               v
+-------------------------------------------------------------+
|          GROUND STATION (ESP32-C3 RISC-V SoC)              |
|  - FreeRTOS Task 1: Thu nhận SPI FIFO & Kiểm tra CRC-8       |
|  - FreeRTOS Task 2: Thống kê tần số gói (Hz), Link Quality  |
|  - WiFi SoftAP: Phát mạng "UAV-GroundStation"               |
|  - HTTP Web Server: Phục vụ Web Dashboard & REST API         |
+------------------------------+------------------------------+
                               |
                    WiFi nội bộ 192.168.4.1
                               |
                               v
+-------------------------------------------------------------+
|            CLIENT (Smartphone / Tablet / PC Browser)        |
|  - Chân trời nhân tạo (Artificial Horizon) HTML5 Canvas     |
|  - La bàn số Heading, Đo 4 kênh động cơ ESC, RC Sticks      |
|  - Giám sát trạng thái Armed / Failsafe tức thời            |
+-------------------------------------------------------------+
```

---

## 2. Thiết Kế Mạch PCB Flight Controller

Thư mục: `mach_flightcontroller (1) (1)/`

### 2.1 Sơ đồ nguyên lý & Ngoại vi
Bo mạch điều khiển bay được thiết kế trên **KiCad 10** với kích thước tiêu chuẩn **92 × 92 mm**, hỗ trợ đầy đủ các khối chức năng phục vụ bay tự cân bằng:
- **MCU Trung tâm (`U3`):** STM32F103C8T6 (BluePill) chạy vòng lặp điều khiển PID tốc độ cao.
- **Khối Quản lý Nguồn:** Tích hợp module chuyển nguồn hạ áp xung **MP1584EN** (Step-down Buck Converter) chuyển đổi từ pin LiPo 2S–4S sang nguồn sạch **5V BEC**, cùng mạch lọc LDO **3.3V** riêng biệt cho cảm biến và radio RF.
- **Cụm Cảm biến:**
  - `J21` (IMU): MPU6050 (3-axis Gyroscope + 3-axis Accelerometer).
  - `J22` (Barometer): Cảm biến áp suất khí quyển đo độ cao tuyệt đối.
  - `J20` (I2C): Giao tiếp cảm biến từ trường / la bàn số magnetometer.
- **Cụm Định vị & Điều hướng:** `J26` kết nối GPS module (hỗ trợ footprint SMD pad 1.0mm và header chuyển tiếp).
- **Cụm Động lực:** `J8`, `J15`, `J16`, `J17` (ESC1 - ESC4) xuất tín hiệu xung PWM (1000 - 2000 µs) điều khiển 4 động cơ không chổi than.
- **Cụm Thu/Phát:** `J32` (nRF24L01+) truyền nhận dữ liệu tầm xa; `J1`, `J2` nhận tín hiệu tay cầm điều khiển (RC Receiver).
- **Cổng Giao tiếp & Gỡ lỗi:** `J18` (USART1 Debug/Telemetry), `J27` (Servo phụ).

### 2.2 Tối ưu hóa mạch 1 lớp & Thuật toán A* Routing
Điểm đột phá của dự án là giải pháp thiết kế mạch **1 lớp (Single-Layer PCB)** trên lớp mặt dưới (`B.Cu`), giúp chế tạo mạch nhanh bằng phương pháp phay CNC hoặc ăn mòn hóa học tại xưởng:
- **Thuật toán A* Pathfinder (`pathfinder.cpp`):** Được viết bằng C++ với độ phức tạp tối ưu, hỗ trợ heuristic khoảng cách Chebyshev/Euclid trên lưới đồ thị rời rạc, xuất thành thư viện động DLL để Python gọi trực tiếp.
- **Tập lệnh Python Tự động hóa (`route_flightcontroller.py`):** Sử dụng API `pcbnew` của KiCad để đọc linh kiện, tạo vùng cản (obstacle grid) dựa trên clearance cách ly 0.20 mm, tự động thương lượng độ rộng đường mạch (Power trace 30–35 mil, Signal 14 mil) và tìm đường đi ngắn nhất không chạm chập.
- **0 Via, Phủ Đồng Mass Toàn Diện:** Toàn bộ đường tín hiệu đi trên `B.Cu`, để dành khoảng trống tối đa phủ mặt phẳng đất GND (`G`) giúp giảm nhiễu vô tuyến.

### 2.3 Hướng dẫn cầu nối và đi dây ngoài
Để đạt được mạch 1 lớp không qua via, 6 đường tín hiệu/nguồn giao cắt được tối ưu thành 6 cầu nối ngoài (**W1 – W6**), có in sẵn nhãn chỉ dẫn trên lớp lụa (`F.SilkS`):
- **W1:** `SPI1_MOSI` (U3 chân 33 $\rightarrow$ J32 chân 6)
- **W2:** `5V_BEC` (J1 chân 2 $\rightarrow$ U3 chân 38)
- **W3:** `USART1_RX` (J18 chân 2 $\rightarrow$ U3 chân 27)
- **W4:** `USART1_TX` (J18 chân 3 $\rightarrow$ U3 chân 26)
- **W5:** `USART3_RX` (U3 chân 16 $\rightarrow$ Lỗ xuyên W5B đến J26 chân 2)
- **W6:** `USART3_TX` (U3 chân 15 $\rightarrow$ Lỗ xuyên W6B đến J26 chân 1)

---

## 3. Phần Mềm Trạm Mặt Đất (ESP32-C3 Ground Station)

Thư mục: `main/`

### 3.1 Driver nRF24L01+ & RF 2.480 GHz
- Giao tiếp qua **SPI phần cứng** với tần số xung nhịp 2 MHz đảm bảo độ ổn định cao với module anten rời PA/LNA.
- Cấu hình ở chế độ **PRX (Primary Receiver)**, địa chỉ nhận 5 byte `{ 0x6B, 0xE3, 0x1C, 0x95, 0xA2 }`.
- Hoạt động tại **Kênh 80 (2480 MHz)**: Tần số này nằm ngoài phổ các kênh WiFi 1–11 thông thường, hạn chế tối đa hiện tượng tự can nhiễu giữa bộ phát WiFi của ESP32 và module nRF24L01+.

### 3.2 Giao thức Telemetry nhị phân (Binary Protocol)
Gói tin có kích thước cố định **32 byte**, thiết kế tối ưu cho nRF24L01 payload:
- **Byte 0:** Định danh loại khung (Frame Type: `0xA1` .. `0xA5`).
- **Byte 1:** Số thứ tự gói (`seq`) dùng để phát hiện gói tin bị mất: $\Delta \text{loss} = \text{seq} - \text{last\_seq} - 1$.
- **Byte 2..30:** Dữ liệu cảm biến và trạng thái bay (Little-endian).
- **Byte 31:** Mã kiểm tra toàn vẹn **CRC-8** (Đa thức $x^8 + x^2 + x + 1$, Poly `0x07`).

#### Các khung truyền dữ liệu:
| Mã khung | Tên khung | Tần số | Nội dung dữ liệu |
| :---: | :---: | :---: | :--- |
| `0xA1` | **ATTITUDE** | 50 Hz | Góc nghiêng Roll, Pitch, Heading (0.01°), Vận tốc góc Gyro 3 trục, Giá trị PID, Xung 4 động cơ ESC (µs), cờ Armed/Failsafe |
| `0xA2` | **RC STICKS** | 25 Hz | Giá trị thô và giá trị chuẩn hóa cần lái RC, Xung điều khiển Servo |
| `0xA3` | **STATUS** | 10 Hz | Uptime FC, Trạng thái cảm biến IMU/Mag, Chu kỳ vòng lặp PID (`loop_dt_us`), Chế độ Failsafe |
| `0xA4` | **TUNE/MAG** | 10 Hz | Dữ liệu từ trường 3 trục, Góc la bàn số (Compass Heading), Dynamic Yaw tuning |
| `0xA5` | **CONFIG** | 5 Hz | Cấu hình tâm RC và bộ hệ số PID (Kp, Ki, Kd) của các trục Roll, Pitch, Yaw |

### 3.3 Kiến trúc đa luồng FreeRTOS
- **`telemetry_rx_task` (Priority 5, 4KB Stack):** Quét bộ đệm RX FIFO của nRF24, kiểm tra CRC-8, giải mã cấu trúc dữ liệu nhị phân và cập nhật vào `GroundStationState` được bảo vệ bởi FreeRTOS Mutex.
- **`telemetry_stats_task` (Priority 1, 3KB Stack):** Chu kỳ 1 giây, tính toán tần số nhận gói thực tế (Hz), tỷ lệ mất gói (Packet Loss Rate), chất lượng đường truyền sóng (Link Quality %) và giám sát timeout mất kết nối (Failsafe Timeout 600 ms).
- **`httpd_task` (Priority 5, 8KB Stack):** Xử lý yêu cầu HTTP GET, trả về trang giao diện và API `/api/telemetry`.

### 3.4 Web Dashboard thời gian thực
- Toàn bộ giao diện được nhúng trực tiếp vào Flash của ESP32-C3 dưới dạng chuỗi HTML/CSS/JS thuần (`web_dashboard.h`), **không phụ thuộc bất kỳ thư viện ngoài nào (Zero-dependency)**, hoạt động trơn tru ngay cả khi không có kết nối Internet.
- **Chân trời nhân tạo (Artificial Horizon):** Vẽ bằng HTML5 Canvas, chuyển động mượt mà theo góc Roll và Pitch truyền về từ cảm biến IMU.
- **Thanh đo động cơ (Motor Output Bars):** Hiển thị trực quan độ rộng xung 4 kênh ESC (1000 µs - 2000 µs).
- **Vị trí cần gạt RC (Stick Visualizer):** Mô phỏng tọa độ 2 cần điều khiển theo thời gian thực.
- **REST API:** Đường dẫn `http://192.168.4.1/api/telemetry` trả về dữ liệu chuẩn JSON cho phép tích hợp với các ứng dụng Ground Station nâng cao (Mission Planner, Python telemetry logger,...).

---

## 4. Sơ Đồ Nối Dây ESP32-C3 và nRF24L01

| Chân nRF24L01 | Ký hiệu | Nối với ESP32-C3 | Mô tả chức năng |
| :---: | :---: | :---: | :--- |
| **Pin 1** | **GND** | **GND** | Mass chung |
| **Pin 2** | **VCC** | **3.3V** | Nguồn 3.3V sạch *(khuyến nghị hàn thêm tụ 10–100 µF)* |
| **Pin 3** | **CE** | **GPIO 3** | Chip Enable (kích hoạt thu sóng) |
| **Pin 4** | **CSN** | **GPIO 7** | SPI Chip Select (Active Low) |
| **Pin 5** | **SCK** | **GPIO 4** | SPI Clock |
| **Pin 6** | **MOSI** | **GPIO 6** | SPI Master Out Slave In |
| **Pin 7** | **MISO** | **GPIO 5** | SPI Master In Slave Out |
| **Pin 8** | **IRQ** | *(Không nối)* | Trạng thái ngắt đọc trực tiếp qua thanh ghi SPI |

---

## 5. Hướng Dẫn Biên Dịch & Nạp Code

### Yêu cầu môi trường:
- **ESP-IDF** phiên bản v5.x hoặc v6.x (đã kiểm thử tương thích).
- Kit phát triển **ESP32-C3** (SuperMini hoặc DevKitC).

### Các bước thực hiện:
```bash
# 1. Cấu hình target chip ESP32-C3
idf.py set-target esp32c3

# 2. Biên dịch firmware
idf.py build

# 3. Nạp code và mở cổng Serial theo dõi log (thay COMx bằng cổng của bạn)
idf.py -p COMx flash monitor
```

### Kết nối Web Dashboard:
1. Dùng điện thoại hoặc máy tính quét mạng WiFi:
   - **Tên WiFi (SSID):** `UAV-GroundStation`
   - **Mật khẩu:** `uavtelemetry`
2. Mở trình duyệt web và truy cập địa chỉ:
   ```text
   http://192.168.4.1
   ```

---

## 6. Kỹ Năng Kỹ Thuật Đạt Được (Portfolio Highlights)

- **Thiết kế Phần Cứng & PCB:** Thiết kế mạch nguyên lý (Schematic) và layout mạch in (PCB) nhiều lớp/một lớp trên KiCad, tính toán tiết diện đường nguồn, khử nhiễu tín hiệu RF, tối ưu hóa điểm tiếp xúc mass.
- **Thuật Toán Tự Động Hóa:** Áp dụng thuật toán tìm đường A* (A-Star) viết bằng C++ và tích hợp qua Python `pcbnew` API để giải bài toán định tuyến mạch in tự động.
- **Lập Trình Vi Điều Khiển & Hệ Điều Hành Thời Gian Thực:** Lập trình C trên nền ESP-IDF, phân luồng đa nhiệm với FreeRTOS (Task Priority, Mutex, Timer, ISR safe).
- **Giao Tiếp Vô Tuyến RF:** Giao tiếp SPI tốc độ cao với nRF24L01+, cấu hình thanh ghi RF, nhảy kênh tránh can nhiễu WiFi, tối ưu hóa kích thước khung truyền và kiểm lỗi CRC-8.
- **Phát Triển Web Nhúng (Embedded Web):** Xây dựng SoftAP, HTTP Server nhúng, thiết kế UI/UX Dashboard trực quan với HTML5 Canvas và REST API phục vụ telemetry thời gian thực.
