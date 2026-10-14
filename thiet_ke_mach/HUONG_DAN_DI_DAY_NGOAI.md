# PCB flight controller — các dây cần hàn ngoài

File chính: `mach_uav.kicad_pcb` (KiCad 10.0.6). Bo 92 × 92 mm. Toàn bộ track nằm trên **B.Cu**; không có via, track GND hoặc vùng phủ đồng. Các connector nằm quanh mép bo. NRF J32 có chân 2/4/6/8 hướng ra mép phải và 1/3/5/7 hướng vào trong.

**Net 3.3 V đã nối đầy đủ trên PCB.** KiCad báo 0 vi phạm DRC, 0 sai khác sơ đồ. Còn 18 mục hở: 13 nhánh mass để phủ sau và 5 nhánh tín hiệu/nguồn sẽ hàn dây ngoài. Ngoài 5 nhánh đó, W1 cũng cần hàn: footprint W1 đánh dấu cầu dây lắp ráp nên KiCad không tính là nhánh hở.

Bảng **W1–W6 được in trên F.SilkS của PCB**, ở cạnh dưới bo, để xác định net và chân hàn:

| Dây | Net | Đầu 1 | Đầu 2 |
|---|---|---|---|
| W1 | SPI1_MOSI | U3 chân 33 | J32 chân 6 |
| W2 | 5V_BEC | J1 chân 2 | U3 chân 38 |
| W3 | USART1_RX | J18 chân 2 | U3 chân 27 |
| W4 | USART1_TX | J18 chân 3 | U3 chân 26 |
| W5 | USART3_RX | U3 chân 16 | Lỗ **W5B**, đã nối B.Cu đến J26 chân 2 |
| W6 | USART3_TX | U3 chân 15 | Lỗ **W6B**, đã nối B.Cu đến J26 chân 1 |

Các đầu U3/J1/J18/J32 là lỗ xuyên có sẵn. W5B và W6B là lỗ xuyên bổ sung gần J26, với **lỗ khoan 0,5 mm và vòng đồng 0,8 mm**, bằng cỡ chân connector. Hàn dây bọc cách điện theo đúng từng hàng, rồi đo thông mạch và kiểm tra chạm chập.

J26 GPS là connector SMD đặt ở **mặt B**. Kiểm tra hướng lắp này trước khi hàn. Chân GPS USART3 đã được đưa đến W5B/W6B để không phải hàn dây trực tiếp vào pad SMD 1 mm.

Khoảng 82,5% chiều dài đường đồng rộng ít nhất 0,762 mm (30 mil), phần lớn là 0,889 mm (35 mil). Các đoạn thoát chân có thể hẹp đến 0,35 mm; riêng tuyến 5V_BEC1 còn khoảng 57,7 mm đường 0,5 mm. **Chưa xác nhận tiết diện đường nguồn theo dòng tải thực tế.**

Khi phủ mass, chọn net **G** và giữ clearance vùng phủ theo project ở **0,5 mm**. Sau khi phủ, chạy lại DRC và xem các nhánh G đã nối đúng trước khi xuất Gerber.
