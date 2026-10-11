# BOM phần cứng, mua tại Việt Nam

## Kết luận trước

Robot chạy nhanh tới đâu do lốp quyết định, không phải động cơ. Tôi đo bán kính cong trên hai file track của dự án: trung vị 112 mm ở vòng loại, 129 mm ở chung kết, và 79-88% chiều dài track có bán kính dưới 300 mm. Vận tốc tối đa qua cua là sqrt(μ·g·R). Lốp cao su thường (μ khoảng 0.8) chỉ cho phép chạy khoảng 1 m/s qua phần lớn track, lốp silicone (μ khoảng 1.1) cho khoảng 1.2 m/s. Động cơ khỏe hơn không đổi được con số này.

Cho nên không cần Pololu hay ProFast, hai loại đó là hàng nhập và không bán ở Việt Nam. Động cơ N20 bán khắp Shopee là đủ. Cái cần mua thật là lốp silicone, rồi vài thứ nhỏ khác.

## Bảng linh kiện

| Linh kiện | Chọn | Giá | Mua ở đâu |
|---|---|---|---|
| Động cơ ×2 | GA12-N20 12V 1000 RPM | 40-80k/chiếc | Shopee (tìm "GA12-N20 12V 1000RPM") |
| Driver | A4950, giữ nguyên | 0 | đã có |
| Bánh xe ×2 | vành in 3D Ø42 + lốp silicone, hoặc bộ bánh 43mm của Hshop bọc silicone | 0-200k | in tại xưởng / Hshop |
| Pin | 3S LiPo 300-450 mAh, giắc XT30 | 150-300k | shop RC trên Shopee |
| IMU | MPU6050 (GY-521) | 25-50k | Hshop, lkcg.vn, Shopee |
| ToF ×3 | VL53L0X, trước + trái + phải | 0 | đã có |
| Cảm biến line | giữ 8 IR hiện tại, nâng lên 16× QRE1113 nếu còn ngân sách | 0 hoặc 150-250k | đã có / Shopee |
| Khung | in 3D PETG, mục tiêu cả xe dưới 150 g | 0 | xưởng in |

Tổng tiền mua thêm: khoảng 400k đến 900k, tùy có nâng cảm biến line hay không.

## Vì sao chọn từng thứ

Động cơ GA12-N20 12V 1000 RPM là loại N20 nhanh nhất bán sẵn ở Việt Nam. 1000 RPM với bánh 42 mm cho 2.2 m/s không tải, thực tế khoảng 1.5 m/s. Số này thừa cho 80% track vì track ép tốc độ ở 1 m/s. Điện áp 12V khớp với pin 3S (11.1V danh định, đầy 12.6V), nên không phải kìm duty như khi chạy motor 6V trên pin 3S. Lưu ý khi mua: bản 1000 RPM thường không có encoder phía sau. Nếu shop bán bản có encoder thì lấy luôn, không thì giữ cụm encoder Hall cũ hoặc gắn AS5600 vào trục bánh.

Lốp silicone là món đáng tiền nhất. Nó nâng μ từ khoảng 0.7 lên khoảng 1.1, tức tăng tốc độ cua khoảng 25%. Trên track uốn lượt như này, 25% đó rải đều cả quãng đường, còn đổi động cơ chỉ giúp trên mấy đoạn thẳng ngắn. In vành rỗng rồi bọc vòng silicone, hoặc mua cặp bánh silicone của Pololu qua trung gian nếu muốn chắc. Bánh 42 mm vẫn vừa giới hạn 160 mm vì 115 mm track cộng 42 mm bằng 157 mm.

Pin 3S là mức điện áp cao nhất luật cho phép, và nó khớp motor 12V. Chọn mAh nhỏ nhất đủ dùng vì mỗi gram đều đè lên tốc độ cua.

MPU6050 cho tín hiệu góc quay để giữ thẳng khi chạy nhanh. Không có nó, robot dễ đánh võng và mất vạch khi vượt 1.5 m/s. Đây là món rẻ nhất trong danh sách.

Vật cản là hộp 105×60 mm màu bất kỳ trên nền trắng. Cảm biến hồng ngoại phản xạ loại rẻ đo độ phản xạ nên không phân biệt được hộp đen với nền trắng, phải dùng ToF đo khoảng cách thật. VL53L0X là ToF rẻ nhất bán ở Việt Nam. Ba chiếc: trước, trái, phải, cho phép chọn phía tránh theo dữ liệu thay vì cố định một bên. VL53L1X đo 4 m không cần cho cách chạy hai pass. Sửa firmware đọc ToF không chặn trước khi nghĩ tới đổi cảm biến.

A4950 giữ nguyên vì dòng đỉnh 3.5A của nó đủ cho N20 (dòng kẹt khoảng 1A). Không cần đổi driver.

Giảm khối lượng xuống dưới 150 g cũng là một cách tăng tốc độ cua, vì robot nhẹ đổi hướng nhanh hơn. Robot vô địch InfoMatrix nặng 100 g. In khung rỗng dạng gyroid, bỏ OLED nếu không cần, chọn pin nhỏ.

## Việc cần làm trước khi mua

Đo độ bám của lốp hiện tại bằng cách cho robot chạy cua tăng dần tốc độ tới khi trượt, rồi tính μ = v²/(g·R). Nếu μ đã trên 1.0 thì lốp hiện tại đủ tốt, bỏ khoản mua lốp. Nếu dưới 0.7 thì lốp là nút thắt lớn nhất.

Sau khi lắp xong, đo RPM thật của motor và cập nhật tuning trong firmware. Các shop Việt Nam ghi thông số motor rất lẫn lộn, cùng tên N20 mà có loại 100 RPM có loại 1000 RPM.

## Nguồn

GA12-N20 12V 1000 RPM trên Shopee: https://shopee.vn (tìm "GA12-N20 12V 1000RPM")
GA12-N20 có encoder và bộ kèm bánh 43mm: https://hshop.vn/dong-co-dc-giamtoc-ga12-n20-encoder
MPU6050: https://hshop.vn/cam-bien-6-dof-bac-tu-do-gy-521-mpu6050
VL53L0X/VL53L1X trên Shopee: https://shopee.vn (tìm "VL53L1X")
Robot vô địch HLR V3 dùng làm tham chiếu khối lượng và lốp silicone: https://github.com/andreipopescufilimon/HyperLine-Robotics-Line-Follower-V3
