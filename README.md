Omni Robot 3 Bánh PID Controller
Giới thiệu
Dự án này xây dựng mô hình động lực học, động học và thiết kế bộ điều khiển robot Omni 3 bánh sử dụng vi điều khiển Arduino Mega. Robot có khả năng di chuyển đa hướng trên mặt phẳng nhờ 3 bánh omni được bố trí lệch nhau 120 độ.
Hệ thống sử dụng encoder 2 kênh A/B để đo tốc độ từng động cơ, từ đó tính toán vị trí robot theo phương pháp odometry. Bộ điều khiển được thiết kế theo cấu trúc PID tầng, gồm PID vị trí và PID tốc độ, giúp robot bám theo quỹ đạo đường thẳng hoặc di chuyển đến tọa độ đặt trước.
Dự án phù hợp cho nghiên cứu mô hình động học, điều khiển chuyển động và thiết kế bộ điều khiển PID cho robot di động holonomic.
Chức năng
•	Điều khiển robot omni 3 bánh di chuyển trên mặt phẳng 2D.
•	Đọc encoder 2 kênh A/B cho 3 động cơ JGB37-545.
•	Tính tốc độ góc thực tế của từng bánh xe.
•	Tính toán tọa độ robot x, y, theta bằng odometry.
•	Thiết kế bộ điều khiển PID vị trí cho các trục: x, y, theta
•	Thiết kế bộ điều khiển PID tốc độ cho từng bánh xe
•	Hỗ trợ điều khiển robot bằng lệnh Serial.
•	Hỗ trợ các chế độ di chuyển:
o	Đi thẳng theo quãng đường(hoặc đường tròn) và góc đặt trước.
o	Đi tới tọa độ đích.
o	Dừng robot.
o	Reset tọa độ và encoder.
o	In trạng thái robot.
o	In giá trị encoder.
Công nghệ sử dụng
•	Xây dựng mô hình động học và động lực học trên matlab/simulimk
•	Ngôn ngữ lập trình C/C++ trên Arduino IDE
•	Vi điều khiển Arduino Mega 2560
•	Điều khiển PID
•	Động học thuận robot omni 3 bánh
•	Động học nghịch robot omni 3 bánh
•	Encoder quadrature A/B
•	Ngắt ngoài attachInterrupt()
•	Giao tiếp Serial UART
Phần cứng
Thiết bị	Mô tả
Arduino Mega 2560	Vi điều khiển trung tâm
3 động cơ JGB37-545 12V	Động cơ DC có encoder
3 bánh xe Omni	Bánh xe cho phép robot di chuyển đa hướng
Driver động cơ DC	Điều khiển chiều quay và tốc độ động cơ
Encoder A/B	Đo tốc độ và chiều quay từng động cơ
Nguồn 12V	Cấp nguồn cho động cơ
Khung robot 3 bánh	Cơ khí robot omni
Sơ đồ chân
Chân điều khiển động cơ
Motor	PWM	IN1	IN2
Motor 1	9	22	23
Motor 2	10	24	25
Motor 3	11	26	27
Chân encoder
Motor	Encoder A	Encoder B
Motor 1	19	18
Motor 2	20	2
Motor 3	21	3
Nguyên lý điều khiển
Hệ thống điều khiển được thiết kế theo cấu trúc điều khiển tầng.
Vòng ngoài: PID vị trí
PID vị trí nhận tọa độ đặt xRef, yRef, thetaRef và so sánh với tọa độ thực tế x, y, theta.
Kết quả của PID vị trí là vận tốc đặt của robot: vxWorldRef, vyWorldRef, omegaRef
Các bộ PID vị trí gồm: pidX, pidY, pidTheta
Chuyển đổi động học
Sau khi có vận tốc đặt trong hệ tọa độ toàn cục, chương trình chuyển đổi sang hệ tọa độ thân robot: worldToBody()
Sau đó sử dụng động học nghịch để tính tốc độ đặt cho từng bánh:inverseKinematic()
Kết quả thu được: wheelRef[0], wheelRef[1], wheelRef[2]
Vòng trong: PID tốc độ
PID tốc độ so sánh tốc độ đặt của từng bánh với tốc độ thực tế đo được từ encoder.
Các bộ PID tốc độ gồm: pidW[0], pidW[1], pidW[2]
Kết quả của PID tốc độ là tín hiệu PWM điều khiển 3 động cơ: pwmOut[0], pwmOut[1], pwmOut[2]
Hướng dẫn chạy
Bước 1: Chuẩn bị phần cứng
Kết nối phần cứng theo sơ đồ chân đã khai báo trong code:
•	Kết nối 3 động cơ với driver.
•	Kết nối chân PWM và chân điều khiển chiều từ Arduino Mega tới driver.
•	Kết nối encoder A/B của từng động cơ về Arduino Mega.
•	Cấp nguồn 12V cho động cơ.
•	Kết nối Arduino Mega với máy tính qua USB.
Bước 2: Mở code bằng Arduino IDE
Mở file: Omni3_PID_Mega_AB_Encoder.ino
Chọn board: Arduino Mega or Mega 2560
Chọn đúng cổng COM của Arduino.
Bước 3: Nạp chương trình
Nhấn nút Upload trong Arduino IDE để nạp chương trình vào Arduino Mega.
Bước 4: Mở Serial Monitor
Mở Serial Monitor và cấu hình:
Baud rate: 9600
Line ending: Newline
Bước 5: Gửi lệnh điều khiển
Một số lệnh hỗ trợ:
Lệnh	Chức năng
START	Chạy quỹ đạo thẳng mặc định
LINE 3.0 0	Đi thẳng 3 m theo hướng 0 độ
LINE 2.0 90	Đi thẳng 2 m theo hướng 90 độ
GOTO 3.0 5.0	Di chuyển tới tọa độ (3m, 5m)
STOP	Dừng robot
RESET	Reset encoder và tọa độ robot
STATUS	In trạng thái robot
ENCODER	In giá trị encoder của 3 động cơ
Ví dụ:
LINE 3.0 0 => Robot sẽ di chuyển thẳng 3 mét theo trục X.
GOTO 3.0 5.0 => Robot sẽ di chuyển tới tọa độ x = 3m, y = 5m.
Các hàm chính trong chương trình
Hàm	Chức năng
setup()	Khởi tạo hệ thống
loop()	Vòng lặp chính
handleSerial()	Đọc và xử lý lệnh Serial
initEncoders()	Khởi tạo encoder A/B
updateEncoder()	Giải mã xung encoder
updateWheelSpeedAndPose()	Tính tốc độ bánh và cập nhật tọa độ robot
forwardKinematic()	Động học thuận
inverseKinematic()	Động học nghịch
bodyToWorld()	Đổi vận tốc từ hệ robot sang hệ toàn cục
worldToBody()	Đổi vận tốc từ hệ toàn cục sang hệ robot
pidCompute()	Tính toán bộ điều khiển PID
computePositionController()	PID vị trí
computeSpeedController()	PID tốc độ
driveMotors()	Xuất PWM điều khiển động cơ
targetReached()	Kiểm tra robot đã tới đích
finishMotion()	Dừng robot khi hoàn thành quỹ đạo

Hiệu chỉnh thông số
Trước khi chạy thực tế, cần kiểm tra và hiệu chỉnh các thông số sau trong code:
ROBOT_RADIUS_L_M
WHEEL_RADIUS_M
ENCODER_PPR_ONE_CHANNEL
QUAD_FACTOR
ENCODER_DIR_SIGN[]
MOTOR_DIR_SIGN[]
Nếu robot chạy sai chiều, cần đảo dấu trong:
MOTOR_DIR_SIGN[]
Nếu encoder đếm ngược chiều thực tế, cần đảo dấu trong:
ENCODER_DIR_SIGN[]
Nếu tốc độ hoặc vị trí tính toán sai, cần kiểm tra lại:
COUNTS_PER_REV
WHEEL_RADIUS_M
ROBOT_RADIUS_L_M
Ghi chú
•	Code hiện tại sử dụng encoder A/B theo kiểu quadrature.
•	Nếu thông số encoder đã là CPR sau giải mã x4, đặt:
QUAD_FACTOR = 1.0f;
•	Nếu thông số encoder là PPR một kênh, có thể dùng:
QUAD_FACTOR = 4.0f;
•	Các hệ số PID cần được hiệu chỉnh thực nghiệm để phù hợp với robot thực tế.
•	Nên thử từng motor riêng trước khi chạy toàn bộ robot.

