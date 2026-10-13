# Kiến trúc dự án Sokoban

## 1. Cấu trúc các file

- `solution.py`: phần sinh viên triển khai. Chứa hàm `solve`, A*, heuristic và giới hạn thời gian.
- `sokoban.py`: định nghĩa trạng thái Sokoban, luật di chuyển, các hướng và bản đồ mẫu.
- `state.py`: lớp trạng thái cơ sở, gồm `parent`, `action`, `gval` và giao diện trạng thái.
- `validation.py`: phát lại chuỗi hành động để xác nhận lời giải hợp lệ.
- `public_tests.py`: một số kiểm tra giao diện của solver.
- `test_framework.py`: kiểm tra luật chơi và hoạt động của framework.
- `autograder.py`: chạy các public test trong tiến trình riêng.
- `README.md`: mô tả yêu cầu và quy tắc chấm bài.
- `REPORT_TEMPLATE.md`: mẫu gợi ý viết báo cáo.

Chỉ nộp `solution.py`. Không sửa các file framework để thay đổi luật chơi.

## 2. Trình tự xử lý

1. **Nhận trạng thái đầu vào**  
   `solve(initial_state, timebound)` nhận bàn chơi ban đầu và ngân sách thời gian. Solver không sửa trạng thái này.

2. **Kiểm tra điều kiện kết thúc sớm**  
   Nếu hết thời gian hoặc số thùng nhiều hơn số ô đích, trả về `False`. Nếu mọi thùng đã nằm trên ô đích, trả về trạng thái ban đầu.

3. **Chuẩn bị thông tin tĩnh của bàn chơi**  
   Đọc kích thước, tường và ô đích. Từ mỗi ô đích, tìm ngược các vị trí mà thùng có thể được đẩy tới đó, có xét vị trí đứng cần thiết của robot.

4. **Tính heuristic cho trạng thái**  
   - Tính chi phí đẩy thấp nhất bằng cách ghép mỗi thùng với một ô đích riêng. Thuật toán Hungarian chọn phương án có tổng chi phí nhỏ nhất.
   - Cộng khoảng cách Manhattan thấp nhất từ robot tới một vị trí có thể đứng để đẩy thùng.
   - Đặt heuristic bằng `0` nếu trạng thái đã là đích. Nếu không thể ghép thùng với các ô đích, loại trạng thái đó khỏi tìm kiếm.

5. **Khởi tạo A***  
   Đưa trạng thái đầu vào vào hàng đợi ưu tiên với độ ưu tiên `gval + heuristic`. `gval` là số bước đã đi; mỗi lần robot di chuyển, kể cả đẩy thùng, tính một bước.

6. **Mở rộng trạng thái**  
   Lấy trạng thái có độ ưu tiên thấp nhất khỏi hàng đợi. Nếu chưa phải đích, gọi `state.successors()` để lấy các nước đi hợp lệ theo luật framework.

7. **Loại đường đi kém hơn**  
   Với mỗi trạng thái kế tiếp, tạo khóa từ vị trí robot và thùng. Bỏ qua trạng thái nếu cùng cấu hình đã được tìm thấy với chi phí bằng hoặc thấp hơn; nếu tốt hơn, ghi nhận chi phí mới và đưa vào hàng đợi.

8. **Kiểm tra thời gian và kết quả**  
   Nếu lấy được trạng thái đích khỏi hàng đợi trước khi hết thời gian, trả về trạng thái đó. Các liên kết `parent`, `action`, `gval` do `successors()` tạo sẵn giữ nguyên để truy ngược lời giải. Nếu hết thời gian hoặc không còn trạng thái để mở rộng, trả về `False`.

9. **Xác nhận lời giải**  
   `validation.py` lần theo `parent` về trạng thái đầu, phát lại từng hành động và kiểm tra tính hợp lệ cũng như trạng thái đích.

## 3. Thành phần và giới hạn

- Solver chỉ dùng thư viện chuẩn Python và các API framework đã cung cấp.
- Heuristic ước lượng số bước còn lại; nó không thay đổi luật chơi.
- Tìm kiếm dừng theo `timebound`; không dùng tiến trình, luồng nền hoặc dữ liệu giữa các test.
- Các lệnh kiểm tra được README hướng dẫn: `python -m unittest test_framework` và `python autograder.py`.
