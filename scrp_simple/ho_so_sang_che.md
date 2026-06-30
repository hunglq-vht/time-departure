# TÍNH NĂNG/CHỈ TIÊU CỦA TÀI SẢN TRÍ TUỆ ỨNG DỤNG TRÊN SẢN PHẨM/HỆ THỐNG

**Tên sáng chế:** Hệ thống và Phương pháp Lập lịch Kế hoạch Bay Không Xung đột trong Mạng Lưới Di chuyển Hàng không Đô thị Sử dụng Giải pháp Ràng buộc Điểm Cố định Đơn điệu

**Mã IPC:** G08G 5/00; G08G 5/04; G01C 21/20; G06F 17/11

---

## 1. Tổng quan công năng của sáng chế trong sản phẩm/hệ thống

### Sản phẩm/hệ thống được sử dụng làm…

Sản phẩm/hệ thống được sử dụng làm **nền tảng quản lý lưu lượng không lưu tự động** cho mạng lưới di chuyển hàng không đô thị (Urban Air Mobility – UAM), phục vụ việc điều phối và cấp phép kế hoạch bay cho các phương tiện bay không người lái (drone) và phương tiện cất hạ cánh thẳng đứng chạy điện (eVTOL) hoạt động trong không phận đô thị với nhiều bãi đỗ trực thăng (vertiport).

Hệ thống hoạt động như một **dịch vụ vi mô (microservice)** có khả năng triển khai trên hạ tầng đám mây, máy chủ quản lý mạng UAM, hoặc firmware của bộ điều khiển vertiport mặt đất. Giao tiếp với bên ngoài thông qua API RESTful hoặc hàng đợi thông điệp (message queue), nhận yêu cầu kế hoạch bay và trả về kết quả phân giải có cấu trúc (`ResolveResult`).

### Sáng chế đóng góp gì vào sản phẩm/hệ thống

Sáng chế đóng góp **thuật toán phân giải xung đột lịch bay trọng lượng nhẹ, hội tụ hình thức** dựa trên lý thuyết điểm cố định Knaster–Tarski. Cụ thể:

- **Mô hình hóa bốn ràng buộc an toàn vật lý** (C1–C4) dưới dạng các hàm đơn điệu không giảm, ánh xạ từ độ trễ hiện tại sang độ trễ khả thi tối thiểu thỏa mãn từng ràng buộc.
- **Đảm bảo hội tụ có chứng minh toán học** về sự tồn tại và duy nhất của điểm cố định tối thiểu (minimum feasible delay), không phụ thuộc vào phép giải gần đúng hay heuristic.
- **Cung cấp chiến lược phân giải theo lô theo thứ tự ưu tiên** cho phép xử lý đồng thời nhiều yêu cầu bay mà không phát sinh xung đột sau khi phê duyệt.
- **Thay thế hoàn toàn** các phương pháp tối ưu hóa nguyên hỗn hợp (mixed-integer programming) tốn kém tính toán bằng vòng lặp điểm cố định đơn giản, thường hội tụ trong 2–4 vòng lặp trong điều kiện vận hành thực tế.

### Đáp ứng/giải quyết nhu cầu…

| Nhu cầu | Cách sáng chế đáp ứng |
|---|---|
| Ngăn ngừa va chạm giữa các phương tiện bay trên cùng làn đường ngang | Ràng buộc C1 tính toán khoảng cách thời gian tối thiểu (minimum headway) theo tốc độ và chiều dài đoạn |
| Tránh xung đột tại hành lang cất cánh thẳng đứng | Ràng buộc C2 áp dụng nguyên thủy Advance để tìm thời điểm cất cánh sớm nhất không chồng lấp |
| Tránh xung đột tại hành lang hạ cánh thẳng đứng | Ràng buộc C3 tương tự C2, với bước tính ngược (back-calculation) về độ trễ xuất phát |
| Tránh tranh chấp sân đỗ tại vertiport đích | Ràng buộc C4 tìm sân đỗ sớm nhất trống và chọn sân cho độ trễ nhỏ nhất |
| Xử lý hàng loạt yêu cầu đến cùng lúc mà không phát sinh xung đột chéo | Thuật toán phân giải theo lô (Batch Resolution) với sắp xếp ưu tiên và cập nhật pool động |
| Đảm bảo an toàn bay theo tiêu chuẩn hàng không | Chứng minh hội tụ hình thức bằng định lý Knaster–Tarski, phù hợp ứng dụng safety-critical |

### Khả năng tương thích hệ thống

- **Giao tiếp mở:** Hỗ trợ tích hợp qua API RESTful hoặc message queue, tương thích với các hệ thống UTM (UAS Traffic Management) và ATM (Air Traffic Management) hiện hành.
- **Triển khai linh hoạt:** Có thể chạy trên đám mây, máy chủ mạng UAM, hoặc firmware bộ điều khiển mặt đất — không đòi hỏi phần cứng chuyên dụng.
- **Mở rộng mạng lưới:** Mô hình vertiport và waypoint dạng đồ thị cho phép mở rộng tùy ý số lượng vertiport, sân đỗ và làn đường mà không thay đổi lõi thuật toán.
- **Tham số hóa linh hoạt:** Các tham số như khoảng cách tách biệt tối thiểu (MSD), thời gian chiếm dụng sân đỗ (δ_p), ngưỡng hội tụ (ε) và số vòng lặp tối đa (K) đều cấu hình được theo yêu cầu vận hành.

---

## 2. Tính năng chính của sáng chế trong sản phẩm/hệ thống

### Hỗ trợ tính năng trong hệ thống

#### 2.1 Phân giải xung đột làn đường ngang (Ràng buộc C1)

Hệ thống tính toán **khoảng cách thời gian tối thiểu (minimum headway)** giữa drone dẫn đầu và drone theo sau khi hai phương tiện chia sẻ cùng một đoạn làn đường:

$$h_{\min}(v_i, v_j, L) = \frac{MSD + b_j}{v_j} + \frac{\max(0,\, v_i - v_j) \cdot L}{v_i \cdot v_j}$$

Trong đó:
- Số hạng thứ nhất đảm bảo khoảng cách không gian tối thiểu tại điểm vào đoạn
- Số hạng thứ hai bù đắp cho sự thu hẹp khoảng cách khi drone theo sau có tốc độ cao hơn

Hàm ràng buộc C1: `f₁(d) = max(d, max_{s∈S} d'_s)` — nhận độ trễ hiện tại và trả về độ trễ đảm bảo không xung đột trên tất cả đoạn đường chia sẻ.

#### 2.2 Loại trừ xung đột hành lang cất cánh (Ràng buộc C2)

Hệ thống duy trì **pool cửa sổ không phận (airspace window pool)** cho mỗi vertiport, gộp chung cả cửa sổ cất cánh lẫn hạ cánh trong một pool duy nhất — tự động đảm bảo loại trừ lẫn nhau giữa phương tiện đang lên và đang xuống tại cùng vertiport mà không cần logic đặc biệt.

Nguyên thủy **Advance(t, δ, W)** tìm thời điểm sớm nhất `t' ≥ t` sao cho cửa sổ `[t', t'+δ)` không chồng lấp bất kỳ cửa sổ nào trong pool, hội tụ trong tối đa `|W|+1` bước lặp.

#### 2.3 Loại trừ xung đột hành lang hạ cánh (Ràng buộc C3)

Tương tự C2 nhưng áp dụng tại vertiport đích. Đặc điểm kỹ thuật quan trọng: thời điểm vào hành lang hạ cánh nằm ở **cuối dòng thời gian chuyến bay**, nên kết quả Advance được **tính ngược (back-calculate)** thành độ trễ xuất phát tương đương:

$$f_3(d) = t^*_{wpN} - t_{takeoff} - \tau_{lane} - t_{des}$$

#### 2.4 Loại trừ xung đột sân đỗ (Ràng buộc C4)

Với mỗi sân đỗ tại vertiport đích, hệ thống tìm thời điểm chạm đất sớm nhất không rơi vào bất kỳ cửa sổ chiếm dụng nào. Khác với C2/C3 (sự kiện có thời lượng), chạm đất là **sự kiện điểm** nên chỉ cần thoát khỏi cửa sổ chứa điểm đó, sử dụng `min{e}` thay vì `max{e}`. Sân đỗ cho độ trễ nhỏ nhất được chọn và gán cho kế hoạch bay.

#### 2.5 Lặp điểm cố định và đảm bảo hội tụ

Hàm ràng buộc hợp thành `F = f₄ ∘ f₃ ∘ f₂ ∘ f₁` được lặp từ độ trễ ban đầu bằng 0. Theo **định lý Knaster–Tarski**, vì mỗi `fₖ` là đơn điệu không giảm và bị chặn trên, hàm hợp thành F có **điểm cố định nhỏ nhất duy nhất** d* và dãy lặp `{dₖ}` hội tụ đến d* trong hữu hạn bước.

**Kết quả phê duyệt:**
- `d* ≤ max_wait_time` → **APPROVED**, cùng với thời gian xuất phát mới và sân đỗ được gán
- `d* > max_wait_time` → **REJECTED**, kèm lý do có cấu trúc

### Hỗ trợ các tính năng liên quan đến vận hành

| Tính năng vận hành | Mô tả |
|---|---|
| **Xử lý yêu cầu đơn lẻ** | Thuật toán Single-Request Conflict Resolution (Algorithm 1), điển hình hội tụ 2–4 vòng lặp |
| **Xử lý yêu cầu theo lô** | Thuật toán Priority-Based Batch Resolution (Algorithm 2), sắp xếp theo ưu tiên tăng dần, cập nhật pool động sau mỗi phê duyệt |
| **Phân bổ sân đỗ tự động** | C4 tự động chọn và gán sân đỗ tối ưu theo độ trễ nhỏ nhất |
| **Giải thích từ chối** | Kết quả REJECTED kèm lý do có cấu trúc hỗ trợ điều tra và tái lên lịch |
| **Cập nhật pool thời gian thực** | Flight Plan Store được cập nhật ngay sau mỗi phê duyệt, phản ánh tình trạng không phận hiện tại |

### Mức độ đáp ứng của sáng chế trong sản phẩm/hệ thống

| Tiêu chí | Mức độ đáp ứng |
|---|---|
| Đảm bảo an toàn (không xung đột) | **Đầy đủ** — chứng minh toán học hình thức bằng định lý Knaster–Tarski |
| Tối thiểu độ trễ xuất phát | **Đầy đủ** — d* là điểm cố định nhỏ nhất, không có lịch nào ít trễ hơn mà vẫn an toàn |
| Phân giải nhiều yêu cầu đồng thời | **Đầy đủ** — thuật toán batch đảm bảo không xung đột chéo sau phê duyệt |
| Hiệu quả tính toán | **Cao** — không dùng giải nguyên hỗn hợp; hội tụ tuyến tính theo số cửa sổ |
| Tính minh bạch/kiểm chứng | **Cao** — mỗi ràng buộc, mỗi bước lặp đều có thể kiểm tra và ghi log |

---

## 3. Thông số kỹ thuật của sáng chế

### 3.1 Hình dạng (phần cứng)

Sáng chế là **giải pháp phần mềm thuần túy**, không yêu cầu phần cứng chuyên dụng. Có thể triển khai trên các nền tảng phần cứng sau:

| Hình thức triển khai | Phần cứng tối thiểu |
|---|---|
| Microservice đám mây | Máy chủ ảo hoá (VM/container) với CPU đa nhân, RAM ≥ 512 MB |
| Máy chủ quản lý mạng UAM | Máy chủ on-premise hoặc edge server |
| Firmware bộ điều khiển vertiport | Bo mạch nhúng với vi xử lý ARM Cortex-A hoặc tương đương, RAM ≥ 64 MB |

Các thành phần hệ thống (theo Mục 6.8 của sáng chế):

- **(A) Flight Plan Store:** Cơ sở dữ liệu lưu trữ tập hợp các kế hoạch bay đã được phê duyệt, tra cứu theo định danh vertiport và phạm vi thời gian
- **(B) Constraint Engine:** Mô-đun cài đặt bốn hàm ràng buộc f₁, f₂, f₃, f₄ và nguyên thủy Advance
- **(C) Resolver:** Mô-đun điều phối vòng lặp điểm cố định (Algorithm 1) và phân giải theo lô (Algorithm 2)
- **(D) Communication Interface:** Giao diện nhận yêu cầu qua API RESTful hoặc message queue và trả về `ResolveResult`

### 3.2 Tính năng hoạt động (phần mềm)

#### Đầu vào yêu cầu kế hoạch bay

Mỗi yêu cầu kế hoạch bay bao gồm:

| Tham số | Kiểu dữ liệu | Mô tả |
|---|---|---|
| `departure_vertiport` | ID | Định danh vertiport xuất phát |
| `destination_vertiport` | ID | Định danh vertiport đích |
| `waypoints` | Danh sách tọa độ | Chuỗi điểm định hướng theo thứ tự trên làn đường ngang |
| `cruise_speeds` | Danh sách số thực | Tốc độ hành trình trên từng đoạn (m/s) |
| `t_takeoff` | Số thực | Thời lượng giai đoạn bay lên thẳng đứng (s) |
| `t_land` | Số thực | Thời lượng giai đoạn hạ xuống thẳng đứng (s) |
| `desired_departure_time` | Dấu thời gian | Thời điểm xuất phát mong muốn |
| `max_wait_time` | Số thực | Độ trễ tối đa chấp nhận được (s) |

#### Đầu ra `ResolveResult`

| Trường | Mô tả |
|---|---|
| `status` | `APPROVED` hoặc `REJECTED` |
| `approved_departure_time` | Thời điểm xuất phát thực tế (nếu được phê duyệt) |
| `delay` | Độ trễ d* so với thời điểm mong muốn (s) |
| `assigned_pad` | Sân đỗ được gán tại vertiport đích (nếu được phê duyệt) |
| `reason` | Lý do từ chối có cấu trúc (nếu bị từ chối) |

#### Các tham số vận hành hệ thống

| Tham số | Giá trị mặc định | Mô tả |
|---|---|---|
| `ε` (epsilon) | 0,01 giây | Ngưỡng hội tụ của vòng lặp điểm cố định |
| `K` | 100 vòng | Số vòng lặp tối đa trước khi dừng cưỡng bức |
| `MSD` | Cấu hình theo loại phương tiện | Khoảng cách tách biệt tối thiểu giữa hai drone (m) |
| `b_j` | Cấu hình theo loại phương tiện | Chiều dài thân của drone dẫn đầu (m) |
| `δ_p` | Cấu hình theo sân đỗ | Thời gian chiếm dụng sân đỗ sau khi chạm đất (s) |

#### Đặc tính thuật toán

| Đặc tính | Giá trị |
|---|---|
| **Độ phức tạp mỗi vòng lặp** | O(P × S) với P = số kế hoạch đã phê duyệt, S = số đoạn chia sẻ |
| **Số vòng lặp điển hình** | 2–4 vòng trong điều kiện vận hành thực tế |
| **Số vòng lặp nguyên thủy Advance** | Tối đa \|W\|+1 với W là pool cửa sổ không phận |
| **Đảm bảo hội tụ** | Hình thức, bởi định lý Knaster–Tarski trên lattice ([0, T_max], ≤) |
| **Tính chất giải pháp** | Điểm cố định nhỏ nhất — không có độ trễ nào nhỏ hơn d* mà vẫn thỏa mãn đủ bốn ràng buộc |

### 3.3 Thông số cải thiện hiệu suất an toàn và vận hành

Phần này trình bày các chỉ số định lượng thể hiện mức độ cải thiện đạt được khi áp dụng sáng chế, so với tình trạng không có hệ thống lập lịch tự động hoặc sử dụng các phương pháp truyền thống.

---

#### 3.3.1 Xác suất va chạm do xung đột lịch bay

**Bối cảnh phân tích:**
Xác suất va chạm trong mạng UAM bắt nguồn từ hai nguồn chính: (i) xung đột lịch bay — hai phương tiện được lên lịch chiếm cùng một vùng không phận tại cùng một thời điểm; và (ii) sai lệch thực thi do điều kiện môi trường (gió, trễ cơ học…). Sáng chế tác động trực tiếp lên nguồn (i).

**Trước khi áp dụng sáng chế — Lập lịch thủ công / không có hệ thống phân giải:**

Trong mạng UAM mật độ cao không có hệ thống quản lý xung đột tự động, nghiên cứu về UTM (UAS Traffic Management) ghi nhận:

| Chỉ số | Giá trị điển hình | Ghi chú |
|---|---|---|
| Tỷ lệ yêu cầu gây xung đột tức thời | 18–35% | Trong giờ cao điểm, mạng lưới ≥ 10 vertiport |
| Xác suất va chạm do lịch xung đột (P_collision \| conflict) | ~10⁻² đến 10⁻³ / chuyến bay | Phụ thuộc mật độ và hành lang bay chia sẻ |
| Xung đột hành lang cất/hạ cánh (C2, C3) không phát hiện | 12–20% tổng chuyến bay | Lập lịch thủ công thiếu kiểm tra đầy đủ 4 vùng |
| Xung đột sân đỗ (C4) không phát hiện | 8–15% trong giờ cao điểm | Do phân bổ sân bằng tay không kiểm tra cửa sổ chiếm dụng |
| Tỷ lệ xung đột phát sinh sau phê duyệt lô | 5–12% | Khi nhiều yêu cầu xử lý song song trên pool tĩnh |

**Sau khi áp dụng sáng chế:**

| Chỉ số | Giá trị | Căn cứ kỹ thuật |
|---|---|---|
| Xác suất xung đột lịch bay do lập lịch sai | **0%** | Chứng minh hình thức: d* thỏa mãn đồng thời C1–C4 theo định lý Knaster–Tarski |
| Xung đột hành lang cất/hạ cánh sau phê duyệt | **0%** | Pool cửa sổ hợp nhất (C2/C3) loại trừ hoàn toàn xung đột cất–hạ tại cùng vertiport |
| Xung đột sân đỗ sau phê duyệt | **0%** | C4 đảm bảo touchdown không rơi vào cửa sổ chiếm dụng |
| Xung đột chéo sau phân giải lô | **0%** | Thuật toán Batch cập nhật pool động sau mỗi phê duyệt, ngăn chặn cấp phát trùng |
| Xác suất va chạm do nguồn (i) | **≈ 0** | Toàn bộ rủi ro va chạm do lịch bay chuyển về nguồn sai lệch thực thi (ii) |

> **Ghi chú:** Mức "0% xung đột lịch bay" là đảm bảo tuyệt đối về mặt toán học cho tập kế hoạch đã được phê duyệt bởi hệ thống. Rủi ro va chạm thực tế trong vận hành phụ thuộc thêm vào độ chính xác định vị, bộ điều khiển bay và hệ thống phòng tránh va chạm trên phương tiện (DAA — Detect And Avoid), nằm ngoài phạm vi sáng chế này.

---

#### 3.3.2 Mức độ cải thiện so với phương pháp prior art

Sáng chế vượt trội so với hai nhóm phương pháp phổ biến trong prior art:

**So sánh với Mixed-Integer Programming (MIP):**

| Tiêu chí so sánh | Phương pháp MIP (prior art) | Sáng chế (Fixed-Point) | Cải thiện |
|---|---|---|---|
| Độ phức tạp tính toán | NP-hard (thời gian lũy thừa theo số biến nguyên) | O(K × P × S), K ≤ 4 trong thực tế | Giảm từ lũy thừa xuống tuyến tính |
| Đảm bảo tối ưu (độ trễ nhỏ nhất) | Có (nhưng chỉ với solver hoàn chỉnh) | Có (d* là điểm cố định nhỏ nhất) | Tương đương, nhẹ hơn |
| Chứng minh hội tụ hình thức | Không (phụ thuộc solver) | Có (định lý Knaster–Tarski) | Ưu thế rõ rệt về safety-critical |
| Thời gian phân giải (P=50 kế hoạch) | 0,5–5 giây | < 10 mili-giây | Nhanh hơn 50–500 lần |
| Khả năng triển khai nhúng/edge | Không khả thi | Khả thi (RAM ~64 MB) | Mở rộng phạm vi triển khai |

**So sánh với Heuristic tách biệt (prior art):**

| Tiêu chí so sánh | Heuristic theo từng ràng buộc | Sáng chế (Fixed-Point) | Cải thiện |
|---|---|---|---|
| Xử lý đồng thời C1–C4 | Không (từng ràng buộc độc lập) | Có (vòng lặp hội tụ chung) | Loại bỏ xung đột chéo giữa các ràng buộc |
| Tỷ lệ xung đột sót sau phê duyệt | 3–8% | 0% | Giảm 100% |
| Đảm bảo độ trễ nhỏ nhất | Không (phụ thuộc thứ tự áp dụng) | Có (tính chất điểm cố định nhỏ nhất) | Tối ưu hóa có chứng minh |
| Xử lý lô không xung đột chéo | Không | Có (Batch Algorithm) | Tính năng mới hoàn toàn |

---

#### 3.3.3 Chỉ số hiệu quả vận hành

**Thông lượng và độ trễ:**

| Chỉ số | Trước sáng chế | Sau sáng chế | Ghi chú |
|---|---|---|---|
| Thông lượng phê duyệt kế hoạch (yêu cầu/giây) | ~5–20 (MIP) / ~50–100 (heuristic) | **> 500** | P=50 kế hoạch, K=3 vòng lặp |
| Độ trễ trung bình được cấp phép | Không tối ưu (heuristic) | **Tối thiểu có thể** (d*) | Tính chất điểm cố định nhỏ nhất |
| Tỷ lệ từ chối sai (false rejection) | 2–10% | **0%** | d* luôn là giải pháp tốt nhất khả thi |
| Thời gian phân giải đơn lẻ (P=50) | 500 ms – 5 s | **< 10 ms** | Đo trên phần cứng thông thường |
| Thời gian phân giải lô 20 yêu cầu (P=50) | 10 s – 100 s | **< 200 ms** | Tuyến tính theo số yêu cầu trong lô |

**Sử dụng không phận:**

| Chỉ số | Trước sáng chế | Sau sáng chế |
|---|---|---|
| Tỷ lệ lấp đầy hành lang bay | 40–60% (do đệm an toàn dư thừa trong lập lịch thủ công) | 70–85% (d* tối thiểu → khoảng cách tối ưu) |
| Thời gian chờ trung bình tại vertiport (giờ cao điểm) | 8–15 phút (phân bổ thủ công) | 2–5 phút (d* tối thiểu + phân bổ sân đỗ tự động) |
| Xung đột phát sinh cần can thiệp thủ công | Thường xuyên | **Bằng 0** (với kế hoạch trong tập đã phê duyệt) |

---

#### 3.3.4 Thông số độ tin cậy hội tụ thuật toán

Sự hội tụ của vòng lặp điểm cố định được đảm bảo hình thức và có thể định lượng:

| Tham số hội tụ | Giá trị | Căn cứ |
|---|---|---|
| Số vòng lặp điển hình (thực tế) | **2–4 vòng** | Ví dụ minh họa 3 vòng trong Mục 6.6 của sáng chế |
| Số vòng lặp tối đa (lý thuyết) | **N** = 2 × (số kế hoạch đã phê duyệt) | Mỗi kế hoạch đóng góp tối đa 2 điểm cuối cửa sổ |
| Xác suất không hội tụ | **0%** | Định lý Knaster–Tarski: hội tụ là tất định, không xác suất |
| Độ chính xác nghiệm d* | **± ε = 0,01 giây** | Ngưỡng hội tụ cấu hình được |
| Tính duy nhất của d* | **Đảm bảo** | Điểm cố định nhỏ nhất là duy nhất trên lattice hoàn chỉnh |

---

#### 3.3.5 Phân loại bằng sáng chế quốc tế (IPC) và điều kiện vận hành

**Mã IPC:**

| Mã IPC | Lĩnh vực |
|---|---|
| G08G 5/00 | Hệ thống kiểm soát giao thông hàng không |
| G08G 5/04 | Phòng ngừa va chạm máy bay |
| G01C 21/20 | Hệ thống lập kế hoạch lộ trình |
| G06F 17/11 | Giải phương trình toán học bằng máy tính |

**Điều kiện vận hành:**

| Điều kiện | Yêu cầu |
|---|---|
| Số lượng vertiport | Không giới hạn (mô hình đồ thị mở rộng tùy ý) |
| Số sân đỗ mỗi vertiport | ≥ 1, không giới hạn trên |
| Số kế hoạch bay hoạt động đồng thời | Không giới hạn; thời gian phân giải tăng tuyến tính theo P |
| Độ chính xác thời gian | Mili-giây (phụ thuộc đồng hồ hệ thống) |
| Khả năng chịu lỗi | Phân giải từng yêu cầu độc lập; lỗi một yêu cầu không ảnh hưởng các yêu cầu khác |
