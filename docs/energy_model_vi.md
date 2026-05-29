# Mô hình tính năng lượng tiêu hao của drone

## Tổng quan

Hệ thống tính năng lượng tiêu hao nhằm hai mục đích:

1. **Đề xuất tuyến bay** (`route_suggestion.py`): lọc bỏ các tuyến drone không đủ pin để hoàn thành.
2. **Phân giải xung đột** (`scrp.py`): kiểm tra xem pin còn đủ sau khi drone bay xong, chờ ở vùng trời đích và hạ cánh không.

Mô hình chia chuyến bay thành ba giai đoạn: **cất cánh**, **hành trình**, **hạ cánh**, mỗi giai đoạn có mức tiêu thụ công suất khác nhau.

---

## 1. Xác định công suất lơ lửng (hover power)

Công suất lơ lửng `P_hover` là nền tảng của toàn bộ mô hình. Hệ thống xác định `P_hover` theo thứ tự ưu tiên sau:

### Ưu tiên 1 — Giá trị trực tiếp từ datasheet

```
P_hover = hover_power_w  (W)
```

Nếu nhà sản xuất công bố mức tiêu thụ khi lơ lửng, đây là con số chính xác nhất, được dùng trực tiếp.

### Ưu tiên 2 — Tính từ dung lượng pin và thời gian bay

```
P_hover = battery_energy_wh / (flight_time_min / 60)
```

Dựa trên định nghĩa: nếu drone bay lơ lửng liên tục trong `flight_time_min` phút thì hết pin `battery_energy_wh` Wh, suy ra công suất trung bình. Chỉ nên dùng khi `flight_time_min` là thời gian bay lơ lửng (hover endurance), không phải thời gian bay hành trình.

### Ưu tiên 3 — Lý thuyết động lượng (Momentum Theory)

Khi cả hai giá trị trên đều không có, hệ thống suy ra `P_hover` từ các thông số hình học của cánh quạt:

**Bước 1 — Tính công suất lý tưởng theo lý thuyết Rankine-Froude:**

```
T       = MTOW × g                         (lực nâng cần thiết = trọng lượng)
A_disk  = N_rotors × π × (D_prop / 2)²    (tổng diện tích đĩa cánh quạt)
P_ideal = T × sqrt(T / (2 × ρ × A_disk))
```

trong đó `ρ = 1.225 kg/m³` (mật độ không khí tại mực nước biển), `g = 9.81 m/s²`.

**Bước 2 — Tính công suất thực tế:**

```
P_hover = P_ideal / η
```

Hiệu suất `η = 0.60` (mặc định) phản ánh tổng tổn thất thực tế gồm: lực cản biên cánh quạt, tổn thất đầu mút, điện trở cuộn dây motor, tổn thất chuyển mạch của ESC. Giá trị 0.60 được chọn theo hướng thận trọng (overestimate) để đảm bảo kế hoạch bay luôn dự phòng đủ năng lượng.

---

## 2. Tính công suất hành trình (cruise power)

### Mô hình vật lý (dùng khi có đủ thông số hình học)

Khi biết `N_rotors`, `D_prop` và `MTOW`, hệ thống tính công suất hành trình theo vận tốc thực của drone. Công suất được chia thành ba thành phần:

#### 2a. Công suất cảm ứng (induced power)

Ở chế độ lơ lửng, toàn bộ công suất dùng để tạo ra lực nâng (công suất cảm ứng). Khi drone bay tiến, lực nâng được tạo ra hiệu quả hơn, nên công suất cảm ứng giảm theo vận tốc (xấp xỉ Glauert):

```
v_i0              = sqrt(MTOW × g / (2 × ρ × A_disk))   (vận tốc cảm ứng khi lơ lửng)
P_induced_cruise  = FM × P_hover / sqrt(1 + (v / v_i0)²)
```

Hệ số hình dạng cánh quạt `FM = 0.75` (figure of merit) biểu thị tỉ lệ công suất cảm ứng trong tổng công suất lơ lửng.

#### 2b. Công suất cản biên (profile drag power)

Phần còn lại của công suất lơ lửng dùng để thắng lực cản hình dạng cánh quạt. Thành phần này gần như không đổi theo vận tốc bay ngang:

```
P_profile = (1 - FM) × P_hover
```

#### 2c. Công suất cản ký sinh (parasite drag power)

Thân drone và các bộ phận tạo ra lực cản tăng theo lập phương vận tốc:

```
F_drag_max = MTOW × g × tan(θ_max)         (lực cản ở tốc độ tối đa, khi nghiêng góc θ_max)
Cd_A       = F_drag_max / (0.5 × ρ × v_max²)   (hệ số cản × diện tích frontal)
P_parasite = 0.5 × ρ × Cd_A × v³
```

#### Tổng công suất hành trình

```
P_cruise(v) = P_induced_cruise(v) + P_profile + P_parasite(v)
```

Kiểm tra tính nhất quán: khi `v = 0`, `P_cruise(0) = FM × P_hover + (1-FM) × P_hover = P_hover` ✓

#### Ảnh hưởng của gió

Gió làm tăng vận tốc không khí tương đối mà drone phải vượt qua. Hiệu ứng kéo trung bình trên cả hành trình được mô hình hóa bằng vận tốc hiệu dụng RMS:

```
v_eff = sqrt(v_cruise² + v_wind²)
```

`v_eff` này được dùng thay cho `v` trong công thức `P_cruise` ở trên.

### Mô hình dự phòng (fallback — dùng khi không có thông số hình học)

Khi chỉ có `P_hover` mà không biết `N_rotors`, `D_prop`, `MTOW`, hệ thống dùng phương pháp ước lượng đơn giản:

```
P_cruise = P_hover × CRUISE_POWER_FACTOR    (mặc định: CRUISE_POWER_FACTOR = 1.2)
```

Đây là mô hình cũ, được giữ lại để tương thích ngược.

---

## 3. Tính năng lượng từng đoạn đường

Mỗi tuyến bay gồm nhiều đoạn (segment). Với mỗi đoạn:

```
v_seg = clamp(v_plan, v_min, v_max)      (vận tốc thực tế, giới hạn trong [v_min, v_max] của đoạn)
t_seg = length / v_seg                   (thời gian bay đoạn, giây)
E_seg = P_cruise(v_eff) × t_seg / 3600   (năng lượng đoạn, Wh)
```

Tổng năng lượng hành trình:

```
E_cruise = Σ E_seg  (trên tất cả các đoạn)
```

---

## 4. Giai đoạn cất cánh và hạ cánh

Hai giai đoạn chuyển tiếp theo phương thẳng đứng được ước lượng đơn giản dựa trên công suất lơ lửng:

```
t_takeoff  = takeoff_height_m / max_ascent_speed_ms
E_takeoff  = 1.10 × P_hover × t_takeoff / 3600   (cất cánh tiêu thụ hơn lơ lửng ~10%)

t_landing  = landing_height_m / max_descent_speed_ms
E_landing  = 0.50 × P_hover × t_landing / 3600   (hạ cánh tiêu thụ ít hơn, motor phanh)
```

Hệ số 1.10 khi cất cánh phản ánh việc motor cần công suất cao hơn để tăng độ cao. Hệ số 0.50 khi hạ cánh phản ánh việc motor hoạt động ở chế độ phanh tái sinh, tiêu thụ ít hơn hover.

---

## 5. Tổng năng lượng và SoC còn lại

### Trong đề xuất tuyến bay

```
E_total    = E_takeoff + E_cruise + E_landing
SoC_remaining = SoC_0 - E_total / battery_energy_wh
```

Tuyến bay bị loại nếu `SoC_remaining < soc_min` (mặc định 20%).

### Trong phân giải xung đột (SCRP)

Ngoài năng lượng bay, drone còn có thể phải chờ lơ lửng ở vùng trời đích nếu cổng hạ cánh chưa sẵn sàng:

```
t_arrive    = t_dep_star + t_takeoff + t_cruise         (thời điểm đến vùng trời đích)
t_hover_wait = max(0, t_slot_start - t_arrive)          (thời gian chờ)
E_hover     = P_hover × t_hover_wait / 3600

SoC_remaining = SoC_0 - (E_cruise + E_hover) / C_bat
```

Kế hoạch bay bị từ chối nếu `SoC_remaining < SOC_MIN`.

---

## 6. Tóm tắt các hằng số vật lý

| Ký hiệu | Giá trị | Ý nghĩa |
|---|---|---|
| `ρ` | 1.225 kg/m³ | Mật độ không khí tại mực nước biển (ISA) |
| `g` | 9.81 m/s² | Gia tốc trọng trường |
| `FM` | 0.75 | Figure of merit — tỉ lệ công suất cảm ứng trong P_hover |
| `η` | 0.60 | Hiệu suất suy diễn P_hover từ lý thuyết động lượng |
| `K_climb` | 1.10 | Hệ số công suất cất cánh (× P_hover) |
| `K_descent` | 0.50 | Hệ số công suất hạ cánh (× P_hover) |
| `CRUISE_POWER_FACTOR` | 1.20 | Hệ số dự phòng trong mô hình cũ (chỉ dùng khi không có thông số hình học) |
