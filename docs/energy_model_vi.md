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

#### Cơ sở lý thuyết: Mô hình đĩa truyền động Rankine-Froude

Mô hình đĩa truyền động (actuator disk model) mô tả cánh quạt như một đĩa mỏng, vô hạn mỏng, đẩy dòng không khí đi qua. Các giả thiết:
- Dòng chảy ổn định, không nén được, không nhớt.
- Đĩa tác dụng lực nâng đồng đều lên toàn bộ diện tích `A_disk`.
- Dòng khí xa phía trước (upstream infinity) có vận tốc bằng 0 (drone lơ lửng).

**Bước dẫn xuất:**

1. **Phương trình liên tục** (bảo toàn khối lượng): khối lượng không khí đi qua đĩa mỗi giây là `ṁ = ρ × A_disk × v_i`, trong đó `v_i` là vận tốc cảm ứng tại đĩa.

2. **Định lý động lượng**: lực nâng bằng tốc độ thay đổi động lượng. Dòng khí được tăng tốc từ 0 đến vận tốc `w` ở xa phía sau (wake velocity):
   ```
   T = ṁ × w = ρ × A_disk × v_i × w
   ```

3. **Định lý Froude**: bằng cách áp dụng bảo toàn năng lượng (Bernoulli phía trên và phía dưới đĩa), có thể chứng minh `v_i = w/2`, tức vận tốc tại đĩa bằng đúng một nửa vận tốc ở wake. Thay vào:
   ```
   T = ρ × A_disk × v_i × (2 × v_i) = 2 × ρ × A_disk × v_i²
   → v_i0 = sqrt(T / (2 × ρ × A_disk))
   ```
   Đây là **vận tốc cảm ứng khi lơ lửng** `v_i0`.

4. **Công suất lý tưởng**: công suất bằng lực nhân vận tốc tại điểm đặt lực (đĩa):
   ```
   P_ideal = T × v_i0 = T × sqrt(T / (2 × ρ × A_disk))
   ```
   Thay `T = MTOW × g` và `A_disk = N_rotors × π × (D_prop/2)²`:
   ```
   T       = MTOW × g
   A_disk  = N_rotors × π × (D_prop / 2)²
   P_ideal = T × sqrt(T / (2 × ρ × A_disk))
   ```

5. **Công suất thực tế** — hiệu chỉnh tổn thất:

   Cánh quạt thực tế tổn thất năng lượng do bốn nguyên nhân:
   - **Lực cản biên (profile drag)**: ma sát của lưỡi dao với không khí (~10–15% tổn thất).
   - **Tổn thất đầu mút (tip losses)**: dòng khí rò từ áp suất cao sang áp suất thấp ở đầu cánh (~5%).
   - **Điện trở cuộn dây motor**: nhiệt tỏa theo `I²R` (~8–12%).
   - **Tổn thất chuyển mạch ESC**: PWM switching losses (~3–5%).

   Tổng hợp thành một hệ số hiệu suất `η = 0.60` (bảo thủ, đảm bảo ước lượng dư năng lượng):
   ```
   P_hover = P_ideal / η
   ```

---

## 2. Tính công suất hành trình (cruise power)

### Mô hình vật lý (dùng khi có đủ thông số hình học)

Khi biết `N_rotors`, `D_prop` và `MTOW`, hệ thống tính công suất hành trình theo vận tốc thực của drone. Công suất được chia thành ba thành phần:

#### 2a. Công suất cảm ứng (induced power)

**Cơ sở lý thuyết: Gần đúng Glauert**

Khi drone bay tiến với vận tốc `v`, dòng khí tới đĩa không còn đứng yên mà có thành phần ngang. Phương trình cân bằng lực nâng tổng quát (Glauert, 1926):

```
T = 2 × ρ × A_disk × v_i × sqrt(v² + v_i²)
```

Để giải phương trình này, ta dùng gần đúng: `v_i << v` khi bay nhanh, khi đó:

```
T ≈ 2 × ρ × A_disk × v_i × v
→ v_i ≈ T / (2 × ρ × A_disk × v) = v_i0² / v
```

Tuy nhiên, gần đúng này thất bại khi `v → 0`. Công thức nội suy toàn dải vận tốc (Bramwell, 1976) được dùng trong thực tế:

```
v_i ≈ v_i0 / sqrt(1 + (v / v_i0)²)
```

Xác minh: khi `v = 0`, `v_i = v_i0` ✓; khi `v >> v_i0`, `v_i ≈ v_i0²/v` ✓

Công suất cảm ứng khi hành trình = `T × v_i`:

```
P_induced_cruise = T × v_i
                 = T × v_i0 / sqrt(1 + (v / v_i0)²)
                 = FM × P_hover / sqrt(1 + (v / v_i0)²)
```

trong đó `FM = T × v_i0 / P_hover = P_induced_hover / P_hover = 0.75` là tỉ lệ công suất cảm ứng trong tổng công suất lơ lửng.

#### 2b. Công suất cản biên (profile drag power)

**Cơ sở lý thuyết:**

Tổng công suất lơ lửng phân thành hai thành phần:
```
P_hover = P_induced_hover + P_profile_hover
```

Theo định nghĩa của Figure of Merit (FM):
```
FM = P_induced_hover / P_hover
→ P_profile_hover = (1 - FM) × P_hover
```

Công suất profile drag phụ thuộc vào lực cản hình dạng lưỡi dao (`C_d0`) và vận tốc quay đầu mút. Khi drone bay tiến ở vận tốc vừa phải (advance ratio `μ = v / (Ω × R) < 0.3`), thành phần này gần như không đổi:

```
P_profile ≈ (1 - FM) × P_hover  ≈ const
```

#### 2c. Công suất cản ký sinh (parasite drag power)

**Cơ sở lý thuyết:**

Thân drone tạo ra lực cản hình dạng (parasite drag) theo công thức khí động học cơ bản:
```
F_drag = 0.5 × ρ × Cd_A × v²
```

trong đó `Cd_A = Cd × A_frontal` là hệ số cản nhân diện tích mặt chắn. Công suất để thắng lực này:
```
P_parasite = F_drag × v = 0.5 × ρ × Cd_A × v³
```

**Dẫn xuất `Cd_A` từ góc nghiêng tối đa:**

Ở tốc độ tối đa `v_max`, drone nghiêng một góc `θ_max` về phía trước. Phân tích lực:
- Thành phần thẳng đứng của lực đẩy cân bằng trọng lực: `T × cos(θ_max) = MTOW × g`
- Thành phần ngang cân bằng lực cản: `T × sin(θ_max) = F_drag`

Chia hai vế:
```
F_drag = MTOW × g × tan(θ_max)
```

Mặt khác `F_drag = 0.5 × ρ × Cd_A × v_max²`, suy ra:
```
Cd_A = F_drag / (0.5 × ρ × v_max²)
     = MTOW × g × tan(θ_max) / (0.5 × ρ × v_max²)
```

#### Tổng công suất hành trình

```
P_cruise(v) = P_induced_cruise(v) + P_profile + P_parasite(v)
```

Kiểm tra tính nhất quán: khi `v = 0`,
```
P_cruise(0) = FM × P_hover + (1 - FM) × P_hover + 0 = P_hover  ✓
```

#### Ảnh hưởng của gió

**Cơ sở lý thuyết:**

Gió tạo ra vận tốc không khí tương đối (airspeed) mà drone phải vượt qua, ngay cả khi drone đứng yên so với mặt đất. Trên hành trình, hướng gió thay đổi liên tục. Giá trị hiệu dụng RMS là ước lượng tốt cho tác động trung bình:

```
v_eff = sqrt(v_cruise² + v_wind²)
```

Đây là norm Euclidean của vector vận tốc bay và vector gió, tương ứng với năng lượng trung bình khi tích phân ngẫu nhiên hướng gió đồng đều. `v_eff` được thay vào `P_cruise` thay cho `v`.

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

### Cơ sở lý thuyết: Bay leo (climbing flight)

Khi drone leo lên với vận tốc thẳng đứng `V_c`, công suất cần thiết gồm hai phần:
1. Công suất cảm ứng để duy trì lực nâng (như hover nhưng dòng khí đến đĩa đã có thành phần `V_c`).
2. Công suất để thắng trọng lực: `P_climb_gravity = T × V_c = MTOW × g × V_c`.

Theo lý thuyết động lượng cho bay leo:
```
P_climb / P_hover ≈ 1 + V_c / (2 × v_i0)
```

Với `v_i0` cỡ 4–6 m/s và `V_c` cỡ 1–3 m/s thông thường, tỉ số này xấp xỉ **1.10–1.25**. Hệ số `K_climb = 1.10` là giá trị bảo thủ (underestimate power, overestimate endurance):

```
t_takeoff  = takeoff_height_m / max_ascent_speed_ms
E_takeoff  = 1.10 × P_hover × t_takeoff / 3600
```

### Cơ sở lý thuyết: Hạ cánh — chế độ tái sinh (regenerative braking)

Khi drone hạ xuống chậm, cánh quạt hoạt động ở chế độ **windmill brake** (đĩa truyền động làm việc ngược): không khí đẩy từ dưới lên, motor trở thành máy phát điện. Phân tích năng lượng:

- Trọng lực thực hiện công dương trên drone: `W_grav = MTOW × g × h`.
- Phần công này được thu hồi qua tái sinh, phần còn lại tiêu tán thành nhiệt.
- Thực nghiệm và mô phỏng CFD cho đa rotor cho thấy năng lượng tiêu thụ từ pin khi hạ cánh bằng **40–55%** so với hover (Stingu et al., 2020). Hệ số `K_descent = 0.50` là điểm giữa vùng này:

```
t_landing  = landing_height_m / max_descent_speed_ms
E_landing  = 0.50 × P_hover × t_landing / 3600
```

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

---

## 7. Sơ đồ dẫn xuất công thức

```
Lý thuyết đĩa truyền động (Rankine-Froude)
│
├─ Bảo toàn khối lượng:  ṁ = ρ·A_disk·v_i
├─ Bảo toàn động lượng:  T = ṁ·w = 2·ρ·A_disk·v_i²
├─ Định lý Froude:        v_i = w/2
│
├──► v_i0 = sqrt(T / (2·ρ·A_disk))           [vận tốc cảm ứng hover]
│
├──► P_ideal = T·v_i0 = T^(3/2)/sqrt(2·ρ·A_disk)  [công suất lý tưởng]
│
└──► P_hover = P_ideal / η                    [công suất thực tế hover]
         │
         ├─ Gần đúng Glauert (bay tiến)
         │      v_i(v) ≈ v_i0 / sqrt(1+(v/v_i0)²)
         │      P_induced = FM·P_hover / sqrt(1+(v/v_i0)²)
         │
         ├─ Phân tách FM (profile drag)
         │      P_profile = (1-FM)·P_hover  ≈ const
         │
         ├─ Cân bằng lực ở v_max (parasite drag)
         │      F_drag = MTOW·g·tan(θ_max)
         │      Cd_A = F_drag / (0.5·ρ·v_max²)
         │      P_parasite = 0.5·ρ·Cd_A·v³
         │
         └──► P_cruise(v) = P_induced + P_profile + P_parasite
```

---

## 8. Tài liệu tham khảo lý thuyết

| Công thức | Nguồn gốc |
|---|---|
| Actuator disk, `P_ideal = T·v_i` | Rankine (1865), Froude (1889) — *Momentum Theory of Propellers* |
| `v_i = w/2` tại đĩa | W.J.M. Rankine, *On the Mechanical Principles of the Action of Propellers*, 1865 |
| Glauert forward-flight inflow | H. Glauert, *Airplane Propellers*, 1935 (Durand Vol. IV) |
| Figure of Merit FM | J. Seddon & S. Newman, *Basic Helicopter Aerodynamics*, 3rd ed., 2011 |
| Parasite drag `P = 0.5·ρ·CdA·v³` | Anderson, *Introduction to Flight*, McGraw-Hill, 2016 |
| Climb power ratio | A.R.S. Bramwell, *Helicopter Dynamics*, 1976 |
| Descent regenerative braking | Stingu et al., *Power Consumption Model for UAV*, IEEE ICUAS 2020 |
