# Thiết kế Bộ Giải Xung Đột Bay Đơn Giản — `scrp_simple`

## Mục lục

1. [Định nghĩa bài toán](#1-định-nghĩa-bài-toán)
2. [Mô hình hóa dòng thời gian một chuyến bay](#2-mô-hình-hóa-dòng-thời-gian-một-chuyến-bay)
3. [Xác định nguồn xung đột](#3-xác-định-nguồn-xung-đột)
4. [Công thức hóa từng constraint](#4-công-thức-hóa-từng-constraint)
   - [C1 — Phân tách trên lane](#41-c1--phân-tách-trên-lane)
   - [C2 — Không phận cất cánh](#42-c2--không-phận-cất-cánh)
   - [C3 — Không phận hạ cánh](#43-c3--không-phận-hạ-cánh)
   - [C4 — Khả dụng của pad](#44-c4--khả-dụng-của-pad)
5. [Sự ràng buộc giữa các constraints (Coupling)](#5-sự-ràng-buộc-giữa-các-constraints-coupling)
6. [Thuật toán hội tụ (Fixed-point iteration)](#6-thuật-toán-hội-tụ-fixed-point-iteration)
7. [Giải batch nhiều request](#7-giải-batch-nhiều-request)
8. [Tổng hợp kết quả trả về](#8-tổng-hợp-kết-quả-trả-về)

---

## 1. Định nghĩa bài toán

### 1.1 Đầu vào

| Thành phần | Mô tả |
|------------|-------|
| `vertiports` | Tập hợp các vertiport, mỗi vertiport có nhiều pad, mỗi pad có `occupation_duration` riêng |
| `approved_plans` | Các kế hoạch bay đã được duyệt và đang trong hệ thống |
| `request` (hoặc `requests`) | Một hoặc nhiều kế hoạch bay mới cần xét duyệt |

Mỗi kế hoạch bay (cả đã duyệt lẫn request mới) bao gồm:
- **Lộ trình**: `takeoff_vertiport_id`, `landing_vertiport_id`, `waypoints` (chỉ các điểm trung gian trên lane, không bao gồm vị trí vertiport)
- **Vận tốc**: `segment_speeds[i]` = vận tốc trên đoạn `waypoints[i] → waypoints[i+1]`
- **Thời gian pha dọc**: `t_takeoff` (ascent từ pad lên `waypoints[0]`), `t_land` (descent từ `waypoints[-1]` xuống pad)
- **Thời điểm**: `start_time` (đã duyệt) hoặc `desired_start_time` + `max_wait_time` (request mới)

### 1.2 Đầu ra

`ResolveResult`:
- `approved`: True/False
- `actual_start_time`: thời điểm cất cánh thực tế (None nếu từ chối)
- `delay`: số giây trễ so với `desired_start_time`
- `assigned_pad_id`: pad hạ cánh được chỉ định (None nếu từ chối)
- `reason` + `conflict_details`: giải thích chi tiết

### 1.3 Tiêu chí giải

Tìm **delay nhỏ nhất** ≥ 0 sao cho **tất cả** bốn constraints (C1–C4) đều thỏa mãn.
Nếu delay tối thiểu vượt quá `max_wait_time` → từ chối.

---

## 2. Mô hình hóa dòng thời gian một chuyến bay

### 2.1 Dòng thời gian đầy đủ

```
start_time
    │
    │── [TAKEOFF PHASE] ── t_takeoff giây
    │   Drone cất cánh dọc từ pad lên waypoints[0]
    │   Chiếm vùng không phận (airspace) của vertiport đi
    │
    ├─ start_time + t_takeoff           ← bắt đầu lane
    │
    │── [LANE PHASE] ── Σ(L_k / v_k) giây
    │   Drone bay ngang qua các đoạn waypoints[k] → waypoints[k+1]
    │   Đây là vùng có thể xảy ra xung đột với drone khác cùng lane
    │
    ├─ start_time + t_takeoff + lane_time   ← kết thúc lane (= t_arrive_wpN)
    │
    │── [LANDING PHASE] ── t_land giây
    │   Drone hạ cánh dọc từ waypoints[-1] xuống pad hạ cánh
    │   Chiếm vùng không phận (airspace) của vertiport đến
    │
    └─ start_time + t_takeoff + lane_time + t_land   ← touchdown
       Pad bận thêm occupation_duration giây
```

### 2.2 Ký hiệu viết tắt

```
lane_time   = Σ_{k} (dist(wp_k, wp_{k+1}) / segment_speeds[k])
t_wpN       = start_time + t_takeoff + lane_time    (thời điểm bắt đầu descent)
t_touch     = t_wpN + t_land                        (thời điểm chạm đất)
```

**Lý do tách `t_takeoff` và `t_land` ra khỏi `segment_speeds`:**
Hai pha này là chuyển động **dọc** (vertical), vật lý hoàn toàn khác với pha ngang (horizontal) trên lane.
Dùng tốc độ lane để tính thời gian pha dọc sẽ cho kết quả sai và không có ý nghĩa vật lý.

---

## 3. Xác định nguồn xung đột

### 3.1 Quan sát vật lý

Khi một drone mới muốn bay, nó có thể gây ra va chạm ở **ba vùng không gian** khác nhau:

| Vùng | Vật lý | Constraint |
|------|--------|------------|
| Lane ngang (waypoint-to-waypoint) | Hai drone cùng segment, quá gần nhau | **C1** |
| Không phận thẳng đứng phía trên vertiport đi | Chỉ một drone được ascent tại một thời điểm | **C2** |
| Không phận thẳng đứng phía trên vertiport đến | Chỉ một drone được descent tại một thời điểm | **C3** |
| Mặt pad hạ cánh | Drone trước chưa rời pad, drone sau không hạ được | **C4** |

### 3.2 Quan sát quan trọng về C2 và C3

C2 và C3 dùng **cùng một pool** airspace windows. Pool này gộp cả:
- Cửa sổ cất cánh của **tất cả** drone đang bay lên từ vertiport đó
- Cửa sổ hạ cánh của **tất cả** drone đang bay xuống vertiport đó

→ Tự động phát hiện conflict **cất cánh vs hạ cánh** tại cùng vertiport, không cần code riêng.

---

## 4. Công thức hóa từng constraint

### 4.1 C1 — Phân tách trên lane

#### Xuất phát điểm

Hai drone đang đi trên cùng một đoạn `A → B` theo cùng chiều. Gọi:
- **Lead** (drone đi trước, đã duyệt): vận tốc `v_j`
- **Follower** (drone mới): vận tốc `v_i`
- `h` = khoảng cách thời gian: lead vào segment sớm hơn follower `h` giây

Tại thời điểm follower vào đầu segment (t = 0 theo đồng hồ follower):
- Follower ở vị trí 0
- Lead ở vị trí `v_j * h` (đã đi được `h` giây trước đó)

#### Diễn biến khoảng cách

Tại thời điểm `t` (tính từ khi follower vào):
```
Vị trí follower  = v_i * t
Vị trí lead      = v_j * h + v_j * t
Khoảng cách      = G(t) = v_j * h - (v_i - v_j) * t
```

Nếu `v_i > v_j` (follower nhanh hơn), khoảng cách **thu hẹp dần**.
Điểm tệ nhất là khi follower rời khỏi segment (t = L/v_i):

```
G_min = v_j * h + v_j * (L/v_i) - L
      = v_j * h - L * (v_i - v_j) / v_i
```

#### Ràng buộc

Yêu cầu `G_min >= MSD + body_len_j` (MSD = minimum separation distance, body_len_j = chiều dài thân drone lead):

```
v_j * h - L * (v_i - v_j) / v_i  >=  MSD + body_len_j

v_j * h  >=  MSD + body_len_j  +  L * (v_i - v_j) / v_i

         h_min  =  (MSD + body_len_j) / v_j
                   + max(0, v_i - v_j) * L / (v_i * v_j)
```

Đây chính là `h_min_full()` trong `scrp/headway.py`, được tái sử dụng trực tiếp.

**Hai số hạng có ý nghĩa riêng biệt:**
- **Số hạng 1** `(MSD + body_j) / v_j`: khoảng cách thời gian tối thiểu ngay tại entry, đảm bảo spatial gap = MSD + body_j tại điểm vào. Chia cho `v_j` vì lead dùng tốc độ `v_j` để tạo ra khoảng cách không gian `v_j * h`.
- **Số hạng 2** `(v_i - v_j) * L / (v_i * v_j)`: bù trừ cho việc gap thu hẹp khi follower nhanh hơn. Bằng 0 khi `v_i <= v_j`.

#### Áp dụng trong code

Với mỗi segment chung giữa new request và approved plan, tính:

```
time_before_seg_new  = t_takeoff + Σ(lane_segs trước segment này)
appr_entry           = approved.start_time + time_before_seg_approved
new_entry            = desired_start + delay + time_before_seg_new
new_exit             = new_entry + L / v_new
```

Kiểm tra hai trường hợp an toàn:

| Trường hợp | Điều kiện an toàn | Ý nghĩa |
|------------|-------------------|---------|
| **New đi trước** | `new_exit + h_lead ≤ appr_entry` | New ra khỏi B trước khi approved vào A, với đủ buffer |
| **New đi sau** | `new_entry ≥ appr_entry + h_follow` | New vào A đủ muộn sau khi approved đã vào |

Nếu không thỏa cả hai → conflict → tính delay cần thiết để đạt trường hợp "New đi sau":

```
required_entry = appr_entry + h_follow
needed_delay   = required_entry - time_before_seg_new - desired_start_time
```

---

### 4.2 C2 — Không phận cất cánh

#### Xuất phát điểm

Phía trên mỗi vertiport có một **vùng không phận dọc** (vertical corridor) chỉ cho phép **một drone** sử dụng tại một thời điểm, vì:
- Không gian hẹp, các drone không thể tránh nhau khi bay thẳng đứng
- Cả ascent lẫn descent đều dùng chung corridor này

#### Window và pool

Với mỗi kế hoạch đã duyệt:
```
Takeoff window:  [start_time,       start_time + t_takeoff]  (nếu xuất phát tại vertiport này)
Descent window:  [t_wpN,            t_wpN + t_land]           (nếu hạ cánh tại vertiport này)
```

**Pool** = union của tất cả các windows trên từ mọi kế hoạch đã duyệt.

#### Điều kiện và công thức

New request muốn cất cánh lúc `t_dep = desired_start + delay`.  
Takeoff window của new: `[t_dep, t_dep + t_takeoff]`.

Điều kiện: window này **không overlap** với bất kỳ window nào trong pool.

Hai windows `[s, e)` và `[t, t + dur)` overlap khi:
```
s < (t + dur)   AND   e > t
```

Thuật toán `_advance_past_conflicts(t, dur, pool)` tìm `t_free ≥ t` nhỏ nhất mà `[t_free, t_free+dur)` không overlap ai:

```
for _ in range(len(pool) + 1):
    conflicts = {(s,e) ∈ pool : s < t+dur AND e > t}
    if not conflicts: return t
    t = max(e for (s,e) in conflicts)   # nhảy qua window xa nhất
```

**Tại sao nhảy đến `max(e)`?** Vì mọi window trong `conflicts` đều overlap với `[t, t+dur)`. Sau khi nhảy đến `max(e)`, ta đảm bảo đã thoát khỏi **tất cả** các window đó. Vòng lặp giới hạn `len(pool)+1` bước vì mỗi bước tiêu thụ ít nhất một window.

Delay cần thiết:
```
t_dep_free     = advance_past_conflicts(t_dep, t_takeoff, pool_at_A)
required_delay = max(current_delay, t_dep_free - desired_start_time)
```

---

### 4.3 C3 — Không phận hạ cánh

#### Giống C2 nhưng điểm thời gian là downstream

Descent window của new request: `[t_wpN, t_wpN + t_land]` tại vertiport đến.

```
t_wpN = desired_start + delay + t_takeoff + lane_time
```

`t_wpN` phụ thuộc vào `delay`, nên sau khi tìm được `t_wpN_free` cần **back-calculate** về delay:

```
t_wpN_free   = advance_past_conflicts(t_wpN, t_land, pool_at_B)

Back-calculate:
    t_wpN_free  =  desired_start + required_delay + t_takeoff + lane_time
    required_delay = t_wpN_free - t_takeoff - lane_time - desired_start_time
```

**Tại sao cần back-calculate?**
C2 xử lý departure: pushing `t_dep` về phía trước **trực tiếp** thay đổi delay.
C3 xử lý arrival: `t_wpN` là điểm cuối của chuỗi tính toán, phải đảo ngược chuỗi đó để ra delay.

#### Pool dùng chung với C2

Cả C2 và C3 đều gọi `_all_airspace_windows(vertiport_id)` với cùng pool, chứa cả takeoff lẫn descent windows. Điều này có nghĩa:
- Một drone đang hạ cánh xuống B **cũng** chặn một drone khác muốn cất cánh từ B, và ngược lại.

---

### 4.4 C4 — Khả dụng của pad

#### Xuất phát điểm

Sau khi chạm đất tại `t_touch`, drone chiếm pad trong `pad.occupation_duration` giây. Các drone tiếp theo phải đợi pad giải phóng.

#### Window mỗi pad

Với mỗi kế hoạch đã duyệt có `assigned_pad_id = pad_id`:
```
occupation_window = [t_touch_approved,  t_touch_approved + pad.occupation_duration)
```

Chỉ kế hoạch có `assigned_pad_id` trùng khớp mới được tính. Nếu `assigned_pad_id = None` thì không đóng góp window.

#### Tìm pad tốt nhất

Với mỗi pad tại vertiport hạ cánh, tính `t_touch_free` (thời điểm sớm nhất pad rảnh):

```
t_touch = desired_start + delay + t_takeoff + lane_time + t_land

for _ in range(len(occ_windows) + 1):
    busy = {(s,e) ∈ occ_windows : s <= t_touch < e}
    if not busy: break
    t_touch = min(e for (s,e) in busy)    # nhảy đến hết window bận gần nhất
```

**Lý do dùng `min(e)` thay vì `max(e)` (khác với airspace):**
- Airspace: window mới phải bắt đầu SAU khi tất cả conflicting windows kết thúc.
- Pad: touchdown là một **điểm** (không phải interval), nên chỉ cần nhảy qua window đang chứa điểm đó.

Back-calculate sang delay:
```
required_start = t_touch_free - t_land - lane_time - t_takeoff
pad_delay      = max(current_delay, required_start - desired_start_time)
```

Chọn pad có `pad_delay` nhỏ nhất → `assigned_pad_id`.

---

## 5. Sự ràng buộc giữa các constraints (Coupling)

### 5.1 Tại sao constraints bị coupled?

Bốn constraints **không độc lập**. Delay từ constraint này thay đổi giá trị đầu vào của constraint khác:

```
delay  ─[C1]→  delay'
                │
                └─ t_dep = desired + delay'
                           │
                           ├─[C2]→  t_dep_free → delay''
                           │
                           └─ t_wpN = t_dep_free + t_takeoff + lane_time
                                      │
                                      ├─[C3]→  t_wpN_free → delay'''
                                      │
                                      └─ t_touch = t_wpN_free + t_land
                                                   │
                                                   └─[C4]→  pad_delay → delay''''
```

### 5.2 Vì sao giải tuần tự một lần không đủ?

Ví dụ: C4 đẩy delay từ 30 lên 130. Nhưng với delay = 130, drone mới **chồng lên** kế hoạch P7 trên lane — một conflict **mới** xuất hiện mà ở delay = 30 không tồn tại. C1 phải chạy lại để phát hiện điều này.

Tổng quát: bất kỳ constraint nào tăng delay đều có thể làm lộ ra vi phạm mới tại constraint trước nó trong thứ tự kiểm tra.

---

## 6. Thuật toán hội tụ (Fixed-point iteration)

### 6.1 Ý tưởng

Chạy lặp lại vòng C1→C2→C3→C4 cho đến khi delay ổn định (thay đổi < 0.01 giây):

```python
delay = 0.0
for _ in range(MAX_ITER):
    prev  = delay
    delay = C1(request, delay)
    delay = C2(request, delay, "departure")
    delay = C3(request, delay, "arrival")
    delay, pad_id = C4(request, delay)
    if |delay - prev| < 0.01:
        break
```

### 6.2 Tại sao đảm bảo hội tụ?

Mỗi hàm constraint `f(delay)` thỏa mãn hai tính chất:
1. **Đơn điệu tăng (monotone)**: `f(d) ≥ d` với mọi `d` — hàm chỉ giữ nguyên hoặc tăng delay, không bao giờ giảm.
2. **Bị chặn trên (bounded)**: với một tập hữu hạn windows, `f(d)` bị chặn bởi `max(window end times)`.

Kết hợp 4 constraints vào hàm tổng hợp `F(d) = C4(C3(C2(C1(d))))`:
- F cũng đơn điệu tăng và bị chặn trên
- Định lý Knaster–Tarski đảm bảo tồn tại **điểm bất động** (fixed point) `d* = F(d*)`
- Dãy `d_0 = 0, d_{k+1} = F(d_k)` là dãy tăng và bị chặn → hội tụ đến `d*`

Trong thực tế: số lượng windows hữu hạn và nhỏ, thường hội tụ trong 2–4 vòng lặp.

### 6.3 Ví dụ 3 vòng lặp (trường hợp cascade)

```
Cấu hình:
  • P1, P2, P3 cất cánh từ A tại t=0,10,20 → block airspace A đến t=30
  • P5 hạ cánh B: touchdown=34, padB1 bận [34,94)
  • P6 hạ cánh B: touchdown=94, padB1 bận [94,154)
  • P7 cất cánh A tại t=130, cùng lane, window [130,140]

Vòng 1:
  C2: t_dep=0 → bị P1,P2,P3 → t_dep_free=30 → delay=30
  C4: t_touch=54 → bị P5[34,94) → t=94 → bị P6[94,154) → t=154 free
      required_start=130 → delay=130

Vòng 2:
  C1: new_entry=140, P7 appr_entry=140 → conflict → delay=131
  C2: t_dep=131, [131,141] overlap P7[130,140] → t_dep_free=140 → delay=140

Vòng 3:
  C1: new_entry=150 > P7 appr_entry+h=141 → safe
  C2: [140,150] không overlap [130,140] → safe
  C3, C4: không thay đổi
  → |delay - prev| = 0 < 0.01 → HỘI TỤ tại delay=140
```

**Cơ chế cascade:** C4 đẩy delay lớn → làm lộ conflict C1 mới → C1 đẩy thêm → C2 lại cần đẩy thêm → ổn định.

---

## 7. Giải batch nhiều request

### 7.1 Vấn đề với batch

Khi có nhiều request đồng thời, thứ tự giải ảnh hưởng đến kết quả. Ví dụ: hai drone cùng muốn chiếm padB1 — ai được giải trước sẽ chiếm pad, người còn lại phải chờ.

### 7.2 Nguyên tắc priority-based batch

```
1. Sắp xếp requests theo (priority ASC, desired_start_time ASC)
   Priority thấp hơn (số nhỏ hơn) = ưu tiên cao hơn

2. Pool ban đầu = approved_plans đã có

3. for req in sorted_requests:
       result = ConflictResolver(vertiports, pool).resolve(req)
       if result.approved:
           # Bổ sung vào pool: drone mới trở thành "đã duyệt"
           # cho tất cả request có độ ưu tiên thấp hơn
           pool.append(ApprovedPlan từ result)

4. Trả về kết quả theo thứ tự gốc (không theo thứ tự ưu tiên)
```

### 7.3 Tại sao pool phải tăng dần?

Nếu giải tất cả request song song trên cùng pool ban đầu → hai request ưu tiên cao và thấp có thể cùng được phân bổ padB1 tại cùng thời điểm → conflict sau khi duyệt.

Bổ sung pool tuần tự đảm bảo: request ưu tiên cao đã "khóa" resource trước khi request ưu tiên thấp được xét.

---

## 8. Tổng hợp kết quả trả về

### 8.1 Quy trình tổng hợp

```
                    ┌─────────────────────────────────────────┐
                    │           ConflictResolver.resolve()     │
                    │                                          │
  desired_start ──▶ │  delay = 0                               │
  max_wait_time     │                                          │
  flight_path  ──▶ │  repeat until converged:                  │
  t_takeoff         │      delay = C1(delay)   → conflict_details│
  t_land            │      delay = C2(delay)   → conflict_details│
  segment_speeds    │      delay = C3(delay)   → conflict_details│
                    │      delay, pad = C4(delay) → details    │
                    │                                          │
                    │  if delay > max_wait_time:               │
                    │      return REJECTED                     │
                    │  else:                                   │
                    │      return APPROVED                     │
                    └─────────────────────────────────────────┘
```

### 8.2 Kết quả trả về

| Trường | Khi duyệt | Khi từ chối |
|--------|-----------|-------------|
| `approved` | `True` | `False` |
| `actual_start_time` | `desired_start + delay` | `None` |
| `delay` | delay tối thiểu | delay tối thiểu cần (> max_wait) |
| `assigned_pad_id` | pad được chỉ định | `None` |
| `reason` | `"Approved"` hoặc `"Approved with Xs delay"` | Giải thích lý do |
| `conflict_details` | Danh sách từng conflict gặp phải | Như vậy |

### 8.3 Bảng tóm tắt toàn bộ

| Constraint | Nguồn gốc | Công thức cốt lõi | Back-calculate cần? |
|------------|-----------|-------------------|---------------------|
| C1 Lane | Khoảng cách không gian tối thiểu trên segment | `h_min = (MSD+body)/v_j + max(0,v_i-v_j)*L/(v_i*v_j)` | Không (tính trực tiếp ra delay) |
| C2 Airspace A | Mutual exclusion của vertical corridor | `advance_past_conflicts(t_dep, t_takeoff, pool)` | Không (`t_dep_free - desired` = delay) |
| C3 Airspace B | Mutual exclusion của vertical corridor | `advance_past_conflicts(t_wpN, t_land, pool)` | **Có**: `t_wpN_free - t_takeoff - lane_time - desired` |
| C4 Pad | Occupation window sau touchdown | Tìm `t_touch_free` bằng cách nhảy qua busy windows | **Có**: `t_touch_free - t_land - lane_time - t_takeoff - desired` |

---

*File này mô tả thiết kế và cách dẫn xuất của module `scrp_simple`.
Xem source code tại `resolver.py` và `models.py` để biết chi tiết cài đặt.
Xem test tại `tests/test_simple_resolver.py` để biết các trường hợp kiểm thử.*
