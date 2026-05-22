# time-departure

# Strategic Conflict Resolution Problem (SCRP)
---

## 1. Tổng quan

### Vai trò của service

Service nhận một **Flight Intention (FI)** mới từ một drone, kiểm tra xung đột với tập các kế hoạch bay đã được phê duyệt, và trả về thời điểm cất cánh tối ưu t_dep* hoặc từ chối với lý do cụ thể.

Service đóng vai trò **Air Traffic Controller**: không lập route, không điều chỉnh vận tốc — chỉ quyết định **khi nào** drone được phép thực hiện route đã nộp.

### Phạm vi của service (trong bản này)

- **Bao gồm:** tính t_dep* thỏa mãn ba ràng buộc xung đột (headway trong tube, junction, landing slot) và ràng buộc năng lượng (SoC).
- **Bao gồm:** cơ chế Soft Reservation — trạng thái trung gian khi chờ operator xác nhận.
- **Không bao gồm:** điều chỉnh vận tốc, buffer zone, in-flight intervention, thay đổi route.

---

## 2. Mô hình dữ liệu đầu vào

### 2.1 Waypoint và Lane

Mạng hành lang được biểu diễn bởi tập M waypoint được quy hoạch cố định trước. Một **lane** là một dãy waypoint liên tiếp, mỗi đoạn giữa hai waypoint liên tiếp gọi là **segment**.

Waypoint: điểm 3D đã được định nghĩa trong hệ thống
  id        : str           — định danh duy nhất
  position  : (x, y, z)    — tọa độ [m]
Segment: đoạn nối hai waypoint liên tiếp trong một lane
  p_start   : Waypoint
  p_end     : Waypoint
  length    : float         — ||p_end - p_start|| [m]
  v_min     : float         — vận tốc tối thiểu cho phép [m/s]
  v_max     : float         — vận tốc tối đa cho phép [m/s]
Lane: dãy waypoint tạo thành một hành lang bay
  id        : str
  waypoints : List[Waypoint]          — [P1, P2, ..., Pm]
  segments  : List[Segment]           — tự suy ra từ waypoints
### 2.2 Flight Intention (FI)

FlightIntention:
  drone_id      : str
  uav_type      : str               — 'A' | 'B' | 'C'
  lane          : Lane              — lane đã chọn (cố định, không thay đổi)
  v_waypoints   : List[float]       — vận tốc tại mỗi waypoint [m/s]
                                      len = len(lane.waypoints)
  t_des         : float             — thời điểm mong muốn cất cánh [s, Unix]
  SoC_0         : float             — trạng thái pin [0.0..1.0]
  C_bat         : float             — dung lượng pin [Wh]
  P_hover       : float             — công suất hover [W]
  priority      : int               — 1 | 2 | 3
  operator_id   : str
**Lưu ý:** v_waypoints[k] là vận tốc trên đoạn từ waypoints[k] đến waypoints[k+1]. Phần tử cuối (v_waypoints[-1]) không dùng đến (drone đã đến đích).

### 2.3 Approved Plan (kế hoạch đã phê duyệt)

ApprovedPlan:
  drone_id      : str
  uav_type      : str
  lane          : Lane
  waypoint_times: List[(Waypoint, float)]   — [(P1, t1), (P2, t2), ..., (Pm, tm)]
                                              t_k = thời điểm tuyệt đối đến P_k [s]
  t_land        : float                     — thời điểm hạ cánh [s]
  pad_id        : str
  slot_index    : int
  status        : str                       — 'SOFT_RESERVED' | 'COMMITTED'
  expires_at    : float | None              — deadline confirm (chỉ khi SOFT_RESERVED)
### 2.4 Vertiport State

VertiportState:
  vertiport_id  : str
  pads          : List[Pad]
  slot_duration : float             — thời gian mỗi slot [s], mặc định 30s
  slots         : Dict[Tuple[str, int], str | None]
                                    — (pad_id, slot_index) → drone_id | None
                                      None = trống
                                      drone_id = SOFT_RESERVED hoặc COMMITTED
  slot_status   : Dict[Tuple[str, int], str]
                                    — (pad_id, slot_index) → 'FREE' | 'SOFT' | 'COMMITTED'
Pad:
  id                : str
  compatible_types  : List[str]     — loại drone được dùng pad này
### 2.5 System State (toàn bộ trạng thái hệ thống)

SystemState:
  approved_plans    : List[ApprovedPlan]    — bao gồm cả SOFT và COMMITTED
  vertiport_state   : VertiportState
  t_now             : float                 — thời điểm hiện tại [s]
  msd_matrix        : Dict[Tuple[str,str], float]
                                            — MSD theo cặp (uav_type_i, uav_type_j) [m]
  body_length       : Dict[str, float]      — chiều dài thân drone theo uav_type [m]
---

## 3. Công thức toán học cốt lõi

### 3.1 Thời điểm đến từng waypoint

Với FI mới cất cánh lúc t_dep, thời điểm đến waypoint P_k:

t_arrival(P_k, t_dep) = t_dep + sum( length(seg_j) / v_waypoints[j]
                                     for j in 0..k-1 )
Thời điểm hạ cánh:

t_land(t_dep) = t_dep + sum( length(seg_j) / v_waypoints[j]
                              for j in 0..M-2 )
### 3.2 Headway tối thiểu

Drone mới (q_i) vào sau drone đã reserved (q_j) trên cùng segment:

h_min(q_i, q_j, segment) = ( MSD(q_i, q_j) + L_body(q_j) ) / v_min(q_i, segment)
trong đó:
- MSD(q_i, q_j) từ msd_matrix
- L_body(q_j) từ body_length
- v_min(q_i, segment) = segment.v_min

### 3.3 Điều kiện an toàn trên đoạn giữa hai waypoint

**Bổ sung quan trọng so với headway đơn thuần** (đã chứng minh toán học):

Để đảm bảo khoảng cách an toàn trên toàn đoạn seg_k = P_k → P_{k+1}, headway tại P_k phải thỏa mãn:

h_min_full(q_i, q_j, seg_k) = (MSD + L_body(q_j)) / v_fast
                              + |v_i - v_j| * L_k / v_fast^2
trong đó:
- v_fast = max(v_i, v_j) — vận tốc của drone nhanh hơn
- v_i, v_j — vận tốc của drone mới và drone đã reserved trên segment k
- L_k = length(seg_k)

Khi v_i == v_j: correction term = 0, công thức rút gọn về h_min cơ bản.

**Service phải dùng h_min_full, không dùng h_min cơ bản.**

### 3.4 Ba thành phần delay

**Delta_C1 — Xung đột headway tại waypoints:**

Với mỗi waypoint P_k trong lane mới
và mỗi approved_plan d' cũng đi qua P_k:
  tau_k(d') = waypoint_times của d' tại P_k
  Nếu drone mới đến sau d' tại P_k:
    needed = tau_k(d') + h_min_full(q_new, q_d', seg_k) - t_arrival(P_k, t_des)
    Delta_C1 = max(Delta_C1, max(0, needed))
**Delta_C2 — Xung đột tại junction (waypoint giao nhau):**

Junction là waypoint thuộc đồng thời nhiều lane khác nhau. Hai drone không được ở trong vùng junction đồng thời.

Delta_t_J = diameter_junction / v_min_global
Với mỗi junction J trên lane mới
và mỗi approved_plan d' cũng đi qua J:
  tau_J(d') = thời điểm d' đến J
  gap = t_arrival(J, t_des) - tau_J(d')
  IF abs(gap) < Delta_t_J:
    IF t_arrival(J, t_des) >= tau_J(d'):
      needed = tau_J(d') + Delta_t_J - t_arrival(J, t_des)
    ELSE:
      needed = 0   (drone mới đến trước — không cần delay)
    Delta_C2 = max(Delta_C2, max(0, needed))
**Delta_C3 — Xung đột landing slot:**

t_land_des = t_land(t_des)
free_slots = các slot còn FREE hoặc SOFT tại vertiport đích
             phù hợp với uav_type của drone mới
s* = slot sớm nhất trong free_slots sao cho t_slot_start(s*) >= t_land_des
Nếu s* tồn tại:
    Delta_C3 = max(0, t_slot_start(s*) - t_land_des)
Nếu không có slot nào:
    Delta_C3 = INF  → REJECT("no_slot_available")
**Nghiệm dạng đóng:**

t_dep* = t_des + max(0, Delta_C1, Delta_C2, Delta_C3)
### 3.5 Ràng buộc năng lượng / State-of-Charge (C4)

**Mục đích:** Đảm bảo drone còn đủ pin sau khi hoàn thành toàn bộ nhiệm vụ (bay qua lane + chờ slot hạ cánh).  Nếu SoC dự kiến sau chuyến bay thấp hơn ngưỡng tối thiểu, FI bị từ chối ngay lập tức.

**Cơ sở vật lý:**  Năng lượng điện tiêu thụ = Công suất × Thời gian.  Đơn vị dùng nhất quán là Watt-giờ (Wh), vì dung lượng pin thường đo bằng Wh (hoặc Ah × V).  Giảm SoC tỉ lệ thuận với năng lượng tiêu thụ:

    ΔSoC = E_tiêu_thụ [Wh] / C_bat [Wh]

#### Bảng biến

| Ký hiệu | Tên đầy đủ | Đơn vị | Ý nghĩa |
|---|---|---|---|
| `SoC_0` | State of Charge (initial) | [0..1] | Phần dung lượng pin còn lại tại thời điểm cất cánh, do operator khai báo |
| `C_bat` | Battery Capacity | Wh | Tổng năng lượng sử dụng được của pin |
| `P_hover` | Hover Power | W | Công suất điện tiêu thụ khi hover tại chỗ (không có chuyển động ngang), do operator khai báo |
| `CRUISE_POWER_FACTOR` | Cruise Power Factor | — | Hệ số nhân từ P_hover sang P_cruise; mặc định 0.8 (cruise tiết kiệm hơn hover ~20 % ở tốc độ hành trình bình thường; < 1.0 là điển hình, > 1.0 chỉ ở tốc độ rất cao) |
| `P_cruise` | Cruise Power | W | Công suất bay bằng ước tính; `P_cruise = P_hover × CRUISE_POWER_FACTOR` |
| `L_k` | Segment Length (k) | m | Khoảng cách Euclid giữa hai waypoint liên tiếp của đoạn k |
| `v_k` | Cruise Velocity (segment k) | m/s | Vận tốc bay trên đoạn k, do operator khai báo trong `v_waypoints[k]` |
| `t_k` | Travel Time (segment k) | s | Thời gian di chuyển qua đoạn k: `t_k = L_k / v_k` |
| `E_cruise` | Cruise Energy | Wh | Tổng năng lượng tiêu thụ trong giai đoạn bay qua toàn bộ lane |
| `t_arrive_at_dest` | Arrival Time at Destination Airspace | s Unix | Thời điểm drone hoàn thành lane traversal và vào vùng tiếp cận vertiport (trước khi bắt đầu pha hạ cánh) |
| `t_slot_start(s*)` | Landing Slot Start Time | s Unix | Thời điểm slot hạ cánh được phân công bắt đầu mở |
| `t_hover_wait` | Hover Wait Duration | s | Thời gian hover chờ slot: `max(0, t_slot_start − t_arrive_at_dest)` |
| `E_hover_wait` | Hover-Wait Energy | Wh | Năng lượng tiêu thụ trong giai đoạn hover chờ slot |
| `E_total` | Total Mission Energy | Wh | Tổng năng lượng cả hai giai đoạn: `E_cruise + E_hover_wait` |
| `SoC_remaining` | Remaining State of Charge | [0..1] | SoC dự kiến sau khi hạ cánh |
| `SOC_MIN` | Minimum State of Charge | [0..1] | Ngưỡng tối thiểu chấp nhận được; mặc định 0.20 (còn 20 % pin) |

#### Các bước tính

**Bước 1 — Tính Cruise Power (P_cruise):**

Với drone đa cánh quạt, bay bằng ở tốc độ hành trình vừa phải **tiêu điện ít hơn** hover tại chỗ.  Lý do: khi hover, rotor phải liên tục tăng tốc luồng khí đã bị nhiễu động từ vòng quay trước (induced velocity cao); khi bay bằng, rotor đón không khí sạch mỗi vòng quay, giảm induced velocity và do đó giảm công suất cảm ứng.  Ở tốc độ 10–15 m/s, tổng công suất điện thường chỉ bằng 70–85 % hover.  Chỉ ở tốc độ rất cao, lực cản khí động học (parasite drag) mới đẩy công suất vượt trở lại mức hover.

Khi không có mô hình khí động học chi tiết, ta xấp xỉ:

    P_cruise = P_hover × CRUISE_POWER_FACTOR          (W)

với `CRUISE_POWER_FACTOR < 1.0` ở tốc độ hành trình bình thường (mặc định 0.8).

**Bước 2 — Tính E_cruise (năng lượng pha bay qua lane):**

Áp dụng công thức cơ bản E = P × t cho từng đoạn lane, rồi cộng tổng:

    E_cruise = Σ_k  P_cruise × (L_k / v_k) / 3600     (Wh)

  - `L_k / v_k` = thời gian di chuyển đoạn k [s]
  - chia 3600 để chuyển giây → giờ (W × h = Wh)

**Bước 3 — Tính thời gian hover chờ slot (t_hover_wait):**

    t_land_new         = t_land(t_dep*)
    t_arrive_at_dest   = t_land_new − t_land_estimated
    t_hover_wait       = max(0, t_slot_start(s*) − t_arrive_at_dest)

  - Nếu drone đến trước khi slot mở: `t_hover_wait > 0` → drone hover chờ.
  - Nếu drone đến sau khi slot đã mở: `t_hover_wait = 0` → không cần hover thêm.
  - `t_land_estimated` (thời lượng pha hạ cánh) được trừ ra để không mô hình hóa drone như đang hover trong khi thực ra đã đang thực hiện thao tác hạ cánh.

**Bước 4 — Tính E_hover_wait (năng lượng pha hover chờ):**

    E_hover_wait = P_hover × (t_hover_wait / 3600)    (Wh)

**Bước 5 — Tính SoC_remaining:**

    E_total       = E_cruise + E_hover_wait
    SoC_remaining = SoC_0 − E_total / C_bat

**Bước 6 — Kiểm tra ràng buộc C4:**

    IF SoC_remaining < SOC_MIN:
        RETURN REJECT("SoC_insufficient")

**Lưu ý về thời tiết:** Trong triển khai hiện tại, P_cruise được tính bằng xấp xỉ `P_hover × CRUISE_POWER_FACTOR` vì không có dữ liệu thời tiết.  Khi có dữ liệu gió, P_cruise có thể được thay bằng hàm `P_cruise(v_k, weather_k)` tính từ blade element theory hoặc dữ liệu thực nghiệm.

---

## 4. Cơ chế Soft Reservation

### Trạng thái tài nguyên

FREE          → tài nguyên hoàn toàn trống
SOFT_RESERVED → đang chờ operator xác nhận (có thời hạn)
COMMITTED     → đã xác nhận, không thể thu hồi
### Quy tắc xử lý

**Khi nhận FI mới:**
- Xử lý SOFT_RESERVED như COMMITTED (pessimistic) — tức là tính Delta_C* với cả SOFT và COMMITTED plans.
- Sau khi tính t_dep* và SoC hợp lệ: tạo reservation mới với status = SOFT_RESERVED.
- Ghi expires_at = t_now + T_RESPONSE_WINDOW.

**Khi operator ACCEPT:**
- Chuyển status từ SOFT_RESERVED → COMMITTED.
- Xóa expires_at.

**Khi operator REJECT:**
- Xóa reservation khỏi approved_plans.
- Giải phóng slot trong vertiport_state.
- Gửi cascade_notify đến các plan có dependency (xem mục 4.1).

**Khi timeout (t_now > expires_at):**
- Theo DEFAULT_TIMEOUT_BEHAVIOR:
  - 'reject' (mặc định, an toàn): tự động reject, giải phóng tài nguyên.
  - 'accept': tự động chuyển sang COMMITTED.

### 4.1 Dependency tracking

PlanDependency:
  plan_id     : str     — plan đang SOFT_RESERVED
  depends_on  : str     — plan khác cũng SOFT_RESERVED mà plan_id đang tránh
Khi plan A bị reject: tìm tất cả plan B có depends_on = A, gửi notification cho operator của B rằng "điều kiện đã thay đổi, có thể có phương án tốt hơn".

---

## 5. Đầu vào và đầu ra của service

### Đầu vào

def resolve_conflict(
    fi: FlightIntention,
    approved_plans: List[ApprovedPlan],    # chỉ các plan trong window thời gian ảnh hưởng
    vertiport_state: VertiportState,
    system_state: SystemState,
    config: SCRPConfig
) -> SCRPResult:
**Lọc approved_plans trước khi truyền vào:**
Caller chỉ cần truyền các plan có khoảng thời gian [t_entry_first, t_land] giao với [t_des - buffer, t_des + max_delay + t_total_flight]. Service không cần tự lọc.

### Đầu ra

@dataclass
class ApproveResult:
    status          : Literal['SOFT_RESERVED']
    t_dep_star      : float                   — thời điểm cất cánh đã điều chỉnh [s]
    t_land_assigned : float                   — thời điểm hạ cánh tương ứng [s]
    pad_id          : str
    slot_index      : int
    waypoint_times  : List[Tuple[str, float]] — [(waypoint_id, t), ...] trajectory đầy đủ
    expires_at      : float                   — deadline confirm [s]
    delay_seconds   : float                   — = t_dep_star - t_des
    delay_source    : str                     — 'C1_headway' | 'C2_junction' | 'C3_slot' | 'none'
    energy_estimate : float                   — E_cruise ước tính [Wh]
    SoC_remaining   : float                   — SoC dự kiến sau hạ cánh

@dataclass
class RejectResult:
    status          : Literal['REJECTED']
    reason          : str                     — 'SoC_insufficient' | 'no_slot_available' |
                                                'no_feasible_t_dep' | 'invalid_fi'
    earliest_possible : float | None          — t_dep sớm nhất có thể (nếu tính được)
    detail          : str                     — mô tả chi tiết lý do

SCRPResult = ApproveResult | RejectResult
---

## 6. Cấu trúc module Python

scrp/
├── models.py          — dataclasses: FlightIntention, ApprovedPlan,
│                        VertiportState, SystemState, SCRPConfig,
│                        ApproveResult, RejectResult
│
├── geometry.py        — tính t_arrival, t_land, length segment
│                        Không có side effects
│
├── headway.py         — h_min_full(q_i, q_j, seg, v_i, v_j)
│                        compute_delta_C1(fi, approved_plans, t_des, state)
│
├── junction.py        — detect_junctions(lane, all_lanes)
│                        compute_delta_C2(fi, approved_plans, t_des, state)
│
├── slot.py            — find_earliest_slot(t_land, vertiport_state, uav_type)
│                        compute_delta_C3(t_land_des, vertiport_state, uav_type)
│
├── energy.py          — compute_E_cruise(fi, config)
│                        compute_SoC_remaining(fi, t_dep_star, t_slot, config)
│
├── reservation.py     — create_soft_reservation(fi, t_dep_star, slot, state)
│                        commit_reservation(plan_id, state)
│                        release_reservation(plan_id, state)
│                        expire_soft_reservations(state, t_now)
│                        cascade_notify(rejected_plan_id, state) → List[str]
│
└── scrp.py            — resolve_conflict(...) → SCRPResult
                         Orchestrates toàn bộ pipeline
---

## 7. Trình tự thực hiện trong resolve_conflict

BƯỚC 1 — Validate đầu vào
  - drone_id có tồn tại không?
  - lane hợp lệ (waypoints liên thông, uav_type compatible với segments)?
  - len(v_waypoints) == len(lane.waypoints)?
  - SoC_0 trong [0, 1]?
  → Nếu lỗi: RETURN RejectResult('invalid_fi')
BƯỚC 2 — Tính Delta_C1 (headway)
  - Với mỗi approved_plan (bao gồm cả SOFT_RESERVED):
    - Tìm waypoints chung giữa lane mới và lane của plan đó
    - Với mỗi waypoint chung P_k:
      - Tính t_arrival(P_k, t_des) cho FI mới
      - So sánh với tau_k của plan đó
      - Tính needed delay nếu vi phạm h_min_full
  - Delta_C1 = max của tất cả needed delays
BƯỚC 3 — Tính Delta_C2 (junction)
  - Detect junctions: waypoints xuất hiện trong >= 2 lanes khác nhau
  - Với mỗi junction trên lane mới:
    - Tìm approved_plans cũng đi qua junction đó
    - Tính needed delay theo điều kiện |gap| >= Delta_t_J
  - Delta_C2 = max của tất cả needed delays
BƯỚC 4 — Tính t_dep* sơ bộ
  t_dep_prelim = t_des + max(0, Delta_C1, Delta_C2)
BƯỚC 5 — Tính Delta_C3 (landing slot)
  t_land_prelim = t_land(t_dep_prelim)
  (s*, Delta_C3) = find_earliest_slot(t_land_prelim, vertiport_state, fi.uav_type)
  Nếu Delta_C3 == INF: RETURN RejectResult('no_slot_available')
BƯỚC 6 — Tính t_dep* cuối cùng
  delay_total = max(0, Delta_C1, Delta_C2, Delta_C3)
  t_dep_star  = t_des + delay_total
  Xác định delay_source:
    IF Delta_C3 == delay_total: 'C3_slot'
    ELIF Delta_C2 == delay_total: 'C2_junction'
    ELIF Delta_C1 == delay_total: 'C1_headway'
    ELSE: 'none'
BƯỚC 7 — Kiểm tra SoC (C4)
  t_land_final    = t_land(t_dep_star)
  t_hover_wait    = max(0, t_slot_start(s*) - t_land_final)
  E_cruise        = compute_E_cruise(fi, config)
  SoC_remaining   = SoC_0 - (E_cruise + P_hover * t_hover_wait / 3600) / C_bat
  Nếu SoC_remaining < SOC_MIN:
    earliest = compute_earliest_with_charging(fi, state, config)
    RETURN RejectResult('SoC_insufficient', earliest_possible=earliest)
BƯỚC 8 — Tạo Soft Reservation (atomic)
  waypoint_times = [(wp.id, t_arrival(wp, t_dep_star)) for wp in fi.lane.waypoints]
  new_plan = ApprovedPlan(
    drone_id       = fi.drone_id,
    uav_type       = fi.uav_type,
    lane           = fi.lane,
    waypoint_times = waypoint_times,
    t_land         = t_land_final,
    pad_id         = pad_id_of(s*),
    slot_index     = slot_index_of(s*),
    status         = 'SOFT_RESERVED',
    expires_at     = t_now + T_RESPONSE_WINDOW
  )
  state.approved_plans.append(new_plan)
  mark_slot(s*, fi.drone_id, 'SOFT', state.vertiport_state)
BƯỚC 9 — Trả về kết quả
  RETURN ApproveResult(
    status          = 'SOFT_RESERVED',
    t_dep_star      = t_dep_star,
    t_land_assigned = t_land_final,
    pad_id          = pad_id_of(s*),
    slot_index      = slot_index_of(s*),
    waypoint_times  = waypoint_times,
    expires_at      = t_now + T_RESPONSE_WINDOW,
    delay_seconds   = t_dep_star - t_des,
    delay_source    = delay_source,
    energy_estimate = E_cruise,
    SoC_remaining   = SoC_remaining
  )
---

## 8. Config

@dataclass
class SCRPConfig:
    SOC_MIN                 : float = 0.20    # SoC tối thiểu khi hạ cánh
    SLOT_DURATION_SEC       : float = 30.0    # giây/slot
    T_RESPONSE_WINDOW_SEC   : float = 120.0   # giây, pre-flight
    JUNCTION_DIAMETER_M     : float = 20.0    # mét, dùng khi không có metadata
    DEFAULT_TIMEOUT_BEHAVIOR: str   = 'reject'  # 'reject' | 'accept'
    CRUISE_POWER_FACTOR     : float = 1.2     # hệ số tính E_cruise xấp xỉ
    MAX_ACCEPTABLE_DELAY_SEC: float = 1800.0  # 30 phút, ngưỡng delay tối đa
---

## 9. Yêu cầu kỹ thuật Python

- Python 3.11+
- Dùng dataclasses và type hints đầy đủ
- Tất cả thời gian là float Unix seconds
- Không dùng external solver (scipy được phép cho tính toán phụ trợ)
- resolve_conflict không có side effects — chỉ đọc state, không ghi
- create_soft_reservation là hàm riêng biệt, có side effects, được gọi sau khi caller xác nhận kết quả
- Mỗi module có unit tests riêng trong tests/
- resolve_conflict phải chạy < 200ms với 500 approved_plans và lane 20 waypoints

### Phân tách side effects

# Cách sử dụng đúng:
result = resolve_conflict(fi, plans, vertiport, state, config)
if isinstance(result, ApproveResult):
    # Caller quyết định có tạo reservation không
    updated_state = create_soft_reservation(result, state)
    # Gửi result về operator để chờ confirm/reject
---

## 10. Test cases tối thiểu cần có

test_no_conflict:
    Không có approved_plans → Delta_C1=0, Delta_C2=0, Delta_C3=0
    t_dep* == t_des
test_headway_single:
    1 approved_plan trên cùng lane, vào trước 10s
    h_min_full > 10s → Delta_C1 = h_min_full - 10s
test_headway_same_velocity:
    v_i == v_j → correction term = 0
    h_min_full == h_min_basic
test_headway_different_velocity:
    v_i > v_j (drone mới nhanh hơn) → correction term > 0
    h_min_full > h_min_basic
test_junction_conflict:
    Hai lanes giao nhau tại waypoint chung
    Drone đến cùng lúc → Delta_C2 = Delta_t_J
test_slot_occupied:
    Tất cả slots trong window đầu tiên đã COMMITTED
    Delta_C3 > 0, t_dep* bị đẩy muộn
test_SoC_insufficient:
    SoC_0 thấp, route dài → SoC_remaining < 0.20
    RETURN RejectResult('SoC_insufficient')
test_soft_reservation_treated_as_committed:
    1 plan SOFT_RESERVED trên cùng lane
    Delta_C1 được tính như plan đó đã COMMITTED
test_soft_reservation_timeout:
    expires_at < t_now → reservation được giải phóng trước khi tính
test_closed_form_correctness:
    t_dep* = t_des + max(Delta_C1, Delta_C2, Delta_C3)
    Verify bằng simulation: sau t_dep*, không có vi phạm nào
Init project
