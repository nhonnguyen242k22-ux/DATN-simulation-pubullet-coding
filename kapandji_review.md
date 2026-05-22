# Review — kapandji_test.py

---

## 1. Luồng chạy tổng thể

```
start
  │
  ├─ exec fix_urdf.py          → tạo assembly_1_fixed.urdf
  ├─ load poses.json           → neutral_pose (toàn bộ joint = 0.0)
  ├─ p.connect(GUI)
  ├─ p.loadURDF(...)           → robot_id
  ├─ build joint_id + link_id  → map tên → index
  └─ main loop (240 Hz)
        │
        ├─ đọc slider          → new_score (0–10)
        ├─ nếu score đổi       → HOMING
        ├─ state machine       → HOMING → MOVING → CONTACT_CHECK
        └─ refresh_ui()
```

---

## 2. State machine

```
          score đổi
              │
              ▼
           HOMING  ──(480 steps)──► MOVING  ──(480 steps)──► CONTACT_CHECK
          drive(neutral)           drive(pose[score])         check mỗi frame
          force=10                 force=5
              │
          score=0?
              │
              ▼
            IDLE
```

- **HOMING**: gọi `drive(neutral_pose, force=10)` một lần → PyBullet motor tự giữ target, không cần gọi lại.
- **MOVING**: gọi `drive(KAPANDJI_POSES[score], force=5)` một lần → motor chạy đến pose đó.
- **CONTACT_CHECK**: gọi `has_contact()` mỗi frame. Khi lần đầu pass → thêm vào `passed`.

Transition dùng biến `steps` tăng mỗi loop. Reset về 0 khi chuyển state.

---

## 3. `drive()` — điều khiển joint

```python
def drive(pose, force=5.0):
    for name, target in pose.items():
        jid = joint_id.get(name)
        if jid is not None:
            p.setJointMotorControl2(
                bodyUniqueId=robot_id, jointIndex=jid,
                controlMode=p.POSITION_CONTROL,
                targetPosition=target, force=force,
            )
```

- `POSITION_CONTROL`: PyBullet thêm một PD controller vào joint, tự drive về `targetPosition`.
- Gọi **một lần** là đủ — motor giữ target cho đến khi bị gọi lại với target khác.
- `force` = lực tối đa motor được dùng (Newton·meter). Nhỏ quá → joint chưa đến target.

---

## 4. Quy ước dấu trong `KAPANDJI_POSES`

Giá trị trong pose dict là **target thực gửi xuống PyBullet** (không nhân sign như bên load_test.py).

| Joint | axis trong URDF | Gập ngón = |
|-------|----------------|------------|
| index_pip_0 | `-1 0 0` | **âm** (ví dụ `-0.5`) |
| middle_pip_0 | `-1 0 0` | **âm** |
| ring_dip_0 | `-1 0 0` | **âm** |
| thumb_ip_0 | `-1 0 0` | **âm** |
| thumb_mcp_0 | `-1 0 0` | **dương** (vì upper=2.0944 là hướng vào lòng bàn tay) |
| tất cả còn lại | `+1 0 0` | **dương** |

---

## 5. `KAPANDJI_TARGET_LINKS` — link nào được dùng để detect contact

Lấy từ URDF parent/child:

```
Score 1  → mcp_10_1_2   child of index_mcp_0  (đốt gần ngón trỏ)
Score 2  → pip_10_2     child of index_pip_0  (đốt giữa ngón trỏ)
Score 3  → dp_10_3_1    child of index_dip_0  (đầu ngón trỏ)
Score 4  → dp_10_3_3    child of middle_dip_0 (đầu ngón giữa)
Score 5  → dp_10_3_2    child of ring_dip_0   (đầu ngón áp út)
Score 6  → dp_10_3      child of little_dip_0 (đầu ngón út)
Score 7  → pip_10_2_1   child of little_pip_0 (đốt giữa ngón út — nếp DIP)
Score 8  → mcp_10_1_1   child of little_mcp_0 (đốt gần ngón út — nếp PIP)
Score 9  → dmc_2        fixed child of palm_3  (xương bàn ngón út — nếp MCP)
Score 10 → palm_3       child of root joint    (lòng bàn tay)
```

---

## 6. `has_contact()` — detect contact giữa thumb tip và target

```python
def has_contact(target_lid):
    if THUMB_TIP is None or target_lid is None:
        return False
    return len(p.getContactPoints(
        bodyA=robot_id, bodyB=robot_id,
        linkIndexA=THUMB_TIP, linkIndexB=target_lid,
    )) > 0
```

- `THUMB_TIP` = link index của `thumb_dp_2_1` (đầu ngón cái).
- `getContactPoints(bodyA=X, bodyB=X, linkIndexA=a, linkIndexB=b)` → trả về list các điểm contact giữa link `a` và link `b` trên cùng body.
- Cần `URDF_USE_SELF_COLLISION` khi load để PyBullet check self-collision.

---

## 7. `tint()` — đổi màu link để hiển thị trạng thái

```python
def tint(lid, rgba):
    if lid is not None:
        p.changeVisualShape(robot_id, lid, rgbaColor=rgba)
```

| Màu | Ý nghĩa |
|-----|---------|
| Vàng `[1,1,0,1]` | Đang approach (state MOVING) |
| Xanh lá `[0,1,0,1]` | Contact PASS |
| Cam `[1,0.3,0,1]` | Contact FAIL |
| Xám `[0.8,0.8,0.8,1]` | Restore khi đổi score |

---

## 8. `refresh_ui()` — cập nhật 4 dòng text trên GUI

```python
txt_score   # dòng 1: tên score đang test
txt_contact # dòng 2: PASS / FAIL / (homing...) / (moving...)
txt_state   # dòng 3: tên state hiện tại
txt_passed  # dòng 4: danh sách score đã pass
```

Dùng `replaceItemUniqueId=<id>` để update text tại chỗ thay vì tạo text mới mỗi frame.

---

## 9. Kết quả cuối

```python
# Cuối loop (Ctrl+C):
Kapandji score: max(passed)   # score cao nhất đạt được
```

---

---

# BUGS

---

## 🔴 BUG 1 (Blocking) — Không có collision geometry

**Vấn đề:**
URDF không có `<collision>` block nào (chỉ có `<visual>`).
PyBullet chỉ detect contact giữa link có collision mesh.
→ `getContactPoints` luôn trả `[]` → `has_contact` mãi `False` → không score nào pass.

```bash
grep -c '<collision>' assembly_1/urdf/assembly_1.urdf   # → 0
```

**Fix — thay bằng proximity check:**

```python
CONTACT_THRESHOLD = 0.018   # mét — chỉnh sau khi chạy thử

def has_contact(target_lid):
    if THUMB_TIP is None or target_lid is None:
        return False
    tip_pos    = p.getLinkState(robot_id, THUMB_TIP)[0]
    target_pos = p.getLinkState(robot_id, target_lid)[0]
    dist = sum((a - b) ** 2 for a, b in zip(tip_pos, target_pos)) ** 0.5
    return dist < CONTACT_THRESHOLD
```

`getLinkState(...)[0]` = vị trí world-frame của link COM.

---

## 🟡 BUG 2 — Restore màu sai

**Vấn đề:** Khi đổi score, link cũ bị reset về xám cứng `[0.8,0.8,0.8,1]`.
Màu gốc của link bị mất vĩnh viễn.

**Fix — lưu màu gốc trước khi tint:**

```python
# Sau khi build link_id:
original_rgba = {}
for lid in link_id.values():
    vdata = p.getVisualShapeData(robot_id, lid)
    if vdata:
        original_rgba[lid] = list(vdata[0][7])

def restore_tint(lid):
    if lid is not None:
        rgba = original_rgba.get(lid, [0.8, 0.8, 0.8, 1.0])
        p.changeVisualShape(robot_id, lid, rgbaColor=rgba)
```

Thay dòng `tint(cur_target_lid, [0.8, 0.8, 0.8, 1.0])` → `restore_tint(cur_target_lid)`.

---

## 🟡 BUG 3 — Joint có thể chưa đến target sau SETTLE steps

**Vấn đề:** Sau 480 steps (~2s), một số joint với `force=5.0` có thể chưa đến đúng vị trí,
khiến CONTACT_CHECK bắt đầu sớm → không detect được contact dù pose gần đúng.

**Fix — thêm check vị trí thực tế:**

```python
def joints_settled(pose, tol=0.05):
    for name, target in pose.items():
        jid = joint_id.get(name)
        if jid is None:
            continue
        current = p.getJointState(robot_id, jid)[0]
        if abs(current - target) > tol:
            return False
    return True
```

Đổi điều kiện transition MOVING → CONTACT_CHECK:

```python
# Cũ:
elif state == "MOVING" and steps >= SETTLE:

# Mới:
elif state == "MOVING" and steps >= SETTLE and joints_settled(KAPANDJI_POSES[score]):
```

---

## 🔵 LOGIC — Kapandji score chuẩn lâm sàng

**Vấn đề:** `max(passed)` không đúng chuẩn. Kapandji score = score liên tục cao nhất từ 1.
Ví dụ pass {1, 2, 4, 5} → lâm sàng là score 2 (chuỗi 1→2 bị đứt ở 3), không phải 5.

**Fix:**

```python
def kapandji_score(passed):
    for i in range(10, 0, -1):
        if all(s in passed for s in range(1, i + 1)):
            return i
    return 0

# Thay max(passed) → kapandji_score(passed)
```
