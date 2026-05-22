# Bionic Hand Simulation — CLAUDE.md

## Tổng quan

Mô phỏng bàn tay robot 5 ngón bằng **PyBullet**, model được xuất từ **OnShape** dưới dạng URDF.  
Robot có 15 bậc tự do (3 khớp × 5 ngón), cộng thêm 1 khớp phụ cho ngón cái (thumb_dmc).

---

## Cấu trúc thư mục

```
bionic_hand_sim/
├── CLAUDE.md                          # file này
├── fix_urdf.py                        # bước tiền xử lý URDF
├── load_test.py                       # script chạy simulation chính
├── venv/                              # Python virtual environment
└── assembly_1/
    ├── meshes/                        # file STL cho từng link
    │   ├── PALM_3.stl
    │   ├── THUMB_MCP_2_1.stl
    │   ├── THUMB_DP_2_1.stl
    │   ├── thumb__test__1_1.stl
    │   ├── MCP_10_1.stl
    │   ├── PIP_10_2.stl
    │   ├── DP_10_3.stl
    │   ├── DMC_2.stl
    │   ├── PALM_FRONT_CASE.stl
    │   ├── Copy_of_RBT_THM_ConnectorHousing_V1_0_FOR_URDF.stl
    │   └── Copy_of_RBT_THM_ProximalJoint_ASM_V1_0_FOR_URDF.stl
    └── urdf/
        ├── assembly_1.urdf            # URDF gốc từ OnShape (KHÔNG sửa tay)
        └── assembly_1_fixed.urdf      # URDF đã xử lý (tự động tạo, KHÔNG commit)
```

---

## Quy trình làm việc

### 1. Sửa joint limits / axis → `fix_urdf.py`

File `fix_urdf.py` đọc `assembly_1.urdf` và tạo ra `assembly_1_fixed.urdf` với:
- Đổi tất cả joint `continuous` → `revolute`
- Gán `axis`, `lower`, `upper` đúng cho từng khớp
- Sửa đường dẫn mesh: `package://assembly_1/meshes/` → `../meshes/`

**Mọi thay đổi về joint config phải sửa trong `fix_urdf.py`, không phải trong `assembly_1.urdf`.**

### 2. Chạy simulation → `load_test.py`

```bash
source venv/bin/activate
python load_test.py
```

Script tự động gọi `fix_urdf.py` rồi load `assembly_1_fixed.urdf` vào PyBullet GUI.  
Giao diện có slider cho từng khớp, kéo slider để điều khiển ngón tay.

---

## Sơ đồ khớp (joint map)

### Ngón tay thường (Index / Middle / Ring / Little)

```
palm_3
  └─[dmc_2_x]──{finger_mcp_0}──[mcp_10_1_x]──{finger_pip_0}──[pip_10_2_x]──{finger_dip_0}──[dp_10_3_x]
```

| Joint | Axis | Lower (rad) | Upper (rad) | Ghi chú |
|-------|------|-------------|-------------|---------|
| index_mcp_0 | +X | 0 | 1.5708 (90°) | |
| index_pip_0 | −X | −1.5708 | 0 | sign=−1 trong load_test |
| index_dip_0 | +X | 0 | 1.5708 | |
| middle_mcp_0 | +X | 0 | 1.5708 | |
| middle_pip_0 | −X | −1.5708 | 0 | sign=−1 |
| middle_dip_0 | +X | 0 | 1.5708 | |
| ring_mcp_0 | +X | 0 | 1.5708 | |
| ring_pip_0 | +X | 0 | 1.5708 | |
| ring_dip_0 | −X | −1.5708 | 0 | sign=−1 |
| little_mcp_0 | +X | 0 | 1.5708 | |
| little_pip_0 | +X | 0 | 1.5708 | |
| little_dip_0 | +X | 0 | 1.5708 | |

### Ngón cái (Thumb)

```
palm_3
  └─[connector]──[proximal_joint]──{thumb_dmc_0}──[thumb_test]
                                                      └─{thumb_mcp_0}──[thumb_mcp_2_1]
                                                                           └─{thumb_ip_0}──[thumb_dp_2_1]
```

| Joint | Axis | Lower (rad) | Upper (rad) | Ghi chú |
|-------|------|-------------|-------------|---------|
| thumb_dmc_0 | +X | 0 | 1.2217 (70°) | khớp gốc ngón cái |
| thumb_mcp_0 | −X | −0.5236 (−30°) | 2.0944 (120°) | chiều dương = gập vào lòng bàn tay |
| thumb_ip_0 | −X | −1.2217 | 0 | sign=−1 |

---

## Convention điều khiển trong `load_test.py`

- **Tất cả slider** đi từ 0 → dương = gập ngón (flexion).
- Các joint có `axis = "-1 0 0"` và giới hạn âm được liệt kê trong `negative_axis_joints` → slider nhân với `sign = -1` trước khi gửi xuống PyBullet.
- `thumb_mcp_0` **không** nằm trong `negative_axis_joints` vì chiều dương đã là hướng vào lòng bàn tay (upper = 2.0944).
- `thumb_mcp_0` có slider riêng trong `custom_ranges`: range `[-0.5236, 2.0944]`, default = 0.

```python
# negative_axis_joints: slider × (−1) → target
negative_axis_joints = ["index_pip_0", "middle_pip_0", "ring_dip_0", "thumb_ip_0"]

# custom_ranges: {joint_name: (min, max, default)}
custom_ranges = {
    "thumb_mcp_0": (-0.5236, 2.0944, 0.0),
}
```

---

## Lưu ý quan trọng

- `assembly_1_fixed.urdf` được **tự động tạo** mỗi lần chạy `load_test.py`. Không cần (và không nên) sửa tay file này.
- Nguồn sự thật duy nhất về joint config là `fix_urdf.py` → `joint_config` dict.
- `assembly_1.urdf` là file gốc từ OnShape. Chỉ sửa khi cần thay đổi cấu trúc link (thêm/bớt link, sửa mesh origin, sửa giới hạn thumb_mcp vì nó được đọc trực tiếp từ file gốc cho joint type=revolute).
- Môi trường Python: luôn `source venv/bin/activate` trước khi chạy.
