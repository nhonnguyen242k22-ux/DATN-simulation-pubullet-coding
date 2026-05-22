"""
Workspace Analysis + Singularity — Thesis Sec.3.1.1 & Sec.5.3.8
Style: fk_test.py (sliders + panel bên phải)

Scans toàn bộ (q1, q2) grid → FK → vẽ reachable workspace boundary.
Live: det(J) + singularity warning khi q1=q2≈0.
"""
import pybullet as p
import pybullet_data
import time, os, math

exec(open(os.path.expanduser("~/bionic_hand_sim/fix_urdf.py")).read())
URDF = os.path.expanduser("~/bionic_hand_sim/assembly_1/urdf/assembly_1_fixed.urdf")

# ── Thesis DH parameters (Table 3.2 / 5.2) ───────────────────────────────
L1, L2, L3 = 39.2, 30.0, 31.0   # mm: proximal / middle / distal phalanx
C           = 0.67               # DIP/PIP coupling ratio (Eq.5.7)
MAX_R       = L1 + L2 + L3      # 100.2 mm — max reach (full extension)

PANEL_X     = 0.22
AXIS_LEN    = 0.010
SIM_HZ      = 240
DT          = 1.0 / SIM_HZ

# ── Connect ────────────────────────────────────────────────────────────────
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.resetDebugVisualizerCamera(
    cameraDistance=0.5, cameraYaw=45,
    cameraPitch=-45, cameraTargetPosition=[0, 0, 0.1]
)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))
robot_id = p.loadURDF(
    URDF, basePosition=[0, 0, 0.15],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    useFixedBase=True
)

# ── Build maps ─────────────────────────────────────────────────────────────
name_to_id, link_id = {}, {}
for i in range(p.getNumJoints(robot_id)):
    info = p.getJointInfo(robot_id, i)
    name_to_id[info[1].decode()] = i
    link_id[info[12].decode()]   = i

negative_axis_joints = [
    "index_mcp_0", "little_mcp_0", "ring_mcp_0",
    "index_pip_0", "middle_pip_0", "ring_dip_0", "thumb_ip_0",
]
custom_ranges = {
    "thumb_dmc_0": (-0.5236, 2.0944, 0.0),
    "thumb_mcp_0": (-0.5236, 2.0944, 0.0),
}
finger_groups = {
    "Thumb":  ["thumb_dmc_0", "thumb_mcp_0", "thumb_ip_0"],
    "Index":  ["index_mcp_0", "index_pip_0",  "index_dip_0"],
    "Middle": ["middle_mcp_0","middle_pip_0", "middle_dip_0"],
    "Ring":   ["ring_mcp_0",  "ring_pip_0",  "ring_dip_0"],
    "Little": ["little_mcp_0","little_pip_0","little_dip_0"],
}

sliders, signs = {}, {}
for finger, joints in finger_groups.items():
    for jname in joints:
        if jname not in name_to_id:
            continue
        jid = name_to_id[jname]
        lo, hi, default = custom_ranges.get(jname, (0.0, 1.5708, 0.0))
        sliders[jid] = p.addUserDebugParameter(
            f"{finger} {jname.split('_')[1].upper()}", lo, hi, default)
        signs[jid] = -1 if jname in negative_axis_joints else 1

# ── Fingertip panel (same 5 tips as fk_test.py) ───────────────────────────
TIPS = [
    ("thumb_dp_2_1", "THUMB",  [1.0, 0.55, 0.0]),
    ("dp_10_3_1",    "INDEX",  [0.2, 1.0,  0.2]),
    ("dp_10_3_3",    "MIDDLE", [0.2, 0.6,  1.0]),
    ("dp_10_3_2",    "RING",   [1.0, 0.2,  1.0]),
    ("dp_10_3",      "LITTLE", [1.0, 1.0,  0.2]),
]
tip_data = []
for idx, (lname, label, color) in enumerate(TIPS):
    lid = link_id.get(lname)
    if lid is None:
        continue
    pos = p.getLinkState(robot_id, lid)[4]
    pp  = [PANEL_X, 0.0, 0.46 - idx * 0.055]
    pid = p.addUserDebugText(
        f"{label}: ({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})",
        pp, textColorRGB=color, textSize=0.85)
    lx = p.addUserDebugLine(pos, [pos[0]+AXIS_LEN,pos[1],pos[2]], [1,0,0], 3)
    ly = p.addUserDebugLine(pos, [pos[0],pos[1]+AXIS_LEN,pos[2]], [0,1,0], 3)
    lz = p.addUserDebugLine(pos, [pos[0],pos[1],pos[2]+AXIS_LEN], [0,0,1], 3)
    tip_data.append({"lid":lid,"label":label,"color":color,
                     "panel_id":pid,"panel_pos":pp,"lx":lx,"ly":ly,"lz":lz})

# ── Workspace Scan controls ───────────────────────────────────────────────
p.addUserDebugText("=== WORKSPACE + SINGULARITY ===",
                   [PANEL_X, 0.0, 0.55], textColorRGB=[1,1,0], textSize=0.90)
scan_btn    = p.addUserDebugParameter("Scan Workspace (press)", 1, 0, 0)
clear_btn   = p.addUserDebugParameter("Clear Lines   (press)", 1, 0, 0)
res_slider  = p.addUserDebugParameter("Scan Resolution (deg step)", 2, 15, 6)

# ── Algorithm output panel ─────────────────────────────────────────────────
ALGO_Y0, ALGO_DY = 0.20, 0.045
algo_ids = []
for i in range(9):
    tid = p.addUserDebugText("", [PANEL_X, 0.0, ALGO_Y0 - i*ALGO_DY],
                             textColorRGB=[1,1,1], textSize=0.78)
    algo_ids.append(tid)

def set_panel(lines):
    for i, tid in enumerate(algo_ids):
        txt = lines[i] if i < len(lines) else ""
        p.addUserDebugText(txt, [PANEL_X, 0.0, ALGO_Y0 - i*ALGO_DY],
                           textColorRGB=[1,1,1], textSize=0.78,
                           replaceItemUniqueId=tid)

# ── Analytical FK + Jacobian ──────────────────────────────────────────────
def fk_local(q1, q2):
    """Planar FK in MCP-local frame (Eq.3.4-3.5, 5.21). Returns (X,Z) in mm."""
    q3 = C * q2
    x  = L1*math.sin(q1) + L2*math.sin(q1+q2) + L3*math.sin(q1+q2+q3)
    z  = L1*math.cos(q1) + L2*math.cos(q1+q2) + L3*math.cos(q1+q2+q3)
    return x, z

def jacobian_det(q1, q2):
    """det(J) of 2×2 analytical Jacobian (Eq.5.27)."""
    q12  = q1 + q2
    q123 = q1 + (1 + C)*q2
    df1q1 =  L1*math.cos(q1) + L2*math.cos(q12) + L3*math.cos(q123)
    df1q2 =  L2*math.cos(q12) + L3*(1+C)*math.cos(q123)
    df2q1 = -L1*math.sin(q1) - L2*math.sin(q12) - L3*math.sin(q123)
    df2q2 = -L2*math.sin(q12) - L3*(1+C)*math.sin(q123)
    return df1q1*df2q2 - df1q2*df2q1

# ── Workspace Scan ────────────────────────────────────────────────────────
ws_lines = []   # debug line IDs
idx_mcp_jid = name_to_id.get("index_mcp_0")
idx_tip_lid = link_id.get("dp_10_3_1")

def do_workspace_scan(step_deg=6):
    """
    Scan (q1, q2) grid → FK → collect XZ points → draw boundary.
    Also marks singular configurations in red.
    Thesis Fig.3.2: blue=reachable area, red=singular.
    """
    global ws_lines
    # Clear old lines
    for lid in ws_lines:
        p.removeUserDebugItem(lid)
    ws_lines.clear()

    step   = math.radians(step_deg)
    q_vals = [i*step for i in range(int(math.pi/2/step)+1)]

    # MCP base position in world (get from PyBullet)
    mcp_world = list(p.getLinkState(robot_id, idx_mcp_jid)[4])
    base_z    = 0.15   # robot base height

    # Collect all reachable points (in WORLD XZ plane, Y ~ mcp_y)
    pts = []
    for q1 in q_vals:
        for q2 in q_vals:
            x, z = fk_local(q1, q2)
            pts.append((x, z, q1, q2))

    # Find boundary: for each q1 slice, find max reach point
    # Draw dots as short line segments in 3D
    SCALE = 0.001   # mm → metres
    mcp_x = mcp_world[0]
    mcp_y = mcp_world[1]
    mcp_z = mcp_world[2]

    n_pts = 0
    for (x, z, q1, q2) in pts:
        # World position: hand faces +Z, finger extends upward
        wx = mcp_x + x * SCALE
        wy = mcp_y
        wz = mcp_z + z * SCALE

        det = jacobian_det(q1, q2)
        if abs(det) < 200:
            color = [1, 0, 0]   # red = singular/near-singular
        elif abs(det) < 800:
            color = [1, 0.5, 0] # orange = near singular
        else:
            color = [0, 0.4, 1] # blue = safe

        lid = p.addUserDebugLine(
            [wx, wy-0.002, wz], [wx, wy+0.002, wz],
            color, lineWidth=2, lifeTime=0
        )
        ws_lines.append(lid)
        n_pts += 1

    print(f"  [WORKSPACE] Scanned {n_pts} points  (step={step_deg}°)")
    print(f"  Blue=safe | Orange=near-singular | Red=singular")
    print(f"  Singular config: q1=q2=0 (full extension, det→0)")

def do_clear():
    global ws_lines
    for lid in ws_lines:
        p.removeUserDebugItem(lid)
    ws_lines.clear()
    print("  [WORKSPACE] Cleared.")

# ── Main loop ──────────────────────────────────────────────────────────────
prev_scan = 0
prev_clear = 0
loop_count = 0
print("\n" + "="*55)
print("  WORKSPACE + SINGULARITY — Thesis Sec.3.1.1 & 5.3.8")
print("="*55)
print("  - Nhấn 'Scan Workspace' để vẽ reachable workspace")
print("  - Điều chỉnh 'Scan Resolution' rồi scan lại")
print("  - Kéo slider → xem det(J) real-time")
print("  - Blue=safe | Orange=near-sing | Red=SINGULAR")
print("  - Singular khi q1=q2=0 (finger fully extended)")
print("="*55 + "\n")

try:
    while True:
        loop_count += 1

        # Check buttons
        scan_val  = p.readUserDebugParameter(scan_btn)
        clear_val = p.readUserDebugParameter(clear_btn)
        if scan_val > prev_scan:
            prev_scan = scan_val
            step_deg  = p.readUserDebugParameter(res_slider)
            do_workspace_scan(step_deg)
        if clear_val > prev_clear:
            prev_clear = clear_val
            do_clear()

        # Drive joints from sliders
        for jid, sid in sliders.items():
            val = p.readUserDebugParameter(sid)
            p.setJointMotorControl2(robot_id, jid, p.POSITION_CONTROL,
                                    targetPosition=val*signs[jid], force=5.0)

        # Update fingertip panel
        for td in tip_data:
            pos = p.getLinkState(robot_id, td["lid"])[4]
            p.addUserDebugText(
                f"{td['label']}: ({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f})",
                td["panel_pos"], textColorRGB=td["color"], textSize=0.85,
                replaceItemUniqueId=td["panel_id"])
            p.addUserDebugLine(pos,[pos[0]+AXIS_LEN,pos[1],pos[2]],
                               [1,0,0],3,replaceItemUniqueId=td["lx"])
            p.addUserDebugLine(pos,[pos[0],pos[1]+AXIS_LEN,pos[2]],
                               [0,1,0],3,replaceItemUniqueId=td["ly"])
            p.addUserDebugLine(pos,[pos[0],pos[1],pos[2]+AXIS_LEN],
                               [0,0,1],3,replaceItemUniqueId=td["lz"])

        # Live workspace + singularity panel
        q1 = p.readUserDebugParameter(sliders[idx_mcp_jid])
        # index_mcp is negative_axis — slider value = |q1|
        q2 = p.readUserDebugParameter(sliders[name_to_id["index_pip_0"]])

        x_fk, z_fk = fk_local(q1, q2)
        reach       = math.sqrt(x_fk**2 + z_fk**2)
        det         = jacobian_det(q1, q2)
        det_ext     = jacobian_det(0, 0)   # should = 0

        if abs(det) < 200:
            sing_str = "!!! SINGULAR ZONE !!!"
        elif abs(det) < 800:
            sing_str = "(near-singular)"
        else:
            sing_str = "OK — nonsingular"

        # Workspace reachability from thesis (Fig.3.2)
        q1_lim = math.radians(90)
        q2_lim = math.radians(90)
        pct_q1 = q1 / q1_lim * 100
        pct_q2 = q2 / q2_lim * 100

        lines = [
            "=WORKSPACE / SINGULARITY=",
            f"q1(MCP)={math.degrees(q1):.1f}d  q2(PIP)={math.degrees(q2):.1f}d",
            f"[FK (Eq.3.4-3.5, 5.21)]",
            f"  X={x_fk:+.2f}mm  Z={z_fk:+.2f}mm",
            f"  Reach={reach:.2f}/{MAX_R:.1f}mm ({reach/MAX_R*100:.0f}%)",
            f"[det(J) Singularity (Eq.5.23)]",
            f"  det(J)={det:+.1f}  {sing_str}",
            f"  @q1=q2=0: det={det_ext:.4f} (thesis→0)",
            f"  [Press 'Scan Workspace' to plot WS]",
        ]
        set_panel(lines)

        # Console print every 5s
        if loop_count % (SIM_HZ*5) == 0:
            print(f"  MCP={math.degrees(q1):.1f}d PIP={math.degrees(q2):.1f}d"
                  f"  Reach={reach:.1f}mm  det(J)={det:+.0f}  {sing_str}")

        p.stepSimulation()
        time.sleep(DT)

except KeyboardInterrupt:
    p.disconnect()
    print("\n🔴 Dừng.")
