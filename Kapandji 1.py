# ─── KAPANDJI POINT 1 ─────────────────────────────────────────────────────────
# Target: lateral side of index PROXIMAL phalanx
# Target link: mcp_10_1_2  (child of index_mcp_0)
# Thumb tip link: thumb_dp_2_1

CONTACT_THRESHOLD_1 = 0.018   # metres — tune after seeing simulation

POSE_1 = {
    # ── Thumb: small abduction + light flexion, IP straight ──
    "thumb_dmc_0":  0.2,   # slight abduction (CMC)
    "thumb_mcp_0":  0.3,   # light flexion toward index
    "thumb_ip_0":   0.0,   # IP stays straight for lateral touch

    # ── Index: slightly raised (MCP flexed), pip/dip relaxed ──
    "index_mcp_0":  0.2,   # bring proximal phalanx up slightly
    "index_pip_0":  0.0,   # straight — we want lateral not tip
    "index_dip_0":  0.0,

    # ── Other fingers: loosely curled out of the way ──
    "middle_mcp_0": 0.3,  "middle_pip_0": -0.3, "middle_dip_0": 0.2,
    "ring_mcp_0":   0.3,  "ring_pip_0":    0.3,  "ring_dip_0":  -0.2,
    "little_mcp_0": 0.3,  "little_pip_0":  0.3,  "little_dip_0": 0.2,
}

TARGET_LINK_1 = "mcp_10_1_2"   # index proximal phalanx body

def has_contact_proximity(thumb_tip_lid, target_lid, threshold):
    """
    Proximity-based contact check — works even without collision geometry.
    Uses getLinkState()[4] = world-frame position of link frame origin.
    """
    if thumb_tip_lid is None or target_lid is None:
        return False, float("inf")
    tip_pos    = p.getLinkState(robot_id, thumb_tip_lid)[4]   # [4] = worldLinkFramePosition
    target_pos = p.getLinkState(robot_id, target_lid)[4]
    dist = sum((a - b) ** 2 for a, b in zip(tip_pos, target_pos)) ** 0.5
    return dist < threshold, dist

# ── How to call it in your main loop for Point 1 ──────────────────────────────
# Replace your existing has_contact() call with:
#
#   thumb_tip_lid  = link_id.get("thumb_dp_2_1")
#   target_lid_1   = link_id.get(TARGET_LINK_1)
#   contact, dist  = has_contact_proximity(thumb_tip_lid, target_lid_1, CONTACT_THRESHOLD_1)
#
# Then use `contact` as True/False, and print `dist` to tune the threshold.