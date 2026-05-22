import os
import re

input_file  = os.path.expanduser("~/bionic_hand_sim/assembly_1/urdf/assembly_1.urdf")
output_file = os.path.expanduser("~/bionic_hand_sim/assembly_1/urdf/assembly_1_fixed.urdf")

with open(input_file, "r") as f:
    lines = f.readlines()

# Dùng axis gốc của URDF mới (không override).
# Chỉ thêm limit + đổi continuous → revolute.
joint_config = {
    # Finger MCP — axis gốc mới là -1 0 0 → limit âm
    "index_mcp_0":  {"axis": "-1 0 0", "lower": -1.5708, "upper": 0},
    "little_mcp_0": {"axis": "-1 0 0", "lower": -1.5708, "upper": 0},
    "middle_mcp_0": {"axis": "1 0 0",  "lower": 0,       "upper": 1.5708},
    "ring_mcp_0":   {"axis": "-1 0 0", "lower": -1.5708, "upper": 0},

    # Other finger joints — giống cũ
    "index_dip_0":  {"axis": "1 0 0",  "lower": 0,       "upper": 1.5708},
    "index_pip_0":  {"axis": "-1 0 0", "lower": -1.5708, "upper": 0},
    "little_dip_0": {"axis": "1 0 0",  "lower": 0,       "upper": 1.5708},
    "little_pip_0": {"axis": "1 0 0",  "lower": 0,       "upper": 1.5708},
    "middle_dip_0": {"axis": "1 0 0",  "lower": 0,       "upper": 1.5708},
    "middle_pip_0": {"axis": "-1 0 0", "lower": -1.5708, "upper": 0},
    "ring_dip_0":   {"axis": "-1 0 0", "lower": -1.5708, "upper": 0},
    "ring_pip_0":   {"axis": "1 0 0",  "lower": 0,       "upper": 1.5708},

    # Thumb — axis giữ nguyên theo URDF mới
    "thumb_dmc_0":  {"axis": "1 0 0",  "lower": -0.5236, "upper": 2.6179},  # -30° to 150°
    "thumb_ip_0":   {"axis": "-1 0 0", "lower": -1.2217, "upper": 0},
    "thumb_mcp_0":  {"axis": "-1 0 0", "lower": -0.5236, "upper": 2.0944},
}

def fix_line(line):
    line = line.replace('type="continuous"', 'type="revolute"')
    line = line.replace('filename="package://assembly_1/', 'filename="../')
    return line

output_lines = []
i = 0
while i < len(lines):
    line = lines[i]

    # Visual block → clone thành collision block
    if '<visual>' in line:
        visual_block = [line]
        i += 1
        while i < len(lines) and '</visual>' not in lines[i]:
            visual_block.append(lines[i])
            i += 1
        visual_block.append(lines[i])

        for vl in visual_block:
            output_lines.append(fix_line(vl))

        origin_line = None
        geom_lines  = []
        in_geom     = False
        for vl in visual_block:
            vl = fix_line(vl)
            if '<origin' in vl and origin_line is None:
                origin_line = vl
            if '<geometry>' in vl:
                in_geom = True
            if in_geom:
                geom_lines.append(vl)
            if '</geometry>' in vl:
                in_geom = False

        pad = ' ' * (len(line) - len(line.lstrip()))
        output_lines.append(f'{pad}<collision>\n')
        if origin_line:
            output_lines.append(origin_line)
        output_lines.extend(geom_lines)
        output_lines.append(f'{pad}</collision>\n')

    # Joint block
    elif re.search(r'<joint name="([^"]+)"', line):
        joint_name = re.search(r'<joint name="([^"]+)"', line).group(1)

        if joint_name in joint_config:
            config = joint_config[joint_name]

            joint_lines = [line]
            i += 1
            while i < len(lines) and '</joint>' not in lines[i]:
                joint_lines.append(lines[i])
                i += 1
            joint_lines.append(lines[i])

            output_lines.append(f'    <joint name="{joint_name}" type="revolute">\n')
            output_lines.append(f'        <limit lower="{config["lower"]}" upper="{config["upper"]}" effort="10" velocity="1.0" />\n')

            for jline in joint_lines[1:-1]:
                if '<origin' in jline:
                    output_lines.append(jline)
                    break

            output_lines.append(f'        <axis xyz="{config["axis"]}" />\n')

            for jline in joint_lines[1:-1]:
                if '<parent' in jline or '<child' in jline:
                    output_lines.append(jline)

            output_lines.append('    </joint>\n')
        else:
            output_lines.append(fix_line(line))

    else:
        output_lines.append(fix_line(line))

    i += 1

with open(output_file, "w") as f:
    f.writelines(output_lines)

print("✅ URDF mới fixed!")
