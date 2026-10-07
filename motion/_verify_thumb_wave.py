import json
import re
from pathlib import Path

d = json.loads(Path(__file__).with_name("motion_show.txt").read_text(encoding="utf-8"))
keys = [k for k in d if re.match(r"\d+_W", k)]
print("sample cascade (MCP):")
for k in keys[0:40:5]:
    fr = d[k]
    print(
        f"{k}: th={fr['left_hand_thumb_MCP_joint']:.3f} "
        f"i={fr['left_hand_index_MCP_joint']:.3f} "
        f"m={fr['left_hand_mid_MCP_joint']:.3f} "
        f"r={fr['left_hand_ring_MCP_joint']:.3f} "
        f"p={fr['left_hand_pinky_MCP_joint']:.3f}"
    )
for k in ["origin", "m1", "00_origin"]:
    print(k, "thumb MCP", d[k]["left_hand_thumb_MCP_joint"])
