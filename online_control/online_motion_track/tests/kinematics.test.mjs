import { test } from "node:test";
import assert from "node:assert/strict";
import { execFileSync } from "node:child_process";
import { existsSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { buildRobot } from "../static/kinematics.js";

const root = fileURLToPath(new URL("../../../", import.meta.url));
const python =
  process.env.HAND_PYTHON ||
  (existsSync("E:/conda/envs/hand_dex/python.exe")
    ? "E:/conda/envs/hand_dex/python.exe"
    : "python");
const source = `
import json, sys
sys.path[:0]=['.','src']
from online_control.online_motion_track.main import model_description
from hand_profiles import load_hand_profile
from TsenseI_retargeting.fk.urdf import UrdfModel
out=[]
for model_id in ('TsenseI_v3_2','TsenseI_v3_2_withid'):
 profile=load_hand_profile(model_id)
 for side in ('left','right'):
  model=UrdfModel(profile.path('urdf',side))
  poses=[]
  for ratio in (0,.3,.8,1):
   q={n:lo+ratio*(hi-lo) for n,(lo,hi) in model.joint_limits().items()}
   transforms={n:m.T.reshape(-1).tolist() for n,m in model.forward_kinematics(q).items()}
   poses.append({'q':q,'transforms':transforms})
  out.append({'description':model_description(side,model_id),'poses':poses})
print(json.dumps(out))
`;
const fixtures = JSON.parse(
  execFileSync(python, ["-c", source], { cwd: root, encoding: "utf8" }),
);
for (const { description, poses } of fixtures) {
  test(`${description.model_id} ${description.side}: Three.js joint transforms equal calibrated Python URDF FK`, () => {
    const robot = buildRobot(description);
    for (const { q, transforms } of poses) {
      robot.setPose(q);
      for (const [name, expected] of Object.entries(transforms)) {
        const actual = robot.links.get(name).matrixWorld.elements;
        for (let i = 0; i < 16; i++)
          assert(Math.abs(actual[i] - expected[i]) < 1e-10, `${name}[${i}]`);
      }
    }
  });
}
