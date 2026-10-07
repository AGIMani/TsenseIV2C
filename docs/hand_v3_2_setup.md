# V3.2 模型、标定和碰撞

本轮资源位于 `assets/hands/TsenseI_hand_v3_2`，源文件为
`TsenseI_v3_2/urdf/A3.1V6 26.8.4-2.urdf`（用户于 2026-09-17 确认）。
已将原 `TsenseI_v3_2/A3.2 URDF/` 内的完整 CAD 包替换到 `TsenseI_v3_2/`；
原文件名保留，项目手型标识仍为 `TsenseI_v3_2`。V2 模型保持不变。
旧来源包已备份到 `assets/hands/_backups/TsenseI_v3_2_source_20260917_103852_119/`，
旧左右模型及手动标定备份到 `assets/hands/_backups/TsenseI_hand_v3_2_20260917_103900_649430/`。
迁移的 30 个文件逐一通过 SHA-256 校验。递归删除被自动审批拦截后，原嵌套目录改为
归档到 `assets/hands/_backups/TsenseI_v3_2_import_20260917_104458_784/`，
`TsenseI_v3_2/A3.2 URDF/` 已不再存在；归档可用于恢复原始导入包。
当前关键点是根据本次模型生成的新草稿，需要重新人工确认，不沿用上一份错误模型的标定。
`source_mapping.yaml` 记录源 URDF 和网格包的 SHA-256，用于检测同名模型内容变更。

## 小指名称

2026-09-20 按物理功能互换左右手小指 swing/MCP 的关节名。对应关系为：

| 原 CAD 关节后缀 | 当前项目关节后缀 | 左手轴向 / 限位（rad） |
| --- | --- | --- |
| pinky_rotaback | pinky_MCP | Y / 0～1.57 |
| pinky_MCP | pinky_swing | X / -0.35～0.35 |
| pinky_swing | pinky_PIP | Y / 0～1.57 |
| pinky_PIP | pinky_DIP | Y / 0～1.57 |

仅交换关节名及对应引用，运动学、link 名与 STL 不变，已有关键点和碰撞几何仍挂在原 link 上。MJCF、关节配置、A3.2 转换器和模型生成工具同步采用该命名；小指 MCP 为屈伸，swing 为侧摆。完整源文件映射见 `source_mapping.yaml`。改名前记录中的两个关节键需互换解释。

新 CAD 掌宽沿 X 方向，右手关于 YZ 平面镜像：位置/网格使用 `M=diag(-1,1,1)`，旋转使用 `M R M`，
转轴使用 `-M axis`，同时镜像惯性张量和 STL 三角形绕序。因此左右同角度对应镜像姿态。
两侧均为 20 个独立可动关节，不继承 V2 的固定小指 rota 规则。

## 手型切换与检查

在项目根目录的 PowerShell 中运行：

```powershell
conda activate hand_dex
python tools/view_hand_profile.py --model TsenseI_v3_2 --hand-side left
```

窗口中可以切换 V2/V3.2、左右手，调整关节弧度并查看碰撞胶囊与接触数。
滑块按 URDF 限位控制，用于检查模型，不会自动避碰，不连接 USB。
原 `example/main.py` 和 `online_control/online_motion_track/pipeline_v2/main.py` 仍使用原 V2 控制链路；
MediaPipe 实时接入使用独立的 `online_control/online_motion_track/pipeline_v3_2/main.py`，见
[V3.2 实时手势说明](mediapipe_v3_2.md)。Quest 入口尚未对接此手型会话。

## 标定

```powershell
python tools/edit_robot_keypoints_21.py --model TsenseI_v3_2 --hand-side left
```

默认使用三维编辑器，加载正确 V3.2 的 URDF、link 列表和新模型生成的 21 点草稿。

- 空白处左键拖动：绕模型旋转；滚轮：缩放；右键或中键拖动：平移。
- 点击编号：选择关键点；拖动关键点：在当前屏幕平面内移动。
- 拖动红/绿/蓝 XYZ 手柄：沿该点所属 link 的局部轴移动，适合调整模型内部的关节中心。
- Shift+点击模型：将选中点贴到**当前所属 link** 的原始网格表面；不会自动改变所属 link。
- “只显示当前点所属 link”：隔离部件，避免遮挡；当前部件用橙色高亮。
- “聚焦当前点”：放大局部；“显示整手”：恢复取景；3D / +X / +Y / +Z：切换视角。
- Solid shaded / X-ray：切换实体或透视显示；编号和骨架作为覆盖层始终可见。

三维视图采用正交投影，直接显示 URDF 引用的原版 STL 全部三角形，不进行减面或外壳替代。
完整网格由 GPU 绘制，表面点击与显示共用原始三角形；相机未变化时缓存模型画面，
关键点和轴手柄单独更新。右侧数值框显示毫米，YAML 保存对应 link 的局部米制坐标。
切换点的所属 link 会保留该点世界位置，并重新计算局部坐标。
点击 Save 保存到 `assets/hands/TsenseI_hand_v3_2/robot_keypoints_21.yaml`，原文件自动备份。

初始配置来自本次 CAD 的腕部网格、关节位置与网格指尖，标记为 Draft，没有复制 V2 或旧 V3.2 的标定数值。
腕点使用腕部网格近端；四指 MCP 使用近端弯曲关节位置，并换算到选定 link 的局部坐标。
请重点检查腕点 0、拇指 1～4、各指 MCP/PIP/DIP 与指尖。
可手动修改 point 所属 link，以便点跟随正确的指节运动。

如需保留原两张二维图，可追加 `--view 2d`；默认 `--view 3d` 无需填写。

完成左手标定后，将标定镜像到右手：

```powershell
python tools/prepare_hand_v3_2.py --sync-right-keypoints
```

此命令只更新右手关键点文件，并备份之前的右手文件，不重新生成 URDF/MJCF。
如需右手独立微调：

```powershell
python tools/edit_robot_keypoints_21.py --model TsenseI_v3_2 --hand-side right
```

右手独立微调后再次执行镜像命令会覆盖这些右手调整（有备份）。
标定后导出检查图：

```powershell
python tools/render_robot_keypoints_21.py --model TsenseI_v3_2 --hand-side left --output assets/hands/TsenseI_hand_v3_2/qa/keypoints_left.png
python tools/render_robot_keypoints_21.py --model TsenseI_v3_2 --hand-side right --output assets/hands/TsenseI_hand_v3_2/qa/keypoints_right.png
```

## 碰撞模型

`mjcf/mjmodel.xml` 和 `mjcf/mjmodel_right.xml` 分别对应左右手。
每侧 20 个关节/位置驱动器、15 个指节胶囊和 1 个掌部碰撞垫。
关节 origin、轴向、范围与 URDF 一致；胶囊为简化几何，不是完整网格包络。
同指碰撞关闭，跨指启用，掌垫与每指末端两节启用接触；掌内安装块不参与碰撞。
惯量、阻尼和驱动增益是仿真代理参数，没有作实机动力学辨识。

可编辑 `collision_geometry.yaml` 中的左手 link 局部米制几何，随后生成两侧：

```powershell
python tools/generate_profile_collision.py
python tools/render_profile_collision.py
```

生成器保留已有 `collision_geometry.yaml`，右手碰撞几何自动镜像。
初始零位两侧均无接触；随机姿态检测能够产生接触。覆盖效果还应结合标定后的握拳、捏合检查。

## 软件接口与实机预留

`src/hand_profiles.py` 提供 `load_hand_profile` 和 `HandModelSession`。
每个会话拥有独立的关节范围、TopoRetarget 求解器和 MuJoCo 状态；切换时新建会话，
不改变 V2 的全局常量。`retarget(points_21, collision=True)` 接收对应手别、机器人腕坐标系
中的 21 点，输出当前模型的 URDF 弧度字典；像素、相机、Quest 坐标转换需在上游完成。
这里的 21 个输入点和 20 个输出关节是不同概念。

`HardwareAdapter.send_joint_targets(model_id, hand_side, joint_values)` 是真机预留通路。
V3.2 配置中 `hardware.enabled=false`，调用下发将明确报“未配置 adapter”，不会套用
V2 的 14 直线/7 旋转通道和连杆参数。A3.2 独立转换现由 `src/a32_kinematics` 提供，
已接入 A3.2 摄像头/网页流程及日志，默认快速查表（`lut`），输出14路行程和6路旋转；
固件槽位、方向及遥测回传仍待配置。
接口与性能见 [角度转行程说明](angle_to_stroke.md)。
本轮不修改动作角度文件或其格式。

## 验证与重建

```powershell
New-Item -ItemType Directory -Force .pytest-runtime | Out-Null
python -m pytest tests/test_hand_profile_v3_2.py tests/test_keypoint_editor_3d.py -q --basetemp .pytest-runtime/v3_2_tests
```

验证重命名前后 CAD FK 不变、左右 URDF/网格/关键点镜像、MJCF 与 URDF 多姿态 FK/限位一致、
零位无接触及非零姿态碰撞检出、两侧 TopoRetarget 与碰撞接口、V2 全局配置不变、真机通路关闭。
三维编辑器测试覆盖原网格射线求交、投影反算、鼠标旋转/局部轴拖动、所属 link 切换、保存备份与重载。

只有需要从源 CAD 重建模型时才执行 `python tools/prepare_hand_v3_2.py`。
它会重建 V3.2 URDF/网格/分组和零位配置，但保留已有关键点、hand_profile 和碰撞几何 YAML；
重建后需重新运行碰撞生成器。不会写入原始 CAD 或 V2 模型。
如果来源路径或内容指纹与现有 `source_mapping.yaml` 不同，则先将旧模型整个归档，再创建新资源，
避免继承不适用于新几何的关键点或碰撞参数。
