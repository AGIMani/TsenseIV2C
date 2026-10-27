# A3.2 带 ID 模型检查

2026-10-27 替换为用户提供的 `TsenseI_hand_A3.2_withid_upper_thumb5mm_noupper_mid`。
网页仍通过 `TsenseI_v3_2_withid` 选择此模型，左右手各保留 22 个 link、20 个活动关节、367 个完整外观网格及源 URDF 的 22 组质量惯量（总质量 0.952 kg）。

## 坐标与控制

源文件原样保存在 `../source_20_outputs/`，与运行模型共享未经修改的 `../meshes/`。源包的配置和审计资料保存在 `../source_package/`，仅用于溯源，不作为项目控制策略。

原包拇指第二旋转关节范围为 [-1.74, 0] rad。本次按用户要求同时修改源目录左右手 URDF：采用 `q_new = -q_old` 并翻转该关节轴，源文件与项目现在均使用 [0, 1.74] rad。原文件以 `.urdf.bak-*` 保存在源目录；项目源快照及哈希同步更新。源包配套配置/历史审计仍作为原始记录保存。关节原点和其他轴保留源值。屈曲范围继续使用 [0.005, 1.565] rad，侧摆模型范围为 [-0.35, 0.35] rad。

右手直接使用提供的右手文件，镜像面位于 base_link 的 Y=4 mm，满足 `y_right=0.008-y_left`。关键点镜像工具已适配该平面偏移。每只手保留原来的局部连杆关键点，随新几何运动，标记为 Draft/待复核，不能视作新 CAD 的实测标定。

## 碰撞模型

所有 44 个连杆的碰撞包络均从各自完整外观网格重新计算，验证源顶点被包络覆盖，凸包外扩误差不超过 0.35 mm。运行 MJCF 每只手启用 15 个手指包络，继续只检查不同手指之间的碰撞；手掌、腕座、同指碰撞不在此策略内。MJCF 沿用运动学碰撞代理的质量惯量，不是源质量参数的动力学仿真模型。

本目录原有预览图属于旧模型历史记录，不能作为本次模型的验收图片。

## 导入与验证

在项目根目录使用 track 环境：

```bash
.venvs/track/bin/python tools/import_a32_withid_package.py /path/to/TsenseI_hand_A3.2_withid_upper_thumb5mm_noupper_mid
.venvs/track/bin/python -m pytest -q tests/test_a32_models_integration.py tests/test_a32_joint_ranges.py tests/test_a32_distal_collision.py tests/test_web_v3_2.py
```

导入先在临时目录构建，成功后将旧模型目录移入项目外的 `../model-backups/`，再安装新模型。来源网格、原始 URDF、运行 URDF 和 MJCF 的 SHA-256 见 `../source_mapping.yaml`。

验证未连接或驱动机械手。已运行的 track/play 服务需重新加载模型；重启对应服务并刷新网页可确保采用新资产。
