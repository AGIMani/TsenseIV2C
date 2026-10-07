# A3.2 带 ID 模型检查

## 碰撞模型

每只手有 15 个碰撞几何：10 个近端/中间指节胶囊，加 5 个完整末端外壳凸包。只检查不同手指之间的碰撞；同指、手掌、安装底座和腕壳不参与此模型的碰撞保护。

上传包的末端胶囊仅根据指尖最后 1 mm 的截面拟合，直径约 4–4.8 mm，不能覆盖宽约 14.5–16 mm 的指腹。本项目因此从每个末端 link 的**全部视觉网格顶点**构建凸包，包含 URDF 的 visual origin 和 scale，并保存到 `../mjcf/collision_meshes/*.stl`。凸包保留收尖外形，同时覆盖整个指腹；它会保守填平局部凹槽。

生成器使用 SciPy ConvexHull，并按凸包平面法向设置三角形绕序。保留原始顶点精度，避免 STL 导出前自动合并非常接近的顶点而产生退化面。右侧网格使用模型规定的 XZ 镜像并反转绕序。

`collision_overlay.png` 显示左右手的掌面、侧面和屈曲姿态。彩色区域包括 10 个胶囊和 5 个末端凸包，灰色区域是原始视觉外壳。两只手零位均无接触；普通屈曲姿态也无假碰撞。

浏览器既有胶囊辅助显示接口返回 10 个胶囊；5 个末端凸包由真实末端模型表面表示，均参与后台 MuJoCo 距离计算和 stop 保护。此图额外绘制完整凸包轮廓。

## 验证

2026-09-28：以下 4 个测试文件合计 **70 项通过**。

```powershell
E:/conda/envs/hand_dex/python.exe -m pytest tests/test_a32_distal_collision.py tests/test_a32_models_integration.py tests/test_a32_mapper.py tests/test_hand_profile_v3_2.py -q --basetemp .pytest-runtime/a32-models-full-hull-04
```

新增碰撞检查包括：

- 验证五个凸包覆盖各自原始视觉网格的所有顶点，边界误差小于 20 nm；验证闭合、凸性、右侧镜像。
- 五根手指、左右手分别检查指腹对碰和指尖对碰：1 mm 间隙时无接触，向内推进 0.5 mm 时均检出接触。
- 整手弯曲交叉姿态中检出末端凸包之间的碰撞；确认正负距离进入 stop guard，并将目标截停在无穿透姿态。
- 随机姿态检查 URDF 与 MJCF 的关节限位及正向运动学一致，初始零位不碰撞。

## 重新生成

在项目根目录运行：

```powershell
python tools/prepare_hand_v3_2_withid.py --sdk-root D:/Qtworks/a3-cpp-sdk
python tools/render_profile_collision.py --model TsenseI_v3_2_withid
```

来源和生成的碰撞网格 SHA-256 记录在 `../source_mapping.yaml`。重新生成保留已存在的 21 点人工标定，以及非末端碰撞几何调整。硬件标定、原模型误差和碰撞代理近似仍需按实际设备校验。
