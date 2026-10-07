# V2 金属渲染工作室

独立 Python / PySide6 工具，加载项目当前默认模型
`assets/hands/TsenseI_hand/urdf/TsenseI__hand_left_v1.urdf`（用户确认的 V2）。
不连接相机、USB 或实体机械手，不修改原始 URDF / STL。

## 启动

在项目根目录执行：

```powershell
& E:\conda\envs\hand_dex\python.exe tools\metal_studio.py
```

或者运行 `tools/start_metal_studio.ps1`。已激活 `hand_dex` 时也可执行
`python tools/metal_studio.py`。独立安装所需的最小依赖：

```powershell
python -m pip install -r tools/metal_studio_requirements.txt
```

要求支持 OpenGL 3.3 的显卡驱动。当前机器使用 RTX 3050 Laptop 验证。

## 操作

- 默认加载唯一的全零 `init` 姿态；「init（全零）」按钮恢复此状态。对指是手动选择的动作：模型末端表面点目标间距为 1.5 mm，可设为 0–30 mm 后点击「中指对指」，命令行可用 `--pose opposition`。
- 「关节」按五指分组，滑杆与数值框同步；旋转关节显示度，范围读取 URDF。
- 左键拖动旋转，右键 / 中键 / Shift+左键平移，滚轮缩放。双击或 F 居中。
- 「材质 / 灯光」可选择红铜、银色、香槟金、钛灰，调节颜色、金属度、粗糙度、曝光及灯光。选择单个部件后可独立改色；应用全局预设会清空部件覆盖。
- 「取景」调相机角度、视场角、正交投影、背景及阴影。
- 「出图」画幅同时应用到预览与输出。设置 PNG 长边像素，最高 8192 px；高分辨率需要相应显存。
- Ctrl+E 导出纯渲染 PNG，不包含 UI；透明背景自动隐藏地面。每次导出附带 `.scene.json`。
- Ctrl+S 保存关节、材质、灯光、相机及画幅；「载入场景」恢复。
- Tab 收起 / 展开控制面板；F11 全屏。

## 无界面出图

```powershell
python tools/metal_studio.py --pose opposition --render renders/metal_studio/v2_copper_opposition.png --width 2400 --height 2400
python tools/metal_studio.py --render renders/metal_studio/v2_silver.png --material silver --transparent
python tools/metal_studio.py --scene renders/metal_studio/v2_copper_opposition.scene.json --render renders/metal_studio/reexport.png
python tools/metal_studio.py --urdf assets/hands/TsenseI_hand/urdf/TsenseI__hand_right_v1.urdf
```

命令行与 UI 使用相同渲染器。载入场景时默认保留相机及画幅；可以显式指定输出宽高。

## 渲染与姿态说明

渲染保留 STL 原始三角面，平滑曲面法线并保留 CAD 锐边。金属着色采用
GGX / Smith / Schlick，配合程序化摄影棚反射、阴影及 MSAA。
程序化反射是实时近似，没有离线路径追踪或反射中的相互遮挡。
参考：[PBR 光照原理](https://learnopengl.com/PBR/Lighting)、
[ModernGL 文档](https://moderngl.readthedocs.io/en/stable/)、
[Qt OpenGL Widget](https://doc.qt.io/qt-6/qopenglwidget.html)。

STL 未提供逐零件 CAD 材质信息，因此默认按照 link 配色，可在 UI 中覆盖。
自动对指使用拇指、中指实际末端网格的端部位置进行有界最小二乘求解，约束端点距离和末端朝向。
支持当前左右手模型；其他 URDF 可打开并调整关节，若无法识别末端命名会禁用自动对指。
显示的间距是端点间距，并非整张网格的最近距离；不包含全手碰撞检测，也不生成硬件控制指令。

## 验证

```powershell
python -m pytest tests/test_metal_studio.py -q
```

覆盖左右手对指、关节限位 / 无效输入、场景往返、透明 PNG、材质 / 姿态变更产生的像素变化及关节面板同步。
