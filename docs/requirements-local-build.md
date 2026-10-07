# 本地编译与依赖安装说明

本文档说明本仓库 `requirements.txt` 在 **Windows + Python 3.11（conda 环境 `hand_dex`）** 下，哪些包需要本地编译、需要哪些工具，以及推荐安装顺序。

适用环境（实测）：

| 项 | 值 |
| --- | --- |
| OS | Windows 10/11 x64 |
| Python | 3.11（Anaconda / Miniconda） |
| 推荐环境名 | `hand_dex` |

---

## 1. 总览：哪些要编译？

多数 `pip` 依赖有现成 **wheel**，直接安装即可。需要本地编译或必须走 **conda** 的主要是：

| 依赖 | 版本 | 安装方式 | 原因 |
| --- | --- | --- | --- |
| **ruckig** | `0.17.3` | **本地编译**（pip 源码） | Win/py3.11 常无对应 wheel |
| **pinocchio** | 建议 4.x | **conda-forge** | 不要用 pip；C++ 绑定复杂 |
| **hidapi** | — | **conda-forge** | 勿与 pip `hidapi` 混用 |
| 其余 `requirements.txt` | 见文件 | 一般 `pip install` | 有预编译包 |

其余如 PySide6、numpy、opencv、mujoco、daqp、mediapipe 等，在该环境下通常可直接装 wheel。

---

## 2. 系统工具（本地编译 ruckig 必备）

### 2.1 Visual Studio C++ 构建工具

- 安装 [**Visual Studio Build Tools**](https://visualstudio.microsoft.com/visual-cpp-build-tools/)（或完整 VS）
- 勾选工作负载：**使用 C++ 的桌面开发**
- 确认存在 `cl.exe`（MSVC）
- 实测可用：MSVC 14.x / “Visual Studio 18 2026” 工具集亦可

未装 VS / 未开 Developer 环境时，CMake 配置会失败。

### 2.2 Python 侧构建依赖（装进 `hand_dex`）

```powershell
conda activate hand_dex

python -m pip install ^
  "scikit-build-core==0.9.10" ^
  cmake ^
  ninja ^
  nanobind ^
  packaging ^
  pathspec ^
  -i https://pypi.org/simple/
```

| 包 | 作用 |
| --- | --- |
| `scikit-build-core==0.9.10` | 构建后端；**必须 &lt; 0.10**（0.17.3 的 `pyproject` 用旧字段 `cmake.targets`） |
| `cmake` / `ninja` | 生成与编译 |
| `nanobind` | ruckig 的 Python 绑定（CMake `find_package(nanobind)`） |
| `packaging` / `pathspec` | scikit-build-core 依赖 |

> **注意：** 默认 `pip install ruckig` 会启用 **build isolation**，隔离环境里仍可能拉到新版 `scikit-build-core`，报错：
> `Use build.targets instead of cmake.targets for scikit-build-core >= 0.10`  
> 因此必须使用 **`--no-build-isolation`**，并事先装好上面这些包。

### 2.3 可选：镜像说明

- 清华等国内镜像可能对旧版本 **只有 sdist、没有 win wheel**，且构建依赖不全。
- 编译 ruckig 时建议改用官方源：`-i https://pypi.org/simple/`。

---

## 3. 编译并安装 ruckig==0.17.3

在已安装第 2 节工具的前提下：

```powershell
conda activate hand_dex

python -m pip uninstall -y ruckig

python -m pip install "ruckig==0.17.3" ^
  --no-build-isolation ^
  --no-binary=ruckig ^
  --force-reinstall ^
  -i https://pypi.org/simple/
```

验证：

```powershell
python -c "import ruckig; print(ruckig.__version__)"
# 期望输出: 0.17.3
```

### 常见失败与处理

| 现象 | 处理 |
| --- | --- |
| `Use build.targets instead of cmake.targets...` | 固定 `scikit-build-core==0.9.10` + `--no-build-isolation` |
| `Could not find ... nanobind` | `pip install nanobind` 后重试 |
| 找不到 C++ 编译器 / CMake 配置失败 | 安装 VS Build Tools（含 MSVC） |
| `No matching distribution ... only-binary` | 0.17.3 本就无 win wheel，不要加 `--only-binary=:all:` |
| 已装成 `0.19.4` | 按上面命令 `--force-reinstall` 回退到 `0.17.3` |

若只求能跑、不强制 0.17.3，Win 上也可直接装带 wheel 的版本（如 `0.19.4`），但本仓库手势下行按 **`0.17.3`** 对齐。

---

## 4. 推荐环境搭建顺序

### 4.1 创建 conda 环境

```powershell
conda create -n hand_dex python=3.11 -y
conda activate hand_dex
```

### 4.2 先装必须走 conda 的包

```powershell
conda install -n hand_dex -c conda-forge ^
  pinocchio pinocchio-python nlopt casadi pybullet hidapi -y
```

说明：

- **Pinocchio**：FK/解析雅可比；`import pinocchio`，勿 `pip install pinocchio`。
- **hidapi**：与 USB HID 相关；只用 conda 版。

### 4.3 再装 pip 依赖（含需编译的 ruckig）

1. 先完成 **第 2、3 节**（编译工具 + ruckig 0.17.3）。
2. 再安装其余依赖：

```powershell
cd D:\Qtworks\hand
python -m pip install -r requirements.txt
```

若 `requirements.txt` 再次触发 ruckig 构建失败，可：

```powershell
python -m pip install -r requirements.txt --no-deps
python -m pip install -r requirements.txt
```

或暂时注释掉其中的 `ruckig==0.17.3` 行（已手工装好时）。

### 4.4 可选后端

- **WiLoR**：见 `requirements.txt` 注释；权重在本地 `external/`（默认不入库）。
- **torch**：按需 CPU/CUDA wheel。
- **HandOccNet**：可选，见 `tracker_config.py` / `HANDOCCNET_ROOT`。

---

## 5. 与项目相关的其它编译/原生依赖（非 pip 源码包）

| 组件 | 建议 |
| --- | --- |
| Pinocchio | conda-forge，见上 |
| MuJoCo | `mujoco==3.10.0` 一般有 wheel |
| daqp | `daqp==0.8.7` 一般有 wheel；失败再查 C 编译器 |
| RealSense `pyrealsense2` | 优先 wheel；驱动需 Intel RealSense SDK |
| OpenGL / moderngl | 需正常显卡驱动；无额外 C++ 编译 |

---

## 6. 快速检查清单

```powershell
conda activate hand_dex
python -c "import sys; print(sys.version)"
python -c "import ruckig; print('ruckig', ruckig.__version__)"
python -c "import pinocchio; print('pinocchio ok')"
python -c "import mujoco, daqp, cv2; print('core ok')"
python -c "from utils.ruckig_interpolator import RuckigJointInterpolator; print('interpolator ok')"
```

启动示例：

```powershell
cd D:\Qtworks\hand
$env:PYTHONPATH="D:\Qtworks\hand\src"
python example\main.py
```

---

## 7. 一键回顾：ruckig 最小命令集

```powershell
conda activate hand_dex

python -m pip install "scikit-build-core==0.9.10" cmake ninja nanobind packaging pathspec -i https://pypi.org/simple/

python -m pip install "ruckig==0.17.3" --no-build-isolation --no-binary=ruckig --force-reinstall -i https://pypi.org/simple/

python -c "import ruckig; print(ruckig.__version__)"
```

系统侧还需已安装 **Visual Studio C++ Build Tools（MSVC）**。
