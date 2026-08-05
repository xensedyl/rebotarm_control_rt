# 部署指南（全新机器，不依赖 ROS）

`rebotarm_control_rt` 含两个原生模块，需在**目标机上现编**：

| 模块 | 工具链 | 依赖 |
|---|---|---|
| `_native.so` | Rust / PyO3 | 本地 `motorbridge` 源码（经 `.deps/motorbridge` 链接） |
| `_math.so` | C++ / pybind11 | **Pinocchio C++ 3.x** + Eigen |

> 为什么要现编：`_math.so` 的 RPATH 烘进了本机 Pinocchio 库路径，**不能跨机器拷 wheel**。

---

## 一键部署（两阶段）

```bash
# 0) motorbridge 可以位于任意本地目录；脚本会自动查找，也可设置：
#    export REBOTARM_MOTORBRIDGE_ROOT=/path/to/motorbridge
cd rebotarm_control_rt

# 阶段 1：创建环境（Python 默认 3.12，可指定）+ Pinocchio C++3.x/Eigen/cmake/编译器
bash ./setup_env.sh --mamba rebot          # Miniforge；或 --conda rebot（Miniconda/Anaconda）
#   指定 Python 版本： bash ./setup_env.sh --mamba rebot 3.11

# 阶段 2：激活后安装本包（装 rust/maturin → build.sh --wheel → 自检）
mamba activate rebot                        # 或 conda activate rebot
bash ./setup_env.sh --install
```

已有环境（例如 `xense-head`）无需向当前 Conda 环境安装 Pinocchio：

```bash
conda activate xense-head
bash ./setup_env.sh --install
```

如果 `xense-head` 没有兼容的 Pinocchio 3.x，脚本会自动创建
`rebotarm_control_rt/.deps/pinocchio`，其中固定使用 conda-forge 的
`libpinocchio=3.9.*`。这个前缀与当前环境隔离，不使用 ROS，也不会改变 RoboStack、Boost、
CUDA 等已有依赖。首次准备约需下载 200 MB；之后会直接复用。

也可以只准备依赖、不构建：

```bash
bash ./setup_env.sh --prepare-pinocchio
```

验证（无需硬件）：

```bash
pip install pytest                 # 测试用依赖（--install 不装它）
bash ./run_tests.sh                # 期望：xx passed + 若干 skipped（无机械臂时跳过硬件测试）
```

> 直接 `pytest` 若报 `ModuleNotFoundError: No module named 'lark'`，是你的 shell source 了 ROS、
> `$PYTHONPATH` 把 ROS 的 pytest 插件（launch_testing）带了进来。用 `run_tests.sh` 即可（它会
> 剥掉 `PYTHONPATH` 并关闭第三方插件自动加载）。

`setup_env.sh` 参数：`--mamba/--conda [env_name] [py_version]`（env 默认 `rebot`，py 默认 `3.12`）、
`--prepare-pinocchio` 和 `--install`。新建的纯 conda 环境仍直接安装 Pinocchio；已有复杂环境
缺少 Pinocchio 时使用仓库内的隔离前缀。可用 `PINOCCHIO_PREFIX` 指定已有前缀，或用
`REBOTARM_PINOCCHIO_PREFIX` 修改私有前缀位置。

Rust 电机后端通过 `.deps/motorbridge` 链接源码。脚本依次查找现有链接、同级
`../motorbridge`、`~/rebot_lerobot/motorbridge`，最后在 HOME 下做一次有限深度搜索。
也可显式设置 `REBOTARM_MOTORBRIDGE_ROOT=/path/to/motorbridge`。

---

## 前置条件

- Linux x86_64（RT 调度可选，PREEMPT_RT 内核才有意义）。
- 已安装 **conda**（推荐 [Miniforge](https://github.com/conda-forge/miniforge)）。`setup_env.sh` 不会替你装 conda。
- 能访问网络（拉 rust / conda-forge / pip 包）。

---

## 手动分步（等价于 setup_env.sh）

```bash
# 1) Rust
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y && source $HOME/.cargo/env

# 2) conda 环境
conda create -n rebot python=3.12 -y && conda activate rebot

# 3) 纯 conda 环境可直接安装 Pinocchio C++ 3.x + Eigen + cmake + 编译器
#    已有大型环境请跳过此命令，改用 setup_env.sh --prepare-pinocchio
conda install -c conda-forge "pinocchio>=3.2,<4" eigen cmake cxx-compiler -y

# 4) Python 构建/运行依赖
pip install maturin pybind11 numpy pyyaml

# 5) 构建
./build.sh --wheel
```

Pinocchio C++ 前缀由 `cpp/CMakeLists.txt` **自动探测**，顺序：
`-DPINOCCHIO_PREFIX` > `$PINOCCHIO_PREFIX` > `$CONDA_PREFIX` > `.deps/pinocchio` >
`/usr/local` > `/usr`；
自动适配 `lib` 与 `lib/x86_64-linux-gnu`，自动定位 Eigen。手动指定示例：

```bash
PINOCCHIO_PREFIX=/opt/openrobots ./build.sh --wheel
```

可选 MeshCat 仿真示例还需要 Python 可视化依赖：

```bash
pip install meshcat
conda install -c conda-forge "pinocchio>=3.2,<4"
```

如果当前 shell source 过 ROS，运行仿真示例时同时清掉 ROS 的 Python 路径和动态库路径：
`env -u PYTHONPATH -u LD_LIBRARY_PATH python example/sim/fk_sim.py`。

---

## 硬件与实时权限

```bash
# 串口免 sudo（Damiao @ /dev/ttyACM*，921600）：加入 dialout 组后重新登录
# 先用 ls 确认实际端口，再把 config/arm.yaml / config/gripper.yaml 的 channel 改成对应 tty。
ls -l /dev/ttyACM* /dev/ttyUSB*
sudo usermod -aG dialout "$USER"

# 让 RT 循环可申请 SCHED_FIFO（仅 PREEMPT_RT 内核有意义）：
sudo setcap cap_sys_nice+ep "$(readlink -f "$(which python)")"   # 或直接 sudo 运行
# 然后：arm.start_rt_loop(rt_priority=80, cpu=2)
```

---

## 常见问题

- **`fatal error: 找不到 Pinocchio C++`**：运行 `bash setup_env.sh --prepare-pinocchio`，
  或用 `PINOCCHIO_PREFIX=` 指定已有的 Pinocchio 3.x。不要在带 RoboStack/复杂 ABI pin 的环境里
  单独执行 `conda install pinocchio`。
- **编译报 `se3-base.hpp ... 'math'/'PI' was not declared`**：装成了 **Pinocchio 4.0**（头文件布局变了），或机器上存在 `/usr/local` 的旧 pinocchio 抢占。请确保 conda 里是 3.x：
  取消错误的 `PINOCCHIO_PREFIX` 后运行 `bash setup_env.sh --prepare-pinocchio`。
- **`import` 报 `libpinocchio_*.so not found`**：把别处编好的 wheel 拷过来了（RPATH 失配）。请在本机重编。
- **`ImportError: cannot import name '_native'`**：用 `PYTHONPATH=python` 跑源码树但还没 `./build.sh --wheel`（它会把 `_native.so` 释放进 `python/`）。装了 wheel 则不受影响。
- **`pinocchio` Python 包冲突**：本项目**不用** Python 的 `pinocchio/pin`，全走 C++。无需安装，装了也不影响。
