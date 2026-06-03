# 部署指南（全新机器，不依赖 ROS）

`rebotarm_control_rt` 含两个原生模块，需在**目标机上现编**：

| 模块 | 工具链 | 依赖 |
|---|---|---|
| `_native.so` | Rust / PyO3 | 同级目录 `../motorbridge` 的 Rust crates |
| `_math.so` | C++ / pybind11 | **Pinocchio C++ 3.x** + Eigen |

> 为什么要现编：`_math.so` 的 RPATH 烘进了本机 Pinocchio 库路径，**不能跨机器拷 wheel**。

---

## 一键部署（两阶段）

```bash
# 0) 目录结构：rebotarm_control_rt 与 motorbridge 必须同级
#    某父目录/{motorbridge, rebotarm_control_rt}
cd rebotarm_control_rt

# 阶段 1：创建环境（Python 默认 3.12，可指定）+ Pinocchio C++3.x/Eigen/cmake/编译器
bash ./setup_env.sh --mamba rebot          # Miniforge；或 --conda rebot（Miniconda/Anaconda）
#   指定 Python 版本： bash ./setup_env.sh --mamba rebot 3.11

# 阶段 2：激活后安装本包（装 rust/maturin → build.sh --wheel → 自检）
mamba activate rebot                        # 或 conda activate rebot
bash ./setup_env.sh --install
```

验证（无需硬件）：

```bash
pip install pytest                 # 测试用依赖（--install 不装它）
bash ./run_tests.sh                # 期望：xx passed + 若干 skipped（无机械臂时跳过硬件测试）
```

> 直接 `pytest` 若报 `ModuleNotFoundError: No module named 'lark'`，是你的 shell source 了 ROS、
> `$PYTHONPATH` 把 ROS 的 pytest 插件（launch_testing）带了进来。用 `run_tests.sh` 即可（它会
> 剥掉 `PYTHONPATH` 并关闭第三方插件自动加载）。

`setup_env.sh` 参数：`--mamba/--conda [env_name] [py_version]`（env 默认 `rebot`，py 默认 `3.12`，
也可用环境变量 `PYVER`）。conda-forge 原生依赖（`pinocchio>=3.2,<4`、`eigen`、`cmake`、`cxx-compiler`）
内联在 `setup_env.sh` 的 `CONDA_FORGE_DEPS` 里。

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

# 3) Pinocchio C++ 3.x + Eigen + cmake + 编译器（关键：钉 3.x，4.0 头文件布局不兼容本代码）
conda install -c conda-forge "pinocchio>=3.2,<4" eigen cmake cxx-compiler -y

# 4) Python 构建/运行依赖
pip install maturin pybind11 numpy pyyaml

# 5) 构建
./build.sh --wheel
```

Pinocchio C++ 前缀由 `cpp/CMakeLists.txt` **自动探测**，顺序：
`-DPINOCCHIO_PREFIX` > `$PINOCCHIO_PREFIX` > `$CONDA_PREFIX` > `/usr/local` > `/usr`；
自动适配 `lib` 与 `lib/x86_64-linux-gnu`，自动定位 Eigen。手动指定示例：

```bash
PINOCCHIO_PREFIX=/opt/openrobots ./build.sh --wheel
```

---

## 硬件与实时权限

```bash
# 串口免 sudo（Damiao @ /dev/ttyACM0, 921600）：加入 dialout 组后重新登录
sudo usermod -aG dialout "$USER"

# 让 RT 循环可申请 SCHED_FIFO（仅 PREEMPT_RT 内核有意义）：
sudo setcap cap_sys_nice+ep "$(readlink -f "$(which python)")"   # 或直接 sudo 运行
# 然后：arm.start_rt_loop(rt_priority=80, cpu=2)
```

---

## 常见问题

- **`fatal error: 找不到 Pinocchio C++`**：未装或前缀不对。`conda install -c conda-forge "pinocchio>=3.2,<4"`，或 `-DPINOCCHIO_PREFIX=` 指定。
- **编译报 `se3-base.hpp ... 'math'/'PI' was not declared`**：装成了 **Pinocchio 4.0**（头文件布局变了），或机器上存在 `/usr/local` 的旧 pinocchio 抢占。请确保 conda 里是 3.x：
  `conda install -c conda-forge "pinocchio>=3.2,<4"`。
- **`import` 报 `libpinocchio_*.so not found`**：把别处编好的 wheel 拷过来了（RPATH 失配）。请在本机重编。
- **`ImportError: cannot import name '_native'`**：用 `PYTHONPATH=python` 跑源码树但还没 `./build.sh --wheel`（它会把 `_native.so` 释放进 `python/`）。装了 wheel 则不受影响。
- **`pinocchio` Python 包冲突**：本项目**不用** Python 的 `pinocchio/pin`，全走 C++。无需安装，装了也不影响。
