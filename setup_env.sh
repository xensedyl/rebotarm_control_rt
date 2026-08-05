#!/bin/bash
# rebotarm_control_rt 环境准备 + 安装（两阶段，不依赖 ROS）。
#
# 阶段 1：创建 conda/mamba 环境（Python 默认 3.12，可指定）+ Pinocchio C++3.x/Eigen/cmake/编译器
#   bash ./setup_env.sh --mamba [env_name] [py_version]     # Miniforge
#   bash ./setup_env.sh --conda [env_name] [py_version]     # Miniconda/Anaconda
#
# 阶段 2：在已激活的环境中安装本包（装 rust/maturin/pybind11 → build.sh --wheel）
#   mamba activate <env_name>      # 或 conda activate <env_name>
#   bash ./setup_env.sh --install
#
# 示例：
#   bash ./setup_env.sh --mamba rebot          # Python 3.12
#   bash ./setup_env.sh --mamba rebot 3.11      # 指定 Python 3.11
#   mamba activate rebot
#   bash ./setup_env.sh --install

set -uo pipefail

# 操作系统检查（仅提示，不阻断）
OS_NAME=$(uname -s)
if [[ "$OS_NAME" != "Linux" ]]; then
    echo "Unsupported operating system: $OS_NAME（本脚本仅支持 Linux）"
    exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SCRIPT_NAME="$(basename "${BASH_SOURCE[0]}")"

# conda-forge 原生依赖（内联，不用 yaml）。
# Pinocchio 钉死 3.x —— 4.0 重排了头文件布局，本项目按 3.x 编写。
CONDA_FORGE_DEPS=( "pinocchio>=3.2,<4" eigen cmake cxx-compiler )

# 在复杂环境（例如已装 RoboStack/ROS/CUDA 的 xense-head）里，直接向当前环境
# conda install Pinocchio 可能触发全局 Boost ABI 冲突。因此 --install 缺少
# Pinocchio 时，默认在仓库内创建隔离的 C++ 依赖前缀，不修改当前环境的 Conda 解。
PRIVATE_PINOCCHIO_PREFIX="${REBOTARM_PINOCCHIO_PREFIX:-$SCRIPT_DIR/.deps/pinocchio}"
PRIVATE_PINOCCHIO_SPEC="${REBOTARM_PINOCCHIO_SPEC:-libpinocchio=3.9.*}"
RESOLVED_PINOCCHIO_PREFIX=""
MOTORBRIDGE_LINK="$SCRIPT_DIR/.deps/motorbridge"
RESOLVED_MOTORBRIDGE_ROOT=""

pinocchio_version_at_prefix() {
    local prefix=$1
    local config="$prefix/include/pinocchio/config.hpp"
    [[ -f "$config" ]] || return 1
    sed -n 's/^[[:space:]]*#[[:space:]]*define[[:space:]]\+PINOCCHIO_VERSION[[:space:]]\+"\([^"]\+\)".*/\1/p' "$config" | head -n 1
}

pinocchio_prefix_is_compatible() {
    local prefix=$1
    local version major minor
    [[ -f "$prefix/include/pinocchio/spatial/se3-base.hpp" ]] || return 1
    version="$(pinocchio_version_at_prefix "$prefix")" || return 1
    IFS=. read -r major minor _ <<<"$version"
    [[ "$major" =~ ^[0-9]+$ && "$minor" =~ ^[0-9]+$ ]] || return 1
    (( major == 3 && minor >= 2 )) || return 1
    compgen -G "$prefix/lib/libpinocchio_default.so*" >/dev/null \
        || compgen -G "$prefix/lib/libpinocchio.so*" >/dev/null \
        || compgen -G "$prefix/lib/"'*linux-gnu/libpinocchio_default.so*' >/dev/null \
        || compgen -G "$prefix/lib/"'*linux-gnu/libpinocchio.so*' >/dev/null
}

resolve_existing_pinocchio() {
    local prefix version
    RESOLVED_PINOCCHIO_PREFIX=""

    # 显式指定时严格校验，避免悄悄换成另一个安装。
    if [[ -n "${PINOCCHIO_PREFIX:-}" ]]; then
        if ! pinocchio_prefix_is_compatible "$PINOCCHIO_PREFIX"; then
            echo "[ERROR] PINOCCHIO_PREFIX=$PINOCCHIO_PREFIX 不是可用的 Pinocchio >=3.2,<4 前缀。"
            return 1
        fi
        RESOLVED_PINOCCHIO_PREFIX="$PINOCCHIO_PREFIX"
        return 0
    fi

    # 顺序：当前环境 -> 项目私有前缀 -> 系统安装。完全不探测 /opt/ros，
    # 保证本项目不会因为 shell source ROS 而隐式链接 ROS 的库。
    for prefix in "${CONDA_PREFIX:-}" "$PRIVATE_PINOCCHIO_PREFIX" /usr/local /usr; do
        [[ -n "$prefix" ]] || continue
        if pinocchio_prefix_is_compatible "$prefix"; then
            RESOLVED_PINOCCHIO_PREFIX="$prefix"
            version="$(pinocchio_version_at_prefix "$prefix")"
            echo "[INFO] 使用 Pinocchio C++ $version：$prefix"
            return 0
        fi
    done
    return 1
}

find_conda_frontend() {
    if [[ -n "${REBOTARM_CONDA_EXE:-}" ]] && command -v "$REBOTARM_CONDA_EXE" >/dev/null 2>&1; then
        printf '%s\n' "$REBOTARM_CONDA_EXE"
    elif command -v mamba >/dev/null 2>&1; then
        printf '%s\n' mamba
    elif command -v conda >/dev/null 2>&1; then
        printf '%s\n' conda
    else
        return 1
    fi
}

prepare_private_pinocchio() {
    local conda_cmd py_ver action
    if pinocchio_prefix_is_compatible "$PRIVATE_PINOCCHIO_PREFIX"; then
        RESOLVED_PINOCCHIO_PREFIX="$PRIVATE_PINOCCHIO_PREFIX"
        return 0
    fi
    conda_cmd="$(find_conda_frontend)" || {
        echo "[ERROR] 未找到 conda/mamba，无法准备项目私有 Pinocchio。"
        echo "        可安装 Miniforge，或设置 PINOCCHIO_PREFIX=<已有 Pinocchio 3.x 前缀>。"
        return 1
    }
    command -v python >/dev/null 2>&1 || {
        echo "[ERROR] 当前 PATH 中没有 python，无法确定私有依赖前缀的 Python ABI。"
        return 1
    }
    py_ver="$(python -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
    mkdir -p "$(dirname "$PRIVATE_PINOCCHIO_PREFIX")"

    if [[ -d "$PRIVATE_PINOCCHIO_PREFIX/conda-meta" ]]; then
        action=install
    else
        action=create
    fi
    echo "[INFO] 当前环境没有兼容的 Pinocchio C++ 3.x。"
    echo "[INFO] 准备项目私有依赖：$PRIVATE_PINOCCHIO_PREFIX"
    echo "[INFO] 该操作只写入仓库 .deps，不修改当前环境，也不依赖 ROS/RoboStack。"
    "$conda_cmd" "$action" -y -p "$PRIVATE_PINOCCHIO_PREFIX" \
        --override-channels -c conda-forge "python=$py_ver" "$PRIVATE_PINOCCHIO_SPEC" || {
        echo "[ERROR] 项目私有 Pinocchio 创建失败。"
        return 1
    }
    if ! pinocchio_prefix_is_compatible "$PRIVATE_PINOCCHIO_PREFIX"; then
        echo "[ERROR] 私有前缀创建完成，但未找到兼容的 Pinocchio C++ 3.x。"
        return 1
    fi
    RESOLVED_PINOCCHIO_PREFIX="$PRIVATE_PINOCCHIO_PREFIX"
}

ensure_pinocchio() {
    if resolve_existing_pinocchio; then
        return 0
    fi
    # 显式 PINOCCHIO_PREFIX 无效时不得静默创建其它前缀。
    [[ -z "${PINOCCHIO_PREFIX:-}" ]] || return 1
    prepare_private_pinocchio || return 1
    echo "[INFO] 使用项目私有 Pinocchio C++ $(pinocchio_version_at_prefix "$RESOLVED_PINOCCHIO_PREFIX")：$RESOLVED_PINOCCHIO_PREFIX"
}

motorbridge_root_is_compatible() {
    local root=$1 vendor
    [[ -f "$root/motor_core/Cargo.toml" ]] || return 1
    for vendor in damiao myactuator robstride hightorque; do
        [[ -f "$root/motor_vendors/$vendor/Cargo.toml" ]] || return 1
    done
}

ensure_motorbridge() {
    local candidate discovered=""
    RESOLVED_MOTORBRIDGE_ROOT=""

    if [[ -n "${REBOTARM_MOTORBRIDGE_ROOT:-}" ]]; then
        if ! motorbridge_root_is_compatible "$REBOTARM_MOTORBRIDGE_ROOT"; then
            echo "[ERROR] REBOTARM_MOTORBRIDGE_ROOT=$REBOTARM_MOTORBRIDGE_ROOT 不是完整的 motorbridge 源码目录。"
            return 1
        fi
        discovered="$REBOTARM_MOTORBRIDGE_ROOT"
    else
        for candidate in \
            "$MOTORBRIDGE_LINK" \
            "$SCRIPT_DIR/../motorbridge" \
            "$HOME/rebot_lerobot/motorbridge"; do
            if motorbridge_root_is_compatible "$candidate"; then
                discovered="$candidate"
                break
            fi
        done
        if [[ -z "$discovered" ]]; then
            candidate="$(find "$HOME" -maxdepth 6 -type f \
                -path '*/motorbridge/motor_core/Cargo.toml' -print -quit 2>/dev/null || true)"
            if [[ -n "$candidate" ]]; then
                discovered="$(dirname "$(dirname "$candidate")")"
            fi
        fi
    fi

    if [[ -z "$discovered" ]] || ! motorbridge_root_is_compatible "$discovered"; then
        echo "[ERROR] 未找到 motorbridge 源码。"
        echo "        请设置 REBOTARM_MOTORBRIDGE_ROOT=/path/to/motorbridge 后重试。"
        return 1
    fi
    discovered="$(cd "$discovered" && pwd -P)"
    mkdir -p "$SCRIPT_DIR/.deps"
    if [[ -e "$MOTORBRIDGE_LINK" && ! -L "$MOTORBRIDGE_LINK" ]]; then
        if [[ "$(cd "$MOTORBRIDGE_LINK" && pwd -P)" != "$discovered" ]]; then
            echo "[ERROR] $MOTORBRIDGE_LINK 已存在且不是目标 motorbridge 的符号链接。"
            return 1
        fi
    elif [[ "$(readlink -f "$MOTORBRIDGE_LINK" 2>/dev/null || true)" != "$discovered" ]]; then
        ln -sfn "$discovered" "$MOTORBRIDGE_LINK"
    fi
    RESOLVED_MOTORBRIDGE_ROOT="$discovered"
    echo "[INFO] 使用 motorbridge：$RESOLVED_MOTORBRIDGE_ROOT"
}

# ── 创建环境（Python + Pinocchio C++ 等原生依赖） ────────────────────────────
create_environment() {
    local CONDA_CMD=$1
    local ENV_NAME=$2
    local PY_VER=$3

    conda deactivate 2>/dev/null || true

    if $CONDA_CMD env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
        echo "Removing existing environment '$ENV_NAME'..."
        $CONDA_CMD env remove -n "$ENV_NAME" -y
    fi

    echo "Creating '$ENV_NAME' (Python $PY_VER) + 原生依赖 (Pinocchio C++ 3.x / Eigen / cmake / compiler)..."
    $CONDA_CMD create -y -n "$ENV_NAME" -c conda-forge "python=$PY_VER" "${CONDA_FORGE_DEPS[@]}" || {
        echo "[ERROR] 创建环境失败"; exit 1; }

    echo -e "\n[INFO] 已创建 $CONDA_CMD 环境 '$ENV_NAME'（Python $PY_VER）。\n"
    echo -e "\t1. 激活环境：       $CONDA_CMD activate $ENV_NAME"
    echo -e "\t2. 安装本包：       bash $SCRIPT_NAME --install"
    echo -e "\t3. 退出环境：       conda deactivate\n"
}

# ── 阶段 2：安装本包到当前已激活环境 ─────────────────────────────────────────
install_package() {
    if [[ -z "${CONDA_DEFAULT_ENV:-}" || "${CONDA_DEFAULT_ENV}" == "base" ]]; then
        echo "Error: 未激活目标环境。请先 conda/mamba activate <env_name> 再 --install。"
        exit 1
    fi
    echo "[INFO] 目标环境：$CONDA_DEFAULT_ENV  ($CONDA_PREFIX)"

    # Pinocchio 与 ROS/RoboStack 完全解耦：已有兼容前缀就复用，否则准备
    # 仓库内的私有 conda-forge C++ 前缀，不向当前环境安装 Pinocchio。
    ensure_pinocchio || exit 1
    ensure_motorbridge || exit 1

    # 1) 串口权限（Damiao @ /dev/ttyACM0）
    if id -nG "$USER" | grep -qw dialout; then
        echo "[INFO] 用户已在 dialout 组。"
    else
        echo "[INFO] 将用户加入 dialout 组（串口访问）..."
        sudo usermod -aG dialout "$USER" || true
        echo "[WARN] 需重新登录/重启后生效。"
    fi

    # 2) Rust
    if command -v cargo >/dev/null 2>&1; then
        echo "[INFO] Rust 已安装：$(cargo --version)"
    else
        echo "[INFO] 安装 Rust（rustup）..."
        curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
        # shellcheck disable=SC1091
        source "$HOME/.cargo/env"
    fi
    command -v cargo >/dev/null 2>&1 || { echo "[ERROR] cargo 不可用"; exit 1; }

    # 3) Python 构建/运行依赖
    # 剥掉 PYTHONPATH：若 shell source 了 ROS，/opt/ros 的包会经 PYTHONPATH 泄漏进本环境，
    # 让 pip 打出无关的依赖冲突告警（如 generate-parameter-library-py）。清掉即干净。
    echo "[INFO] 检查 maturin / pybind11 / numpy / pyyaml ..."
    local pip_deps=()
    env -u PYTHONPATH -u LD_LIBRARY_PATH python -c 'import maturin' 2>/dev/null \
        || pip_deps+=("maturin>=1.5,<2")
    env -u PYTHONPATH -u LD_LIBRARY_PATH python -c 'import pybind11' 2>/dev/null \
        || pip_deps+=("pybind11>=2.11")
    env -u PYTHONPATH -u LD_LIBRARY_PATH python -c 'import numpy' 2>/dev/null \
        || pip_deps+=("numpy>=1.24,<2")
    env -u PYTHONPATH -u LD_LIBRARY_PATH python -c 'import yaml' 2>/dev/null \
        || pip_deps+=("pyyaml>=6")
    if (( ${#pip_deps[@]} > 0 )); then
        echo "[INFO] 仅安装缺失项：${pip_deps[*]}"
        env -u PYTHONPATH -u LD_LIBRARY_PATH python -m pip install -q --no-deps "${pip_deps[@]}" \
            || { echo "[ERROR] pip 安装失败"; exit 1; }
    else
        echo "[INFO] Python 构建依赖已满足，不修改当前环境。"
    fi

    # 4) 构建并安装 wheel（CMake 编 _math + maturin 编 _native）
    echo "[INFO] 构建并安装（build.sh --wheel）..."
    env -u PYTHONPATH -u LD_LIBRARY_PATH \
        PINOCCHIO_PREFIX="$RESOLVED_PINOCCHIO_PREFIX" PY="$(which python)" \
        bash "$SCRIPT_DIR/build.sh" --wheel || { echo "[ERROR] build.sh 失败"; exit 1; }

    # 5) 自检
    echo "[INFO] 导入自检..."
    env -u PYTHONPATH -u LD_LIBRARY_PATH python - <<'PYEOF'
import rebotarm_control_rt as p
from rebotarm_control_rt.kinematics import load_robot_model
from rebotarm_control_rt.controllers import ArmEndPos  # noqa: F401
L = load_robot_model()
print("  package:", p.__version__, "| subpackages:", p.__all__)
print("  FK(neutral) shape:", L.fk(L.neutral())[2].shape)
print("[OK] rebotarm_control_rt 安装成功")
PYEOF
    echo -e "\n[INFO] 完成。跑测试：pip install pytest && bash ./run_tests.sh"
    echo "[INFO] 构建成功检查："
    echo "  conda activate $CONDA_DEFAULT_ENV"
    echo "  python -c \"import rebotarm_control_rt._math, rebotarm_control_rt._native; print('ok')\""
}

# ── 解析参数 ─────────────────────────────────────────────────────────────────
MODE="${1:-}"
ENV_NAME="${2:-rebot}"
PY_VER="${3:-${PYVER:-3.12}}"

case "$MODE" in
  --conda|--mamba)
    if [[ "$MODE" == "--mamba" ]]; then
        BASES=("$HOME/miniforge3" "$HOME/mambaforge")
        CONDA_CMD="mamba"
    else
        BASES=("$HOME/miniconda3" "$HOME/anaconda3" "$HOME/miniforge3")
        CONDA_CMD="conda"
    fi
    SOURCED=0
    for b in "${BASES[@]}"; do
        if [[ -f "$b/etc/profile.d/conda.sh" ]]; then
            # shellcheck disable=SC1091
            . "$b/etc/profile.d/conda.sh"
            [[ -f "$b/etc/profile.d/mamba.sh" ]] && . "$b/etc/profile.d/mamba.sh"
            SOURCED=1; break
        fi
    done
    if [[ "$SOURCED" -eq 0 ]]; then
        echo "未找到 conda/mamba。请安装 Miniforge3：https://github.com/conda-forge/miniforge"
        exit 1
    fi
    command -v "$CONDA_CMD" >/dev/null 2>&1 || CONDA_CMD="conda"
    create_environment "$CONDA_CMD" "$ENV_NAME" "$PY_VER"
    ;;
  --install)
    install_package
    ;;
  --prepare-pinocchio)
    ensure_pinocchio || exit 1
    echo "[OK] Pinocchio C++ 前缀已就绪：$RESOLVED_PINOCCHIO_PREFIX"
    ;;
  *)
    echo "用法："
    echo "  bash $SCRIPT_NAME --mamba [env_name] [py_version]   # 创建 mamba 环境（Miniforge）"
    echo "  bash $SCRIPT_NAME --conda [env_name] [py_version]   # 创建 conda 环境（Miniconda/Anaconda）"
    echo "  bash $SCRIPT_NAME --install                         # 在已激活环境中安装本包"
    echo "  bash $SCRIPT_NAME --prepare-pinocchio               # 仅准备/检查隔离的 Pinocchio C++"
    echo ""
    echo "示例："
    echo "  bash $SCRIPT_NAME --mamba rebot        # Python 3.12（默认）"
    echo "  bash $SCRIPT_NAME --mamba rebot 3.11    # 指定 Python 3.11"
    echo "  mamba activate rebot && bash $SCRIPT_NAME --install"
    exit 1
    ;;
esac
