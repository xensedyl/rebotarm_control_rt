#!/usr/bin/env bash
# 构建 rebotarm_control_rt：先用 CMake 编译 C++ 数学层 _math（链接 Pinocchio C++），
# 再用 maturin 把 Rust actuator _native + _math.so + Python 打进同一 wheel。
#
# 用法：
#   PY=/path/to/python ./build.sh            # 只构建 C++ _math（开发期）
#   PY=/path/to/python ./build.sh --wheel    # 额外 maturin 打 wheel 并安装
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PY="${PY:-python}"

echo "== Python: $(command -v "$PY") =="
PYBIND11_DIR="$("$PY" -m pybind11 --cmakedir)"
echo "== pybind11 cmakedir: $PYBIND11_DIR =="

# Pinocchio C++ 前缀由 CMake 自动探测（conda-forge / 系统）；如需可显式：
#   PINOCCHIO_PREFIX=/path/to/prefix ./build.sh
CMAKE_PINO_ARG=()
if [[ -n "${PINOCCHIO_PREFIX:-}" ]]; then
  CMAKE_PINO_ARG=(-DPINOCCHIO_PREFIX="$PINOCCHIO_PREFIX")
fi

# 清理上一次的 CMake 缓存：换 conda 环境后，find_library/find_package 的缓存会指向旧
# 环境的 Pinocchio/Eigen，导致链接与 RPATH 指错环境。每次干净配置以保证正确。
rm -rf "$HERE/cpp/build"

cmake -S "$HERE/cpp" -B "$HERE/cpp/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -Dpybind11_DIR="$PYBIND11_DIR" \
  -DPYBIND11_FINDPYTHON=ON \
  -DPython_EXECUTABLE="$PY" \
  -DPython3_EXECUTABLE="$PY" \
  -DPYTHON_EXECUTABLE="$PY" \
  "${CMAKE_PINO_ARG[@]}"
cmake --build "$HERE/cpp/build" -j"$(nproc)"
echo "== built _math.so into python/rebotarm_control_rt/ =="

if [[ "${1:-}" == "--wheel" ]]; then
  # 清掉旧 wheel：dist/ 可能残留其它 Python ABI（如 cp312）的 wheel，
  # 否则 `pip install dist/*.whl` 会匹配到不兼容的那个而报错。
  rm -f "$HERE"/dist/*.whl
  ( cd "$HERE" && "$PY" -m maturin build -i "$PY" -o dist )
  "$PY" -m pip install --force-reinstall --no-deps "$HERE"/dist/*.whl
  # 同时把 _native.so 释放进源码树，使 PYTHONPATH=$HERE/python 可直接运行（免装 wheel）。
  "$PY" - "$HERE" <<'PYEOF'
import glob, os, sys, zipfile
here = sys.argv[1]
whl = sorted(glob.glob(os.path.join(here, "dist", "*.whl")))[-1]
with zipfile.ZipFile(whl) as z:
    for n in z.namelist():
        if "_native" in n and n.endswith(".so"):
            z.extract(n, os.path.join(here, "python"))
            print("extracted", n, "-> python/")
PYEOF
  echo "== wheel built+installed; _native.so also placed in source tree =="
fi
