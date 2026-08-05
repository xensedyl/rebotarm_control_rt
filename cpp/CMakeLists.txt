cmake_minimum_required(VERSION 3.16)
project(rebotarm_math LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# ---------------------------------------------------------------------------
# 自动探测 Pinocchio C++ 前缀（当前环境 / 项目私有 / 系统；不依赖 ROS）。
# 优先级：-DPINOCCHIO_PREFIX > $PINOCCHIO_PREFIX > $CONDA_PREFIX > .deps > /usr/local > /usr
# 仅链接 C++ 核心库，绕开 pinocchioConfig.cmake（其会拖入 eigenpy/Boost.Python）。
# ---------------------------------------------------------------------------
function(rebotarm_pinocchio_prefix_compatible prefix result_var)
  set(is_compatible FALSE)
  set(config_header "${prefix}/include/pinocchio/config.hpp")
  if(EXISTS "${prefix}/include/pinocchio/spatial/se3-base.hpp" AND EXISTS "${config_header}")
    file(STRINGS "${config_header}" version_line
      REGEX "^[ \\t]*#[ \\t]*define[ \\t]+PINOCCHIO_VERSION[ \\t]+\"")
    string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" candidate_version "${version_line}")
    if(candidate_version
       AND NOT candidate_version VERSION_LESS "3.2.0"
       AND candidate_version VERSION_LESS "4.0.0")
      set(is_compatible TRUE)
    endif()
  endif()
  set(${result_var} ${is_compatible} PARENT_SCOPE)
endfunction()

if(NOT PINOCCHIO_PREFIX)
  if(DEFINED ENV{PINOCCHIO_PREFIX})
    set(PINOCCHIO_PREFIX "$ENV{PINOCCHIO_PREFIX}")
  endif()
endif()
if(NOT PINOCCHIO_PREFIX AND DEFINED ENV{CONDA_PREFIX})
  rebotarm_pinocchio_prefix_compatible("$ENV{CONDA_PREFIX}" conda_pinocchio_compatible)
  if(conda_pinocchio_compatible)
    set(PINOCCHIO_PREFIX "$ENV{CONDA_PREFIX}")
  endif()
endif()
if(NOT PINOCCHIO_PREFIX)
  set(private_pinocchio_prefix "${CMAKE_CURRENT_SOURCE_DIR}/../.deps/pinocchio")
  rebotarm_pinocchio_prefix_compatible("${private_pinocchio_prefix}" private_pinocchio_compatible)
  if(private_pinocchio_compatible)
    set(PINOCCHIO_PREFIX "${private_pinocchio_prefix}")
  endif()
endif()
foreach(system_pinocchio_prefix IN ITEMS /usr/local /usr)
  if(NOT PINOCCHIO_PREFIX)
    rebotarm_pinocchio_prefix_compatible(
      "${system_pinocchio_prefix}" system_pinocchio_compatible)
    if(system_pinocchio_compatible)
      set(PINOCCHIO_PREFIX "${system_pinocchio_prefix}")
    endif()
  endif()
endforeach()
if(NOT PINOCCHIO_PREFIX OR NOT EXISTS "${PINOCCHIO_PREFIX}/include/pinocchio")
  message(FATAL_ERROR
    "未找到 Pinocchio C++。请运行 `bash setup_env.sh --prepare-pinocchio`，"
    "或显式指定 -DPINOCCHIO_PREFIX=<prefix>。本项目不要求 ROS。")
endif()
set(PINOCCHIO_CONFIG_HEADER "${PINOCCHIO_PREFIX}/include/pinocchio/config.hpp")
if(NOT EXISTS "${PINOCCHIO_CONFIG_HEADER}")
  message(FATAL_ERROR "Pinocchio 前缀缺少 config.hpp：${PINOCCHIO_PREFIX}")
endif()
file(STRINGS "${PINOCCHIO_CONFIG_HEADER}" PINOCCHIO_VERSION_LINE
  REGEX "^[ \\t]*#[ \\t]*define[ \\t]+PINOCCHIO_VERSION[ \\t]+\"")
string(REGEX MATCH "[0-9]+\\.[0-9]+\\.[0-9]+" PINOCCHIO_DETECTED_VERSION
  "${PINOCCHIO_VERSION_LINE}")
if(NOT PINOCCHIO_DETECTED_VERSION
   OR PINOCCHIO_DETECTED_VERSION VERSION_LESS "3.2.0"
   OR NOT PINOCCHIO_DETECTED_VERSION VERSION_LESS "4.0.0")
  message(FATAL_ERROR
    "需要 Pinocchio >=3.2,<4，检测到 ${PINOCCHIO_DETECTED_VERSION}：${PINOCCHIO_PREFIX}")
endif()
message(STATUS "Pinocchio prefix: ${PINOCCHIO_PREFIX}")
message(STATUS "Pinocchio version: ${PINOCCHIO_DETECTED_VERSION}")
list(APPEND CMAKE_PREFIX_PATH "${PINOCCHIO_PREFIX}")

# pybind11（经 -Dpybind11_DIR=$(python -m pybind11 --cmakedir)）。
find_package(pybind11 REQUIRED)

# Eigen：优先 Eigen3Config，找不到则在前缀/系统路径搜索头文件。
find_package(Eigen3 QUIET CONFIG)
if(NOT Eigen3_FOUND)
  find_path(EIGEN3_INCLUDE_DIR Eigen/Dense
    PATHS "${PINOCCHIO_PREFIX}/include/eigen3" /usr/include/eigen3 /usr/local/include/eigen3
    REQUIRED)
  message(STATUS "Eigen include: ${EIGEN3_INCLUDE_DIR}")
endif()

# Pinocchio 库：自动适配 lib 与 lib/x86_64-linux-gnu。
find_library(PINOCCHIO_DEFAULT_LIB NAMES pinocchio_default pinocchio
  HINTS "${PINOCCHIO_PREFIX}/lib" "${PINOCCHIO_PREFIX}/lib/x86_64-linux-gnu" REQUIRED)
find_library(PINOCCHIO_PARSERS_LIB NAMES pinocchio_parsers
  HINTS "${PINOCCHIO_PREFIX}/lib" "${PINOCCHIO_PREFIX}/lib/x86_64-linux-gnu")
get_filename_component(PINO_LIBDIR "${PINOCCHIO_DEFAULT_LIB}" DIRECTORY)
message(STATUS "Pinocchio lib dir: ${PINO_LIBDIR}")
find_library(URDFDOM_MODEL_LIB NAMES urdfdom_model
  HINTS "${PINOCCHIO_PREFIX}/lib" "${PINOCCHIO_PREFIX}/lib/x86_64-linux-gnu"
        /usr/local/lib /usr/lib/x86_64-linux-gnu)
find_library(CONSOLE_BRIDGE_LIB NAMES console_bridge
  HINTS "${PINOCCHIO_PREFIX}/lib" "${PINOCCHIO_PREFIX}/lib/x86_64-linux-gnu"
        /usr/local/lib /usr/lib/x86_64-linux-gnu)
find_library(TINYXML2_LIB NAMES tinyxml2
  HINTS "${PINOCCHIO_PREFIX}/lib" "${PINOCCHIO_PREFIX}/lib/x86_64-linux-gnu"
        /usr/local/lib /usr/lib/x86_64-linux-gnu)

set(REBOTARM_MATH_CORE_SOURCES
  src/robot_model.cpp
  src/dynamics.cpp
  src/identification.cpp
  src/trajectory.cpp
  src/math_c_api.cpp
)

add_library(rebotarm_math SHARED
  ${REBOTARM_MATH_CORE_SOURCES}
)
target_include_directories(rebotarm_math PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PINOCCHIO_PREFIX}/include
)
target_compile_definitions(rebotarm_math PUBLIC PINOCCHIO_WITH_URDFDOM)
target_link_libraries(rebotarm_math PUBLIC ${PINOCCHIO_DEFAULT_LIB})
if(PINOCCHIO_PARSERS_LIB)
  target_link_libraries(rebotarm_math PUBLIC ${PINOCCHIO_PARSERS_LIB})
endif()
if(URDFDOM_MODEL_LIB)
  target_link_libraries(rebotarm_math PUBLIC ${URDFDOM_MODEL_LIB})
endif()
if(CONSOLE_BRIDGE_LIB)
  target_link_libraries(rebotarm_math PUBLIC ${CONSOLE_BRIDGE_LIB})
endif()
if(TINYXML2_LIB)
  target_link_libraries(rebotarm_math PUBLIC ${TINYXML2_LIB})
endif()
if(Eigen3_FOUND)
  target_link_libraries(rebotarm_math PUBLIC Eigen3::Eigen)
else()
  target_include_directories(rebotarm_math PUBLIC ${EIGEN3_INCLUDE_DIR})
endif()
set_target_properties(rebotarm_math PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../python/rebotarm_control_rt
  OUTPUT_NAME "rebotarm_math"
  BUILD_RPATH "${PINO_LIBDIR}"
  INSTALL_RPATH "${PINO_LIBDIR}")

pybind11_add_module(_math
  src/bindings.cpp
  src/arm_endpos.cpp
)
target_include_directories(_math PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PINOCCHIO_PREFIX}/include
)
target_compile_definitions(_math PRIVATE PINOCCHIO_WITH_URDFDOM)
target_link_libraries(_math PRIVATE rebotarm_math)
if(Eigen3_FOUND)
  target_link_libraries(_math PRIVATE Eigen3::Eigen)
else()
  target_include_directories(_math PRIVATE ${EIGEN3_INCLUDE_DIR})
endif()

# 直接把 .so 输出到 Python 包目录（被 maturin 一并打包）；RPATH 指向 Pinocchio 库目录。
set_target_properties(_math PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../python/rebotarm_control_rt
  OUTPUT_NAME "_math"
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../python/rebotarm_control_rt;${PINO_LIBDIR}"
  INSTALL_RPATH "$ORIGIN;${PINO_LIBDIR}")
