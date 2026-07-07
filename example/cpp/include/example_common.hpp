#pragma once

#include "motorbridge/motorbridge.hpp"
#include "rebotarm/robot_model.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <csignal>
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace example {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr int kArmDof = 6;
constexpr int kAllDof = 7;
constexpr const char* kDefaultPort = "/dev/ttyACM0";
constexpr double kDefaultRateHz = 150.0;
constexpr double kEndLinkLoadScaleWithGripper = 0.7;

struct JointSpec {
  const char* name;
  uint16_t motor_id;
  uint16_t feedback_id;
  const char* model;
  const char* vendor;
  float mit_kp;
  float mit_kd;
  float vel_kp;
  float vel_ki;
  float pos_kp;
  float pos_ki;
  float vlim;
};

inline const std::vector<JointSpec>& b601_joints() {
  static const std::vector<JointSpec> joints = {
      {"shoulder_pan", 0x01, 0x11, "4340P", "damiao", 120.0f, 8.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.6179938f},
      {"shoulder_lift", 0x02, 0x12, "4340P", "damiao", 120.0f, 8.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.6179938f},
      {"elbow_flex", 0x03, 0x13, "4340P", "damiao", 120.0f, 8.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.6179938f},
      {"wrist_flex", 0x04, 0x14, "4310", "damiao", 18.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.6179938f},
      {"wrist_yaw", 0x05, 0x15, "4310", "damiao", 18.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.6179938f},
      {"wrist_roll", 0x06, 0x16, "4310", "damiao", 18.0f, 2.0f, 0.0f, 0.0f, 0.0f, 0.0f, 2.6179938f},
      {"gripper", 0x07, 0x17, "4310", "damiao", 8.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 5.2359877f},
  };
  return joints;
}

inline std::atomic_bool& stop_flag() {
  static std::atomic_bool flag{false};
  return flag;
}

inline void signal_handler(int) {
  stop_flag().store(true, std::memory_order_seq_cst);
}

inline void install_signal_handler() {
  stop_flag().store(false, std::memory_order_seq_cst);
#if defined(__unix__) || defined(__APPLE__)
  struct sigaction action {};
  action.sa_handler = signal_handler;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  sigaction(SIGINT, &action, nullptr);
  sigaction(SIGTERM, &action, nullptr);
#else
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
#endif
}

inline bool stop_requested() {
  return stop_flag().load(std::memory_order_seq_cst);
}

inline std::string repo_root() {
  const char* env = std::getenv("REBOTARM_CONTROL_RT_ROOT");
  if (env && env[0] != '\0') return env;
  return "/home/xense/rebot_lerobot/rebotarm_control_rt";
}

inline std::string default_urdf_path() {
  return repo_root() +
         "/urdf/reBot-DevArm_fixend_description/urdf/"
         "reBot-DevArm_fixend.urdf";
}

struct ExampleConfig {
  std::string channel = kDefaultPort;
  std::string urdf_path;
  std::vector<JointSpec> joints = b601_joints();
};

struct JointSpecStorage {
  std::vector<std::string> names;
  std::vector<std::string> models;
  std::vector<std::string> vendors;
};

inline JointSpecStorage& joint_spec_storage() {
  static JointSpecStorage storage;
  return storage;
}

inline std::string arg_value(int argc, char** argv, const std::string& name,
                             const std::string& default_value = "") {
  const std::string prefix = name + "=";
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == name && i + 1 < argc) return argv[i + 1];
    if (arg.rfind(prefix, 0) == 0) return arg.substr(prefix.size());
  }
  return default_value;
}

inline std::vector<std::string> arg_values(int argc, char** argv, const std::string& name) {
  std::vector<std::string> out;
  const std::string prefix = name + "=";
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == name && i + 1 < argc) {
      out.push_back(argv[++i]);
    } else if (arg.rfind(prefix, 0) == 0) {
      out.push_back(arg.substr(prefix.size()));
    }
  }
  return out;
}

inline bool has_flag(int argc, char** argv, const std::string& name) {
  for (int i = 1; i < argc; ++i) {
    if (argv[i] == name) return true;
  }
  return false;
}

inline std::string strip_yaml_scalar(std::string value) {
  const auto comment = value.find('#');
  if (comment != std::string::npos) value.resize(comment);
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) return "";
  const auto end = value.find_last_not_of(" \t\r\n");
  value = value.substr(begin, end - begin + 1);
  if (value.size() >= 2 &&
      ((value.front() == '"' && value.back() == '"') ||
       (value.front() == '\'' && value.back() == '\''))) {
    value = value.substr(1, value.size() - 2);
  }
  return value;
}

inline std::optional<std::pair<std::string, std::string>> yaml_key_value(const std::string& line) {
  const std::string stripped = strip_yaml_scalar(line);
  const auto pos = stripped.find(':');
  if (pos == std::string::npos) return std::nullopt;
  return std::make_pair(strip_yaml_scalar(stripped.substr(0, pos)),
                        strip_yaml_scalar(stripped.substr(pos + 1)));
}

inline uint16_t parse_yaml_id(const std::string& value) {
  const std::string v = strip_yaml_scalar(value);
  if (v.rfind("0x", 0) == 0 || v.rfind("0X", 0) == 0) {
    return static_cast<uint16_t>(std::stoul(v, nullptr, 16));
  }
  return static_cast<uint16_t>(std::stoul(v));
}

inline int indent_width(const std::string& line) {
  int n = 0;
  while (n < static_cast<int>(line.size()) && (line[n] == ' ' || line[n] == '\t')) ++n;
  return n;
}

inline std::string resolve_repo_path(const std::string& path, const std::string& base = "") {
  if (path.empty()) return path;
  std::filesystem::path p(path);
  if (p.is_absolute()) return p.string();
  std::vector<std::filesystem::path> candidates;
  if (!base.empty()) candidates.push_back(std::filesystem::path(base) / p);
  candidates.push_back(std::filesystem::path(repo_root()) / p);
  candidates.push_back(std::filesystem::current_path() / p);
  candidates.push_back(p);
  for (const auto& candidate : candidates) {
    if (std::filesystem::exists(candidate)) return candidate.string();
  }
  return candidates.front().string();
}

struct JointBuilder {
  std::string name;
  uint16_t motor_id = 0;
  uint16_t feedback_id = 0;
  std::string model = "4340P";
  std::string vendor = "damiao";
  float mit_kp = 0.0f;
  float mit_kd = 0.0f;
  float vel_kp = 0.0f;
  float vel_ki = 0.0f;
  float pos_kp = 0.0f;
  float pos_ki = 0.0f;
  float vlim = 2.0f;
  bool started = false;
};

inline void append_joint_spec(JointBuilder& builder, std::vector<JointSpec>& joints) {
  if (!builder.started) return;
  if (builder.name.empty() || builder.motor_id == 0) {
    throw std::runtime_error("invalid YAML joint entry");
  }
  auto& storage = joint_spec_storage();
  storage.names.push_back(builder.name);
  storage.models.push_back(builder.model);
  storage.vendors.push_back(builder.vendor);
  joints.push_back(JointSpec{
      storage.names.back().c_str(),
      builder.motor_id,
      builder.feedback_id,
      storage.models.back().c_str(),
      storage.vendors.back().c_str(),
      builder.mit_kp,
      builder.mit_kd,
      builder.vel_kp,
      builder.vel_ki,
      builder.pos_kp,
      builder.pos_ki,
      builder.vlim,
  });
  builder = JointBuilder{};
}

inline ExampleConfig load_example_config(int argc, char** argv) {
  ExampleConfig cfg;
  const std::string config_arg = arg_value(argc, argv, "--config", arg_value(argc, argv, "-c"));
  if (config_arg.empty()) return cfg;

  const std::string config_path = resolve_repo_path(config_arg);
  const std::filesystem::path base = std::filesystem::path(config_path).parent_path();
  std::ifstream input(config_path);
  if (!input) throw std::runtime_error("failed to open config: " + config_path);

  std::vector<JointSpec> joints;
  JointBuilder current;
  std::string section;
  std::string line;
  while (std::getline(input, line)) {
    const auto comment = line.find('#');
    const std::string raw = comment == std::string::npos ? line : line.substr(0, comment);
    if (strip_yaml_scalar(raw).empty()) continue;
    const int indent = indent_width(raw);
    const std::string stripped = strip_yaml_scalar(raw);

    if (indent == 0) {
      if (auto kv = yaml_key_value(stripped)) {
        if (kv->first == "channel") cfg.channel = kv->second;
        else if (kv->first == "urdf_path") cfg.urdf_path = resolve_repo_path(kv->second, base.string());
      }
      continue;
    }

    if (stripped.rfind("- name:", 0) == 0) {
      append_joint_spec(current, joints);
      current.started = true;
      current.name = strip_yaml_scalar(stripped.substr(std::string("- name:").size()));
      section.clear();
      continue;
    }
    if (stripped == "MIT:") {
      section = "MIT";
      continue;
    }
    if (stripped == "POS_VEL:") {
      section = "POS_VEL";
      continue;
    }
    const auto kv = yaml_key_value(stripped);
    if (!kv) continue;
    const auto& key = kv->first;
    const auto& value = kv->second;
    if (section == "MIT" && key == "kp") current.mit_kp = std::stof(value);
    else if (section == "MIT" && key == "kd") current.mit_kd = std::stof(value);
    else if (section == "POS_VEL" && key == "vel_kp") current.vel_kp = std::stof(value);
    else if (section == "POS_VEL" && key == "vel_ki") current.vel_ki = std::stof(value);
    else if (section == "POS_VEL" && key == "pos_kp") current.pos_kp = std::stof(value);
    else if (section == "POS_VEL" && key == "pos_ki") current.pos_ki = std::stof(value);
    else if (section == "POS_VEL" && key == "vlim") current.vlim = std::stof(value);
    else if (key == "motor_id") current.motor_id = parse_yaml_id(value);
    else if (key == "feedback_id") current.feedback_id = parse_yaml_id(value);
    else if (key == "model") current.model = value;
    else if (key == "vendor") current.vendor = value;
  }
  append_joint_spec(current, joints);
  if (!joints.empty()) cfg.joints = joints;
  return cfg;
}

inline ExampleConfig& active_config() {
  static ExampleConfig cfg;
  return cfg;
}

inline const std::vector<JointSpec>& active_joints() {
  return active_config().joints;
}

inline std::string parse_port(int argc, char** argv) {
  active_config() = load_example_config(argc, argv);
  const std::string short_port = arg_value(argc, argv, "-p");
  if (!short_port.empty()) return short_port;
  return arg_value(argc, argv, "--port", active_config().channel);
}

inline double arg_double(int argc, char** argv, const std::string& name, double default_value) {
  const std::string value = arg_value(argc, argv, name);
  if (value.empty()) return default_value;
  return std::stod(value);
}

inline int arg_int(int argc, char** argv, const std::string& name, int default_value) {
  const std::string value = arg_value(argc, argv, name);
  if (value.empty()) return default_value;
  return std::stoi(value);
}

inline double parse_rate(int argc, char** argv, double default_hz = kDefaultRateHz) {
  const double rate = arg_double(argc, argv, "--rate", default_hz);
  return std::max(1.0, rate);
}

inline bool parse_bool_arg(int argc, char** argv, const std::string& name, bool default_value) {
  const std::string value = arg_value(argc, argv, name);
  if (value.empty()) return default_value;
  return value == "1" || value == "true" || value == "True" || value == "yes" ||
         value == "on";
}

inline std::optional<std::string> prompt(const std::string& text) {
  std::cout << text << std::flush;
  std::string line;
  if (!std::getline(std::cin, line)) return std::nullopt;
  return line;
}

inline std::string urdf_arg(int argc, char** argv) {
  active_config() = load_example_config(argc, argv);
  const std::string explicit_urdf = arg_value(argc, argv, "--urdf");
  if (!explicit_urdf.empty()) return explicit_urdf;
  if (!active_config().urdf_path.empty()) return active_config().urdf_path;
  return default_urdf_path();
}

inline std::vector<double> parse_numbers(const std::string& line) {
  std::istringstream in(line);
  std::vector<double> values;
  double value = 0.0;
  while (in >> value) values.push_back(value);
  return values;
}

inline std::vector<double> parse_floats(const std::string& line) {
  return parse_numbers(line);
}

inline int parse_joint(const std::string& value) {
  try {
    const int index = std::stoi(value);
    if (0 <= index && index < kAllDof) return index;
    if (1 <= index && index <= kAllDof) return index - 1;
  } catch (...) {
  }
  if (value.rfind("joint", 0) == 0) {
    const int one_based = std::stoi(value.substr(5));
    if (1 <= one_based && one_based <= kAllDof) return one_based - 1;
  }
  const auto& joints = active_joints();
  for (int i = 0; i < static_cast<int>(joints.size()); ++i) {
    if (value == joints[i].name) return i;
  }
  throw std::runtime_error("unknown joint: " + value);
}

inline float deg_to_rad_f32(double value) {
  return static_cast<float>(value * kPi / 180.0);
}

inline double rad_to_deg(double value) {
  return value * 180.0 / kPi;
}

inline double rad_to_deg(float value) {
  return static_cast<double>(value) * 180.0 / kPi;
}

inline Eigen::VectorXd q_from_degrees(const std::vector<double>& values, int nq) {
  if (static_cast<int>(values.size()) < nq) {
    throw std::runtime_error("not enough joint values");
  }
  Eigen::VectorXd q(nq);
  for (int i = 0; i < nq; ++i) q[i] = values[i] * kPi / 180.0;
  return q;
}

inline Eigen::VectorXd q_degrees(const Eigen::VectorXd& q) {
  Eigen::VectorXd out(q.size());
  for (int i = 0; i < q.size(); ++i) out[i] = q[i] * 180.0 / kPi;
  return out;
}

inline Eigen::Matrix4d pose_from_xyz_rpy(double x, double y, double z, double roll,
                                         double pitch, double yaw) {
  const Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  const Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = (rz * ry * rx).toRotationMatrix();
  T.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
  return T;
}

inline void print_vector(const std::string& label, const Eigen::VectorXd& v,
                         int precision = 4) {
  std::cout << label << " [";
  for (int i = 0; i < v.size(); ++i) {
    if (i) std::cout << ", ";
    std::cout << std::fixed << std::setprecision(precision) << v[i];
  }
  std::cout << "]\n";
}

inline void print_pose(const rebotarm::RobotModel& model, const Eigen::VectorXd& q,
                       const std::string& frame = "") {
  const auto [pos, rot, T] = model.fk(q, frame);
  (void)T;
  const Eigen::Vector3d rpy = Eigen::Vector3d(
      std::atan2(rot(2, 1), rot(2, 2)),
      std::asin(std::clamp(-rot(2, 0), -1.0, 1.0)),
      std::atan2(rot(1, 0), rot(0, 0)));
  std::cout << "  ee position: [" << std::fixed << std::setprecision(4) << pos.x() << ", "
            << pos.y() << ", " << pos.z() << "] m\n";
  std::cout << "  ee rpy:      [" << std::fixed << std::setprecision(2)
            << rpy.x() * 180.0 / kPi << ", " << rpy.y() * 180.0 / kPi << ", "
            << rpy.z() * 180.0 / kPi << "] deg\n";
}

inline std::vector<float> default_kp() {
  std::vector<float> out;
  for (const auto& joint : active_joints()) out.push_back(joint.mit_kp);
  return out;
}

inline std::vector<float> default_kd() {
  std::vector<float> out;
  for (const auto& joint : active_joints()) out.push_back(joint.mit_kd);
  return out;
}

inline std::vector<float> default_vlim() {
  std::vector<float> out;
  for (const auto& joint : active_joints()) out.push_back(joint.vlim);
  return out;
}

inline void sleep_to_rate(std::chrono::steady_clock::time_point start, double rate_hz) {
  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz));
  const auto elapsed = std::chrono::steady_clock::now() - start;
  if (elapsed < period) {
    std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::microseconds>(period - elapsed));
  }
}

inline std::unique_ptr<motorbridge::Controller> open_controller(const std::string& port) {
  std::string lower = port;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  if (port.rfind("/dev/", 0) == 0 || lower.rfind("com", 0) == 0) {
    return std::make_unique<motorbridge::Controller>(
        motorbridge::Controller::from_dm_serial(port, 921600));
  }
  return std::make_unique<motorbridge::Controller>(port);
}

struct B601Arm {
  std::string port;
  std::unique_ptr<motorbridge::Controller> controller;
  std::vector<motorbridge::Motor> motors;
  std::vector<JointSpec> joints;

  B601Arm() = default;
  B601Arm(B601Arm&&) noexcept = default;
  B601Arm& operator=(B601Arm&&) noexcept = default;
  B601Arm(const B601Arm&) = delete;
  B601Arm& operator=(const B601Arm&) = delete;

  static B601Arm open(const std::string& port) {
    B601Arm arm;
    arm.port = port;
    arm.controller = open_controller(port);
    arm.joints = active_joints();
    arm.motors.reserve(arm.joints.size());
    if (!arm.joints.empty()) {
      const std::string vendor = arm.joints.front().vendor;
      for (const auto& joint : arm.joints) {
        if (vendor != joint.vendor) {
          throw std::runtime_error("C++ examples currently expect one vendor per config");
        }
      }
    }
    for (const auto& joint : arm.joints) {
      const std::string vendor = joint.vendor;
      if (vendor == "damiao") {
        arm.motors.emplace_back(
            arm.controller->add_damiao_motor(joint.motor_id, joint.feedback_id, joint.model));
      } else if (vendor == "robstride") {
        arm.motors.emplace_back(
            arm.controller->add_robstride_motor(joint.motor_id, joint.feedback_id, joint.model));
      } else {
        throw std::runtime_error("unsupported example vendor: " + vendor);
      }
    }
    for (auto& motor : arm.motors) {
      try {
        motor.robstride_set_active_report(true);
      } catch (...) {
      }
    }
    return arm;
  }

  void enable() {
    for (auto& motor : motors) {
      try {
        motor.clear_error();
      } catch (const std::exception& e) {
        std::cerr << "warning: clear_error failed: " << e.what() << "\n";
      }
    }
    controller->enable_all();
  }

  void disable() {
    controller->disable_all();
  }

  void close() {
    try {
      controller->disable_all();
    } catch (...) {
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    try {
      controller->shutdown();
    } catch (...) {
    }
    try {
      controller->close_bus();
    } catch (...) {
    }
  }

  void ensure_all_mode(motorbridge::Mode mode, uint32_t timeout_ms = 300) {
    for (int i = 0; i < static_cast<int>(motors.size()); ++i) {
      try {
        motors[i].ensure_mode(mode, timeout_ms);
      } catch (const std::exception& e) {
        std::cerr << "warning: " << joints[i].name << " mode switch failed: "
                  << e.what() << "\n";
      }
    }
  }

  void ensure_arm_mode(motorbridge::Mode mode, uint32_t timeout_ms = 300) {
    for (int i = 0; i < std::min(kArmDof, static_cast<int>(motors.size())); ++i) {
      try {
        motors[i].ensure_mode(mode, timeout_ms);
      } catch (const std::exception& e) {
        std::cerr << "warning: " << joints[i].name << " mode switch failed: "
                  << e.what() << "\n";
      }
    }
  }

  void request_feedback() {
    for (auto& motor : motors) {
      try {
        motor.request_feedback();
      } catch (...) {
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  std::vector<std::optional<motorbridge::State>> states() {
    request_feedback();
    std::vector<std::optional<motorbridge::State>> out;
    out.reserve(motors.size());
    for (auto& motor : motors) {
      try {
        out.push_back(motor.get_state());
      } catch (...) {
        out.push_back(std::nullopt);
      }
    }
    return out;
  }

  std::vector<float> positions_or_zero() {
    std::vector<float> out;
    for (const auto& state : states()) {
      out.push_back(state ? state->pos : 0.0f);
    }
    while (static_cast<int>(out.size()) < kAllDof) out.push_back(0.0f);
    return out;
  }

  Eigen::VectorXd arm_positions_or_zero() {
    const auto q_all = positions_or_zero();
    Eigen::VectorXd q(kArmDof);
    for (int i = 0; i < kArmDof; ++i) q[i] = q_all[i];
    return q;
  }

  void print_state() {
    const auto all_states = states();
    for (int i = 0; i < static_cast<int>(joints.size()); ++i) {
      std::cout << std::left << std::setw(14) << joints[i].name;
      if (i < static_cast<int>(all_states.size()) && all_states[i]) {
        const auto& s = *all_states[i];
        std::cout << " pos=" << std::right << std::setw(8) << std::fixed
                  << std::setprecision(2) << rad_to_deg(s.pos) << " deg"
                  << " vel=" << std::setw(8) << rad_to_deg(s.vel) << " deg/s"
                  << " torque=" << std::setw(8) << std::setprecision(3) << s.torq
                  << " status=" << static_cast<int>(s.status_code) << "\n";
      } else {
        std::cout << " no feedback\n";
      }
    }
  }

  void send_mit_all(const std::vector<float>& pos, const std::vector<float>& vel,
                    const std::vector<float>& kp, const std::vector<float>& kd,
                    const std::vector<float>& tau) {
    for (int i = 0; i < static_cast<int>(motors.size()); ++i) {
      motors[i].send_mit(pos.size() > i ? pos[i] : 0.0f, vel.size() > i ? vel[i] : 0.0f,
                         kp.size() > i ? kp[i] : joints[i].mit_kp,
                         kd.size() > i ? kd[i] : joints[i].mit_kd,
                         tau.size() > i ? tau[i] : 0.0f);
    }
  }

  void send_pos_vel_all(const std::vector<float>& pos, const std::vector<float>& vlim) {
    for (int i = 0; i < static_cast<int>(motors.size()); ++i) {
      motors[i].send_pos_vel(pos.size() > i ? pos[i] : 0.0f,
                             vlim.size() > i ? vlim[i] : joints[i].vlim);
    }
  }
};

inline void move_pos_vel_path(B601Arm& arm, const std::vector<float>& start,
                              const std::vector<float>& end, double duration_s,
                              double rate_hz) {
  const int steps = std::max(1, static_cast<int>(std::ceil(std::max(0.02, duration_s) *
                                                           std::max(1.0, rate_hz))));
  const auto vlim = default_vlim();
  for (int step = 1; step <= steps && !stop_requested(); ++step) {
    const auto tick = std::chrono::steady_clock::now();
    const float alpha = static_cast<float>(step) / static_cast<float>(steps);
    std::vector<float> target(kAllDof, 0.0f);
    for (int i = 0; i < kAllDof; ++i) {
      const float s = start.size() > i ? start[i] : 0.0f;
      const float e = end.size() > i ? end[i] : s;
      target[i] = s + (e - s) * alpha;
    }
    arm.send_pos_vel_all(target, vlim);
    sleep_to_rate(tick, rate_hz);
  }
}

inline std::string format_float(double value) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(10) << value;
  std::string text = out.str();
  while (text.find('.') != std::string::npos && text.back() == '0') text.pop_back();
  if (!text.empty() && text.back() == '.') text.push_back('0');
  return text;
}

inline std::optional<size_t> find_xml_element(const std::string& source,
                                              const std::string& element) {
  const std::string needle = "<" + element;
  size_t offset = 0;
  while (true) {
    const size_t pos = source.find(needle, offset);
    if (pos == std::string::npos) return std::nullopt;
    const size_t next_pos = pos + needle.size();
    if (next_pos >= source.size() || std::isspace(static_cast<unsigned char>(source[next_pos])) ||
        source[next_pos] == '/' || source[next_pos] == '>') {
      return pos;
    }
    offset = next_pos;
  }
}

inline std::string scale_attr_once(const std::string& source, const std::string& element,
                                   const std::string& attr, double scale) {
  const auto elem_start_opt = find_xml_element(source, element);
  if (!elem_start_opt) throw std::runtime_error("URDF inertial block is missing <" + element + ">");
  const size_t elem_start = *elem_start_opt;
  const size_t elem_end = source.find('>', elem_start);
  if (elem_end == std::string::npos) throw std::runtime_error("URDF <" + element + "> tag is malformed");
  const std::string tag = source.substr(elem_start, elem_end - elem_start);
  size_t attr_offset = tag.find(attr + "=\"");
  char quote = '"';
  if (attr_offset == std::string::npos) {
    attr_offset = tag.find(attr + "='");
    quote = '\'';
  }
  if (attr_offset == std::string::npos) {
    throw std::runtime_error("URDF <" + element + "> tag is missing " + attr);
  }
  const size_t value_start = elem_start + attr_offset + attr.size() + 2;
  const size_t value_end = source.find(quote, value_start);
  if (value_end == std::string::npos) {
    throw std::runtime_error("URDF <" + element + "> " + attr + " quote is not closed");
  }
  const double value = std::stod(source.substr(value_start, value_end - value_start));
  std::string out = source;
  out.replace(value_start, value_end - value_start, format_float(value * scale));
  return out;
}

inline std::string scale_inertial_block(const std::string& block, double scale) {
  std::string out = scale_attr_once(block, "mass", "value", scale);
  for (const std::string attr : {"ixx", "ixy", "ixz", "iyy", "iyz", "izz"}) {
    out = scale_attr_once(out, "inertia", attr, scale);
  }
  return out;
}

inline std::string scale_end_link_inertial(const std::string& xml, double scale) {
  size_t end_link_pos = xml.find("name=\"end_link\"");
  if (end_link_pos == std::string::npos) end_link_pos = xml.find("name='end_link'");
  if (end_link_pos == std::string::npos) {
    throw std::runtime_error("URDF does not contain link name=\"end_link\"");
  }
  const size_t inertial_rel = xml.find("<inertial", end_link_pos);
  if (inertial_rel == std::string::npos) {
    throw std::runtime_error("URDF end_link does not contain an inertial block");
  }
  const size_t inertial_start = inertial_rel;
  const size_t inertial_open_end = xml.find('>', inertial_start);
  if (inertial_open_end == std::string::npos) {
    throw std::runtime_error("URDF end_link inertial block is malformed");
  }
  const size_t close_start = xml.find("</inertial>", inertial_open_end);
  if (close_start == std::string::npos) {
    throw std::runtime_error("URDF end_link inertial block is not closed");
  }
  const size_t inertial_end = close_start + std::string("</inertial>").size();
  std::string out = xml.substr(0, inertial_start);
  if (scale > 0.0) {
    out += scale_inertial_block(xml.substr(inertial_start, inertial_end - inertial_start), scale);
  }
  out += xml.substr(inertial_end);
  return out;
}

inline bool has_end_link_inertial(const std::string& xml) {
  size_t end_link_pos = xml.find("name=\"end_link\"");
  if (end_link_pos == std::string::npos) end_link_pos = xml.find("name='end_link'");
  if (end_link_pos == std::string::npos) return false;
  return xml.find("<inertial", end_link_pos) != std::string::npos;
}

class TemporaryUrdf {
 public:
  explicit TemporaryUrdf(std::string path) : path_(std::move(path)) {}
  TemporaryUrdf(TemporaryUrdf&&) noexcept = default;
  TemporaryUrdf& operator=(TemporaryUrdf&&) noexcept = default;
  TemporaryUrdf(const TemporaryUrdf&) = delete;
  TemporaryUrdf& operator=(const TemporaryUrdf&) = delete;
  ~TemporaryUrdf() {
    if (!path_.empty()) {
      std::error_code ec;
      std::filesystem::remove(path_, ec);
    }
  }
  const std::string& path() const { return path_; }

 private:
  std::string path_;
};

struct GravityUrdf {
  std::string path;
  std::unique_ptr<TemporaryUrdf> temp;
  double end_link_scale;
};

inline GravityUrdf gravity_urdf_for_gripper(int argc, char** argv, bool use_gripper) {
  active_config() = load_example_config(argc, argv);
  const std::string explicit_urdf = arg_value(argc, argv, "--urdf");
  const bool configured_urdf = !active_config().urdf_path.empty();
  const std::string base_urdf = !explicit_urdf.empty()
                                    ? explicit_urdf
                                    : (configured_urdf ? active_config().urdf_path
                                                       : default_urdf_path());
  const std::string explicit_scale = arg_value(argc, argv, "--end-link-load-scale");
  const double scale = !explicit_scale.empty()
                           ? std::stod(explicit_scale)
                           : (!use_gripper ? 0.0
                                           : (!explicit_urdf.empty() || configured_urdf
                                                  ? 1.0
                                                  : kEndLinkLoadScaleWithGripper));
  std::ifstream input(base_urdf);
  if (!input) throw std::runtime_error("failed to open URDF: " + base_urdf);
  std::ostringstream buffer;
  buffer << input.rdbuf();
  const std::string xml = buffer.str();
  if (std::abs(scale - 1.0) <= std::numeric_limits<double>::epsilon() ||
      !has_end_link_inertial(xml)) {
    return GravityUrdf{base_urdf, nullptr, scale};
  }
  const std::string modified = scale_end_link_inertial(xml, scale);
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();
  const std::string temp_path = (std::filesystem::temp_directory_path() /
                                 ("rebotarm_control_rt_end_link_" + std::to_string(now) + ".urdf"))
                                    .string();
  std::ofstream output(temp_path);
  output << modified;
  output.close();
  return GravityUrdf{temp_path, std::make_unique<TemporaryUrdf>(temp_path), scale};
}

inline int run_single_motor_console(int argc, char** argv) {
  if (has_flag(argc, argv, "--help") || has_flag(argc, argv, "-h")) {
    std::cout << "Usage: ./0x01damiao_test --port /dev/ttyACM0 --joint 0\n";
    return 0;
  }

  const std::string joint_arg = arg_value(argc, argv, "--joint",
                                          arg_value(argc, argv, "-j", "0"));
  active_config() = load_example_config(argc, argv);
  const int joint_idx = parse_joint(joint_arg);
  auto arm = B601Arm::open(parse_port(argc, argv));
  if (joint_idx < 0 || joint_idx >= static_cast<int>(arm.joints.size())) {
    throw std::runtime_error("joint index out of range");
  }
  const auto& joint = arm.joints[joint_idx];

  std::cout << "connected: B601 on " << arm.port << "\n";
  std::cout << "joint: " << joint_idx << " (" << joint.name << ")\n";
  std::cout << "commands: enable / disable / set_zero / mode / mit / posvel / vel / forcepos / state / q\n";
  std::cout << "examples: mit 10 0 20 2 0 | posvel 10 1.0 | vel 0.2 | forcepos -120 3.0 0.05\n";

  arm.enable();
  arm.ensure_all_mode(motorbridge::Mode::MIT);

  while (true) {
    const auto line_opt = prompt("> ");
    if (!line_opt) break;
    const std::string line = *line_opt;
    if (line.empty()) continue;
    if (line == "q" || line == "quit" || line == "exit") break;
    if (line == "enable") {
      arm.enable();
      std::cout << "enabled\n";
      continue;
    }
    if (line == "disable") {
      arm.disable();
      std::cout << "disabled\n";
      continue;
    }
    if (line == "state") {
      arm.print_state();
      continue;
    }
    if (line == "set_zero") {
      std::cout << "set zero requires disabled motor. Type YES to continue.\n";
      if (prompt("confirm> ").value_or("") == "YES") {
        arm.motors[joint_idx].disable();
        arm.motors[joint_idx].set_zero_position();
        std::cout << "zero set for " << joint.name << "\n";
      }
      continue;
    }

    std::istringstream in(line);
    std::string cmd;
    in >> cmd;
    try {
      if (cmd == "mode") {
        std::string mode;
        in >> mode;
        if (mode == "mit") arm.motors[joint_idx].ensure_mode(motorbridge::Mode::MIT, 300);
        else if (mode == "posvel" || mode == "pos_vel")
          arm.motors[joint_idx].ensure_mode(motorbridge::Mode::POS_VEL, 300);
        else if (mode == "vel") arm.motors[joint_idx].ensure_mode(motorbridge::Mode::VEL, 300);
        else if (mode == "forcepos" || mode == "force_pos")
          arm.motors[joint_idx].ensure_mode(motorbridge::Mode::FORCE_POS, 300);
        else {
          std::cout << "unknown mode: " << mode << "\n";
          continue;
        }
        std::cout << "mode set for " << joint.name << "\n";
      } else if (cmd == "mit") {
        double pos_deg = 0.0;
        in >> pos_deg;
        float vel = 0.0f, kp = joint.mit_kp, kd = joint.mit_kd, tau = 0.0f;
        in >> vel;
        if (!in.fail()) in >> kp;
        if (!in.fail()) in >> kd;
        if (!in.fail()) in >> tau;
        arm.motors[joint_idx].send_mit(deg_to_rad_f32(pos_deg), vel, kp, kd, tau);
        std::cout << "sent MIT target " << pos_deg << " deg\n";
      } else if (cmd == "posvel" || cmd == "pos_vel") {
        double pos_deg = 0.0;
        float vlim = joint.vlim;
        in >> pos_deg;
        if (!(in >> vlim)) vlim = joint.vlim;
        arm.motors[joint_idx].send_pos_vel(deg_to_rad_f32(pos_deg), vlim);
        std::cout << "sent POS_VEL target " << pos_deg << " deg, vlim=" << vlim << "\n";
      } else if (cmd == "vel") {
        float vel = 0.0f;
        in >> vel;
        arm.motors[joint_idx].send_vel(vel);
        std::cout << "sent velocity " << vel << " rad/s\n";
      } else if (cmd == "forcepos" || cmd == "force_pos") {
        double pos_deg = 0.0;
        float vlim = joint.vlim;
        float ratio = 0.05f;
        in >> pos_deg;
        if (!(in >> vlim)) vlim = joint.vlim;
        if (!(in >> ratio)) ratio = 0.05f;
        arm.motors[joint_idx].send_force_pos(deg_to_rad_f32(pos_deg), vlim, ratio);
        std::cout << "sent FORCE_POS target " << pos_deg << " deg, vlim=" << vlim
                  << ", ratio=" << ratio << "\n";
      } else {
        std::cout << "unknown command\n";
      }
    } catch (const std::exception& e) {
      std::cout << "command failed: " << e.what() << "\n";
    }
  }

  arm.close();
  return 0;
}

}  // namespace example
