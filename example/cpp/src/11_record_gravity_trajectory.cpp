#include "example_common.hpp"
#include "rebotarm/dynamics.hpp"

#include <atomic>
#include <condition_variable>
#include <exception>
#include <iostream>
#include <mutex>
#include <thread>

namespace {

struct TrajectoryRow {
  double t = 0.0;
  std::vector<double> q;
  std::vector<double> dq;
  std::vector<double> tau;
};

void ensure_parent_dir(const std::string& path) {
  const auto parent = std::filesystem::path(path).parent_path();
  if (!parent.empty()) std::filesystem::create_directories(parent);
}

void save_trajectory_csv(const std::string& path, const std::vector<TrajectoryRow>& rows, int dof) {
  ensure_parent_dir(path);
  std::ofstream out(path);
  if (!out) throw std::runtime_error("failed to write " + path);
  out << "time";
  for (const char* prefix : {"q", "dq"}) {
    for (int i = 1; i <= dof; ++i) out << "," << prefix << i;
  }
  for (int i = 1; i <= dof; ++i) out << ",tau" << i;
  out << "\n";
  out << std::setprecision(12);
  for (const auto& row : rows) {
    out << row.t;
    for (int i = 0; i < dof; ++i) out << "," << row.q[i];
    for (int i = 0; i < dof; ++i) out << "," << row.dq[i];
    for (int i = 0; i < dof; ++i) out << "," << row.tau[i];
    out << "\n";
  }
}

void print_summary(const std::vector<TrajectoryRow>& rows, int dof) {
  if (rows.empty()) return;
  std::vector<double> q_min(dof, std::numeric_limits<double>::infinity());
  std::vector<double> q_max(dof, -std::numeric_limits<double>::infinity());
  std::vector<double> dq_abs(dof, 0.0);
  for (const auto& row : rows) {
    for (int j = 0; j < dof; ++j) {
      q_min[j] = std::min(q_min[j], row.q[j]);
      q_max[j] = std::max(q_max[j], row.q[j]);
      dq_abs[j] = std::max(dq_abs[j], std::abs(row.dq[j]));
    }
  }
  const double dt = rows.size() >= 2 ? (rows.back().t - rows.front().t) / (rows.size() - 1) : 0.0;
  std::cout << "samples=" << rows.size() << " dof=" << dof
            << " duration=" << rows.back().t << "s dt~=" << dt << "s\n";
  std::cout << "q min/max [deg]:\n";
  for (int j = 0; j < dof; ++j) {
    std::cout << "  joint" << j + 1 << ": " << example::rad_to_deg(q_min[j])
              << " .. " << example::rad_to_deg(q_max[j]) << "\n";
  }
  std::cout << "max |dq| [deg/s]: [";
  for (int j = 0; j < dof; ++j) {
    if (j) std::cout << ", ";
    std::cout << example::rad_to_deg(dq_abs[j]);
  }
  std::cout << "]\n";
}

void release_mit_torque_hold(example::B601Arm& arm, int frames = 10, double dt_s = 0.02) {
  const auto q = arm.positions_or_zero();
  std::vector<float> zeros(example::kAllDof, 0.0f);
  for (int i = 0; i < frames; ++i) {
    arm.send_mit_all(q, zeros, zeros, zeros, zeros);
    std::this_thread::sleep_for(std::chrono::duration<double>(dt_s));
  }
}

bool str_to_bool(int argc, char** argv, const std::string& name, bool default_value) {
  return example::parse_bool_arg(argc, argv, name, default_value);
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (example::has_flag(argc, argv, "--help") || example::has_flag(argc, argv, "-h")) {
      std::cout << "Usage: ./11_record_gravity_trajectory --output calibration/recorded_trajectory.csv "
                   "--port /dev/ttyACM0 --rate 200 --sample-rate 100 --kd 1.0 "
                   "--gravity-scale 1.0 [--urdf robot.urdf] [--use_gripper=true]\n";
      return 0;
    }

    const std::string output = example::arg_value(argc, argv, "--output", "calibration/recorded_trajectory.csv");
    const double rate = example::parse_rate(argc, argv, 200.0);
    const double sample_rate = example::arg_double(argc, argv, "--sample-rate", 100.0);
    const double max_duration = example::arg_double(argc, argv, "--max-duration-s", 180.0);
    const float kd = static_cast<float>(example::arg_double(argc, argv, "--kd", 1.0));
    const float gravity_scale = static_cast<float>(example::arg_double(argc, argv, "--gravity-scale", 1.0));
    const bool use_gripper = str_to_bool(argc, argv, "--use_gripper", true);
    if (sample_rate <= 0.0) throw std::runtime_error("--sample-rate must be positive");
    if (max_duration <= 0.0) throw std::runtime_error("--max-duration-s must be positive");

    auto gravity_urdf = example::gravity_urdf_for_gripper(argc, argv, use_gripper);
    rebotarm::RobotModel model(gravity_urdf.path);
    const int dof = model.nq();
    const int command_count = use_gripper ? example::kAllDof : example::kArmDof;

    std::cout << "========================================================================\n";
    std::cout << "  reBotArm hand-guided trajectory recording (C++)\n";
    std::cout << "========================================================================\n";
    std::cout << "[urdf] " << gravity_urdf.path << "\n";
    std::cout << "[model] nq=" << model.nq() << ", nv=" << model.nv() << "\n";
    std::cout << "[gripper/end_link load] scale=" << gravity_urdf.end_link_scale << "\n";
    std::cout << "[free-drive] rate=" << rate << " Hz, kd=" << kd
              << ", gravity_scale=" << gravity_scale << "\n";
    std::cout << "[record] sample_rate=" << sample_rate << " Hz, max_duration=" << max_duration << "s\n";
    std::cout << "After saving, gravity compensation keeps running. Drag the arm back to zero, then Ctrl+C.\n";
    std::cout << "------------------------------------------------------------------------\n";

    example::install_signal_handler();
    auto arm = example::B601Arm::open(example::parse_port(argc, argv));
    arm.enable();
    if (use_gripper) {
      arm.ensure_all_mode(motorbridge::Mode::MIT);
    } else {
      arm.ensure_arm_mode(motorbridge::Mode::MIT);
    }
    std::cout << "[connect] OK\n[enable] OK\n[free-drive] started. Drag the arm by hand.\n";

    std::atomic_bool start_record{false};
    std::atomic_bool stop_record{false};
    std::thread input_thread([&]() {
      std::string line;
      std::cout << "Press Enter to START recording after gravity compensation feels stable...\n";
      std::getline(std::cin, line);
      start_record.store(true, std::memory_order_seq_cst);
      std::cout << "Recording. Press Enter to STOP recording and save. Gravity compensation will keep running...\n";
      std::getline(std::cin, line);
      stop_record.store(true, std::memory_order_seq_cst);
    });

    std::vector<TrajectoryRow> rows;
    bool recording = false;
    auto record_start = std::chrono::steady_clock::now();
    auto next_sample = record_start;
    const auto sample_period = std::chrono::duration<double>(1.0 / sample_rate);

    while (!example::stop_requested()) {
      const auto tick = std::chrono::steady_clock::now();
      const auto states = arm.states();
      std::vector<float> q_all(example::kAllDof, 0.0f);
      std::vector<float> dq_all(example::kAllDof, 0.0f);
      std::vector<float> tau_all(example::kAllDof, 0.0f);
      for (int i = 0; i < std::min(example::kAllDof, static_cast<int>(states.size())); ++i) {
        if (states[i]) {
          q_all[i] = states[i]->pos;
          dq_all[i] = states[i]->vel;
          tau_all[i] = states[i]->torq;
        }
      }

      Eigen::VectorXd q_model(dof);
      for (int i = 0; i < dof; ++i) q_model[i] = q_all[i];
      const Eigen::VectorXd tau_g = gravity_scale * rebotarm::dyn::generalized_gravity(model, q_model);
      for (int i = 0; i < command_count; ++i) {
        const float tau = i < tau_g.size() ? static_cast<float>(tau_g[i]) : 0.0f;
        arm.motors[i].send_mit(q_all[i], 0.0f, 0.0f, kd, tau);
      }

      if (!recording && start_record.load(std::memory_order_seq_cst)) {
        recording = true;
        record_start = tick;
        next_sample = tick;
        std::cout << "[record] started.\n";
      }

      if (recording && tick >= next_sample) {
        const double t = std::chrono::duration<double>(tick - record_start).count();
        TrajectoryRow row;
        row.t = t;
        row.q.resize(dof);
        row.dq.resize(dof);
        row.tau.resize(dof);
        for (int i = 0; i < dof; ++i) {
          row.q[i] = q_all[i];
          row.dq[i] = dq_all[i];
          row.tau[i] = tau_all[i];
        }
        rows.push_back(std::move(row));
        next_sample += std::chrono::duration_cast<std::chrono::steady_clock::duration>(sample_period);
        if (t >= max_duration) {
          std::cout << "[record] max duration reached.\n";
          stop_record.store(true, std::memory_order_seq_cst);
        }
      }

      if (recording && stop_record.load(std::memory_order_seq_cst)) {
        if (!rows.empty()) {
          save_trajectory_csv(output, rows, dof);
          std::cout << "[saved] " << output << "\n";
          print_summary(rows, dof);
        } else {
          std::cout << "[record] no samples captured.\n";
        }
        std::cout << "[free-drive] still running. Drag the arm back to zero, then press Ctrl+C to exit.\n";
        recording = false;
        stop_record.store(false, std::memory_order_seq_cst);
      }

      example::sleep_to_rate(tick, rate);
    }

    if (input_thread.joinable()) input_thread.detach();
    std::cout << "\n[stop] disconnecting...\n";
    release_mit_torque_hold(arm);
    arm.close();
    std::cout << "[done] disconnected\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
