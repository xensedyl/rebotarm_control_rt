#include "example_common.hpp"

#include <exception>
#include <iostream>

namespace {

struct RecordedTrajectory {
  std::vector<double> time;
  std::vector<std::vector<double>> q;
  std::vector<std::vector<double>> dq;

  int samples() const { return static_cast<int>(q.size()); }
  int dof() const { return q.empty() ? 0 : static_cast<int>(q[0].size()); }
  double duration() const { return time.empty() ? 0.0 : time.back(); }
  double dt() const {
    if (time.size() < 2) return 0.0;
    return (time.back() - time.front()) / static_cast<double>(time.size() - 1);
  }
};

struct SampleRow {
  double t = 0.0;
  std::vector<double> q;
  std::vector<double> dq;
  std::vector<double> tau;
};

std::vector<std::string> split_csv(const std::string& line) {
  std::vector<std::string> out;
  std::stringstream ss(line);
  std::string cell;
  while (std::getline(ss, cell, ',')) {
    const auto start = cell.find_first_not_of(" \t\r\n");
    const auto end = cell.find_last_not_of(" \t\r\n");
    out.push_back(start == std::string::npos ? "" : cell.substr(start, end - start + 1));
  }
  return out;
}

int find_col(const std::vector<std::string>& header, const std::string& name) {
  for (int i = 0; i < static_cast<int>(header.size()); ++i) {
    if (header[i] == name) return i;
  }
  return -1;
}

RecordedTrajectory load_trajectory(const std::string& path) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("failed to open " + path);
  std::string line;
  if (!std::getline(in, line)) throw std::runtime_error("empty trajectory CSV");
  const auto header = split_csv(line);
  int dof = 0;
  while (find_col(header, "q" + std::to_string(dof + 1)) >= 0) ++dof;
  if (dof <= 0) throw std::runtime_error("could not infer trajectory dof from q columns");
  std::vector<int> q_cols, dq_cols;
  for (int i = 1; i <= dof; ++i) {
    const int q_col = find_col(header, "q" + std::to_string(i));
    const int dq_col = find_col(header, "dq" + std::to_string(i));
    if (q_col < 0 || dq_col < 0) throw std::runtime_error("trajectory CSV missing q/dq columns");
    q_cols.push_back(q_col);
    dq_cols.push_back(dq_col);
  }
  const int time_col = find_col(header, "time");
  RecordedTrajectory traj;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    const auto cells = split_csv(line);
    std::vector<double> values;
    values.reserve(cells.size());
    for (const auto& cell : cells) values.push_back(std::stod(cell));
    traj.time.push_back(time_col >= 0 ? values[time_col] : static_cast<double>(traj.time.size()));
    traj.q.emplace_back();
    traj.dq.emplace_back();
    for (int col : q_cols) traj.q.back().push_back(values[col]);
    for (int col : dq_cols) traj.dq.back().push_back(values[col]);
  }
  if (traj.q.empty()) throw std::runtime_error("trajectory contains no samples");
  return traj;
}

void print_summary(const RecordedTrajectory& traj) {
  const int dof = traj.dof();
  std::vector<double> q_min(dof, std::numeric_limits<double>::infinity());
  std::vector<double> q_max(dof, -std::numeric_limits<double>::infinity());
  std::vector<double> dq_abs(dof, 0.0);
  for (int i = 0; i < traj.samples(); ++i) {
    for (int j = 0; j < dof; ++j) {
      q_min[j] = std::min(q_min[j], traj.q[i][j]);
      q_max[j] = std::max(q_max[j], traj.q[i][j]);
      dq_abs[j] = std::max(dq_abs[j], std::abs(traj.dq[i][j]));
    }
  }
  std::cout << "samples=" << traj.samples() << " dof=" << dof
            << " duration=" << traj.duration() << "s dt=" << traj.dt() << "s\n";
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

void ensure_parent_dir(const std::string& path) {
  const auto parent = std::filesystem::path(path).parent_path();
  if (!parent.empty()) std::filesystem::create_directories(parent);
}

std::vector<SampleRow> resample_rows(const std::vector<SampleRow>& rows, double fps) {
  if (fps <= 0.0 || rows.size() < 2) return rows;
  std::vector<SampleRow> out;
  const double t0 = rows.front().t;
  const double t_end = rows.back().t;
  int idx = 0;
  for (double t = t0; t <= t_end + 1e-12; t += 1.0 / fps) {
    while (idx + 1 < static_cast<int>(rows.size()) && rows[idx + 1].t <= t) ++idx;
    out.push_back(rows[idx]);
  }
  return out;
}

void save_identification_csv(const std::string& path, const std::vector<SampleRow>& raw_rows,
                             int dof, double output_fps) {
  const auto rows = resample_rows(raw_rows, output_fps);
  if (rows.empty()) throw std::runtime_error("no samples collected");

  std::vector<std::vector<double>> ddq(rows.size(), std::vector<double>(dof, 0.0));
  if (rows.size() >= 2) {
    for (std::size_t i = 0; i < rows.size(); ++i) {
      const std::size_t prev = i == 0 ? 0 : i - 1;
      const std::size_t next = i + 1 >= rows.size() ? rows.size() - 1 : i + 1;
      const double dt = std::max(1e-9, rows[next].t - rows[prev].t);
      for (int j = 0; j < dof; ++j) {
        ddq[i][j] = (rows[next].dq[j] - rows[prev].dq[j]) / dt;
      }
    }
  }

  ensure_parent_dir(path);
  std::ofstream out(path);
  if (!out) throw std::runtime_error("failed to write " + path);
  out << "time";
  for (const char* prefix : {"q", "dq", "ddq", "tau"}) {
    for (int i = 1; i <= dof; ++i) out << "," << prefix << i;
  }
  out << "\n";
  out << std::setprecision(12);
  const double t0 = rows.front().t;
  for (std::size_t i = 0; i < rows.size(); ++i) {
    out << rows[i].t - t0;
    for (int j = 0; j < dof; ++j) out << "," << rows[i].q[j];
    for (int j = 0; j < dof; ++j) out << "," << rows[i].dq[j];
    for (int j = 0; j < dof; ++j) out << "," << ddq[i][j];
    for (int j = 0; j < dof; ++j) out << "," << rows[i].tau[j];
    out << "\n";
  }
}

void move_to_start(example::B601Arm& arm, const std::vector<double>& q_start,
                   const std::vector<float>& vlim, double rate, double threshold_rad) {
  const auto q_cur_all = arm.positions_or_zero();
  double max_delta = 0.0;
  for (int i = 0; i < static_cast<int>(q_start.size()); ++i) {
    max_delta = std::max(max_delta, std::abs(q_start[i] - q_cur_all[i]));
  }
  std::vector<float> target = q_cur_all;
  if (max_delta <= threshold_rad) {
    for (int i = 0; i < static_cast<int>(q_start.size()); ++i) target[i] = static_cast<float>(q_start[i]);
    arm.send_pos_vel_all(target, vlim);
    return;
  }
  const float min_vlim = *std::min_element(vlim.begin(), vlim.begin() + q_start.size());
  const double duration = std::max(max_delta / std::max(1e-6, static_cast<double>(min_vlim)), 1.0);
  const int steps = std::max(2, static_cast<int>(std::ceil(duration * rate)));
  std::cout << "[pre] moving to trajectory start in " << duration << "s (" << steps << " steps)\n";
  for (int step = 0; step <= steps && !example::stop_requested(); ++step) {
    const auto tick = std::chrono::steady_clock::now();
    double alpha = static_cast<double>(step) / steps;
    alpha = alpha * alpha * (3.0 - 2.0 * alpha);
    target = q_cur_all;
    for (int i = 0; i < static_cast<int>(q_start.size()); ++i) {
      target[i] = static_cast<float>((1.0 - alpha) * q_cur_all[i] + alpha * q_start[i]);
    }
    arm.send_pos_vel_all(target, vlim);
    example::sleep_to_rate(tick, rate);
  }
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (example::has_flag(argc, argv, "--help") || example::has_flag(argc, argv, "-h")) {
      std::cout << "Usage: ./12_collect_identification_data --trajectory calibration/recorded.csv "
                   "--output calibration/id_data.csv --port /dev/ttyACM0 [--execute]\n";
      return 0;
    }
    const std::string trajectory_path = example::arg_value(argc, argv, "--trajectory");
    if (trajectory_path.empty()) throw std::runtime_error("--trajectory is required");
    const std::string output = example::arg_value(argc, argv, "--output", "calibration/id_data_train.csv");
    const double rate = example::parse_rate(argc, argv, 150.0);
    const double feedback_rate = example::arg_double(argc, argv, "--feedback-rate", 300.0);
    const double output_fps = example::arg_double(argc, argv, "--output-fps", 0.0);
    const double vlim_value = example::arg_double(argc, argv, "--vlim", 0.8);
    const double start_threshold = example::arg_double(argc, argv, "--start-threshold-deg", 2.0) * example::kPi / 180.0;
    const double settle_s = example::arg_double(argc, argv, "--settle-s", 1.0);
    const bool execute = example::has_flag(argc, argv, "--execute");

    const auto traj = load_trajectory(trajectory_path);
    std::cout << "========================================================================\n";
    std::cout << "  reBotArm recorded trajectory replay and identification data collection (C++)\n";
    std::cout << "========================================================================\n";
    print_summary(traj);
    std::cout << "output: " << output << "\n";
    if (!execute) {
      std::cout << "\n[dry-run] Add --execute to move the real arm and collect data.\n";
      return 0;
    }

    example::install_signal_handler();
    auto arm = example::B601Arm::open(example::parse_port(argc, argv));
    arm.enable();
    arm.ensure_all_mode(motorbridge::Mode::POS_VEL);
    std::cout << "[connect] OK\n[enable] OK\n[mode] POS_VEL\n";

    std::vector<float> vlim(example::kAllDof, static_cast<float>(vlim_value));
    move_to_start(arm, traj.q.front(), vlim, rate, start_threshold);
    if (settle_s > 0.0) {
      std::cout << "[pre] settling " << settle_s << "s\n";
      const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(settle_s);
      std::vector<float> target = arm.positions_or_zero();
      for (int i = 0; i < traj.dof(); ++i) target[i] = static_cast<float>(traj.q.front()[i]);
      while (std::chrono::steady_clock::now() < end && !example::stop_requested()) {
        const auto tick = std::chrono::steady_clock::now();
        arm.send_pos_vel_all(target, vlim);
        example::sleep_to_rate(tick, rate);
      }
    }

    std::vector<SampleRow> rows;
    std::cout << "[record] executing trajectory...\n";
    const auto start = std::chrono::steady_clock::now();
    auto next_sample = start;
    const auto sample_period =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(1.0 / feedback_rate));
    int target_index = 0;
    while (!example::stop_requested() && target_index < traj.samples()) {
      const auto tick = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(tick - start).count();
      while (target_index + 1 < traj.samples() && traj.time[target_index + 1] <= elapsed) ++target_index;
      std::vector<float> target = arm.positions_or_zero();
      for (int i = 0; i < traj.dof(); ++i) target[i] = static_cast<float>(traj.q[target_index][i]);
      arm.send_pos_vel_all(target, vlim);

      if (tick >= next_sample) {
        const auto states = arm.states();
        SampleRow row;
        row.t = elapsed;
        row.q.resize(traj.dof());
        row.dq.resize(traj.dof());
        row.tau.resize(traj.dof());
        for (int i = 0; i < traj.dof(); ++i) {
          if (i < static_cast<int>(states.size()) && states[i]) {
            row.q[i] = states[i]->pos;
            row.dq[i] = states[i]->vel;
            row.tau[i] = states[i]->torq;
          }
        }
        rows.push_back(std::move(row));
        next_sample += sample_period;
      }
      if (elapsed >= traj.time.back()) break;
      example::sleep_to_rate(tick, rate);
    }

    if (!rows.empty()) {
      save_identification_csv(output, rows, traj.dof(), output_fps);
      std::cout << "[saved] " << output << "\n";
      std::cout << "[record] collected " << rows.size() << " raw samples\n";
    }
    arm.close();
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }
}
