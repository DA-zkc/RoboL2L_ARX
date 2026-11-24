// Thin frozen-trajectory bridge for franky. Ruckig plans; franky's Robot.move
// owns the realtime loop; libfranka's internal joint impedance controls the arm.
// No Python callbacks, custom torque controller, recovery, or gain/load changes.
#include <franka/control_tools.h>
#include <franka/lowpass_filter.h>
#include <franka/joint_velocity_limits.h>
#include <franky/motion/joint_motion.hpp>
#include <franky/motion/motion.hpp>
#include <franky/robot_state.hpp>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <ruckig/ruckig.hpp>
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>
#include "sampled_joint_curve.h"

namespace py = pybind11;
using Joints = std::array<double, 7>;

class StationJointTrajectory {
 public:
  std::vector<ruckig::Trajectory<7>> segments;
  std::vector<double> ends;
  Joints start, goal;
  double duration = 0;
  std::shared_ptr<station::SampledJointCurve> sampled;

  StationJointTrajectory(const std::vector<station::Polynomial>& coefficients, double dt)
      : sampled(std::make_shared<station::SampledJointCurve>(coefficients, dt)) {
    duration=sampled->duration;
    start=std::get<0>(sampled->sample(0));goal=std::get<0>(sampled->sample(duration));
  }

  StationJointTrajectory(const std::vector<Joints>& points, double velocity,
                         double acceleration, double jerk) {
    if (points.size() < 2 ||
        !std::isfinite(velocity + acceleration + jerk) ||
        velocity <= 0 || acceleration <= 0 || jerk <= 0)
      throw std::invalid_argument("invalid_station_trajectory");
    for (const auto& q : points)
      for (double x : q) if (!std::isfinite(x)) throw std::invalid_argument("nonfinite_joint");
    start = points.front(); goal = points.back();
    ruckig::Ruckig<7> planner;
    ruckig::InputParameter<7> input;
    input.current_velocity.fill(0); input.current_acceleration.fill(0);
    input.target_velocity.fill(0); input.target_acceleration.fill(0);
    input.max_velocity.fill(velocity); input.max_acceleration.fill(acceleration);
    input.max_jerk.fill(jerk);
    for (size_t i = 1; i < points.size(); ++i) {
      if (points[i] == points[i-1]) continue;
      input.current_position = points[i-1]; input.target_position = points[i];
      ruckig::Trajectory<7> trajectory;
      if (planner.calculate(input, trajectory) < 0)
        throw std::runtime_error("ruckig_planning_failed");
      duration += trajectory.get_duration();
      if (!std::isfinite(duration))
        throw std::invalid_argument("invalid_trajectory_duration");
      segments.push_back(trajectory); ends.push_back(duration);
    }
    if (segments.empty()) throw std::invalid_argument("zero_length_trajectory");
  }

  std::tuple<Joints, Joints, Joints> sample(double t) const {
    if (sampled) return sampled->sample(t);
    if (!std::isfinite(t)) throw std::invalid_argument("invalid_sample_time");
    Joints q, v, a;
    if (t <= 0) { v.fill(0); a.fill(0); return {start, v, a}; }
    if (t >= duration) { v.fill(0); a.fill(0); return {goal, v, a}; }
    size_t i = std::upper_bound(ends.begin(), ends.end(), t) - ends.begin();
    segments[i].at_time(t - (i ? ends[i-1] : 0), q, v, a);
    return {q, v, a};
  }
};

struct StationTraceRow {
  double time, success_rate;
  Joints q, q_d, dq, torque, command;
};

class StationJointMotion : public franky::Motion<franka::JointPositions> {
 public:
  std::shared_ptr<const StationJointTrajectory> trajectory;
  std::vector<StationTraceRow> trace;
  double elapsed = 0, final_error = 0;
  bool completed = false, ok = false;
  explicit StationJointMotion(std::shared_ptr<StationJointTrajectory> path)
      : StationJointMotion(path, std::make_shared<franky::JointMotion>(
            franky::JointState(franky::toEigen(path->goal)))) {}
  // The injected finisher is only used by the offline replay below. Production
  // uses the unmodified upstream JointMotion, including its final filter hold.
  StationJointMotion(std::shared_ptr<StationJointTrajectory> path,
                     std::shared_ptr<franky::JointMotion> finisher)
      : trajectory(std::move(path)), finisher_(std::move(finisher)) {
    trace.reserve(static_cast<size_t>((trajectory->duration + 1.) * 1000));
  }
 protected:
  void initImpl(const franky::RobotState& state,
                const std::optional<franka::JointPositions>& previous) override {
    finisher_->init(robot(), endpointState(state), franka::JointPositions(trajectory->goal));
  }
  franka::JointPositions nextCommandImpl(
      const franky::RobotState& state, franka::Duration step, franka::Duration rel,
      franka::Duration absolute, const std::optional<franka::JointPositions>&) override {
    elapsed = rel.toSec();
    franka::JointPositions signal(std::get<0>(trajectory->sample(elapsed)));
    if (elapsed >= trajectory->duration) {
      if (!finish_started_) finish_started_ = rel;
      const auto tail_time = rel - *finish_started_;
      signal = finisher_->nextCommand(endpointState(state),
          tail_time.toMSec() ? step : franka::Duration(0), tail_time, absolute,
          franka::JointPositions(trajectory->goal));
    }
    const auto& command = signal.q;
    StationTraceRow row{elapsed, state.control_command_success_rate};
    final_error = 0;
    for (size_t i = 0; i < 7; ++i) {
      row.q[i] = state.q[i]; row.q_d[i] = state.q_d[i]; row.dq[i] = state.dq[i];
      row.torque[i] = state.tau_J[i]; row.command[i] = command[i];
      final_error = std::max(final_error, std::abs(state.q[i] - trajectory->goal[i]));
    }
    if (trace.size() < trace.capacity()) trace.push_back(row);
    ok = completed = signal.motion_finished;
    return signal;
  }
 private:
  std::shared_ptr<franky::JointMotion> finisher_;
  std::optional<franka::Duration> finish_started_;
  franky::RobotState endpointState(const franky::RobotState& measured) const {
    auto reference = measured;
    // The frozen, unfiltered command curve ends at rest. Start a zero-distance
    // upstream JointMotion there, rather than chase measured/filter lag with a
    // second trajectory. Measured feedback remains unchanged in the trace.
    reference.q_d = franky::toEigen(trajectory->goal);
    reference.dq_d.setZero(); reference.ddq_d.setZero();
    return reference;
  }
};

// Substitute only robot-owned dynamics configuration for a software replay.
// Trajectory generation and completion still execute upstream JointMotion.
class OfflineFinishMotion : public franky::JointMotion {
 public:
  explicit OfflineFinishMotion(const Joints& goal)
      : JointMotion(franky::JointState(franky::toEigen(goal))) {}
 protected:
  void setInputLimits(const franky::PositionWaypoint<franky::JointState>&,
                      ruckig::InputParameter<7>& input) const override {
    input.max_velocity.fill(1); input.max_acceleration.fill(1); input.max_jerk.fill(1);
  }
  std::tuple<franky::Vector7d, franky::Vector7d, franky::Vector7d>
  getAbsoluteInputLimits() const override {
    return {franky::Vector7d::Ones(), franky::Vector7d::Ones(), franky::Vector7d::Ones()};
  }
};

void bind_station(py::module& m) {
  m.attr("station_numeric_motion_gates") = false;
  py::class_<StationJointTrajectory, std::shared_ptr<StationJointTrajectory>>(m, "StationJointTrajectory")
      .def(py::init<const std::vector<Joints>&, double, double, double>())
      .def_static("from_coefficients", [](const std::vector<station::Polynomial>& c, double dt) {
        return std::make_shared<StationJointTrajectory>(c, dt);
      })
      .def_readonly("duration", &StationJointTrajectory::duration)
      .def("sample", &StationJointTrajectory::sample);
  py::class_<StationJointMotion, franky::Motion<franka::JointPositions>,
             std::shared_ptr<StationJointMotion>>(m, "StationJointMotion")
      .def(py::init<std::shared_ptr<StationJointTrajectory>>())
      .def(py::init([](std::shared_ptr<StationJointTrajectory> path, double, double) {
        return std::make_shared<StationJointMotion>(path);  // Legacy signature; no thresholds.
      }))
      .def("report", [](const StationJointMotion& motion) {
        py::dict result;
        result["completed"] = motion.completed; result["ok"] = motion.ok;
        result["control_duration_s"] = motion.elapsed;
        result["curve_duration_s"] = motion.trajectory->duration;
        result["completion"] = "franky.JointMotion";
        result["final_max_joint_error_rad"] = motion.final_error;
        result["controller"] = "libfranka.internal_joint_impedance";
        return result;
      })
      .def("trace", [](const StationJointMotion& motion) {
        py::list time, q, q_d, dq, torque, command, success;
        for (const auto& row : motion.trace) {
          time.append(row.time); q.append(row.q); q_d.append(row.q_d); dq.append(row.dq);
          torque.append(row.torque); command.append(row.command); success.append(row.success_rate);
        }
        py::dict result;
        result["reference_time_s"] = time; result["q"] = q; result["q_d"] = q_d;
        result["dq"] = dq; result["tau_J"] = torque; result["reference_q"] = command;
        result["control_command_success_rate"] = success;
        return result;
      });
  m.def("station_realtime_ready", [] {
    std::string error;
    return franka::hasRealtimeKernel() && franka::setCurrentThreadToHighestSchedulerPriority(&error);
  });
  m.def("station_velocity_limits_for_test", [](const std::string& urdf, const std::vector<Joints>& positions) {
    const franka::JointVelocityLimitsConfig limits(urdf);
    std::vector<Joints> lower, upper;
    for (const auto& q : positions) {
      lower.push_back(limits.getLowerJointVelocityLimits(q));
      upper.push_back(limits.getUpperJointVelocityLimits(q));
    }
    return std::make_pair(lower, upper);
  });
  // Software validation of the actual native motion callback, with synthetic
  // measured feedback. Not exported by the CaP API and never physical evidence.
  m.def("station_replay_for_test", [](std::shared_ptr<StationJointTrajectory> path, double error, bool filtered) {
    auto motion = std::make_shared<StationJointMotion>(path, std::make_shared<OfflineFinishMotion>(path->goal));
    franky::RobotState state{};
    state.dq.setZero(); state.dq_d.setZero(); state.tau_J.setZero();
    for (size_t i = 0; i < 7; ++i) state.q_d[i] = path->start[i];
    motion->init(nullptr, state, std::nullopt);
    Joints previous = path->start;
    for (uint64_t ms = 0; ms < static_cast<uint64_t>((path->duration + 4.) * 1000); ++ms) {
      auto q = std::get<0>(path->sample(ms * .001));
      for (size_t i = 0; i < 7; ++i) {
        const double desired = filtered ? franka::lowpassFilter(.001, previous[i], state.q_d[i], 100.) : q[i];
        state.dq_d[i] = state.dq[i] = (desired - state.q_d[i]) / .001;
        state.q_d[i] = desired; state.q[i] = desired + error;
      }
      auto command = motion->nextCommand(state, franka::Duration(ms ? 1 : 0),
          franka::Duration(ms), franka::Duration(ms), std::nullopt);
      previous = command.q;
      if (command.motion_finished) break;
    }
    return motion;
  }, py::arg("path"), py::arg("error"), py::arg("filtered") = false);
}
