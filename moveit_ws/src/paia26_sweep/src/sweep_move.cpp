// PAIA26 v1.0: centered upright -> elbow-level forearm -> left/right 90 -> center -> home.
// Simulator only. First plan every segment without movement. Hard-stop on failure.
// Joint targets are deterministic; the LLM chooses/checks only station order.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

namespace {
using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using MoveItCode = moveit::core::MoveItErrorCode;

constexpr double HALF_PI = 1.5707963267948966;
constexpr double MAX_ERROR_RAD = 0.07;
constexpr double START_TOLERANCE_RAD = 0.12;
const std::array<std::string, 4> JOINTS = {
  "crane_plus_joint1", "crane_plus_joint2", "crane_plus_joint3", "crane_plus_joint4"
};
struct Phase {
  std::string name;
  std::array<double, 4> q;
};
const std::vector<Phase> PHASES = {
  {"CENTER_UP",         {0.0,      0.0,      0.0,  0.0}},
  // Joint 2 remains upright. Joint 3 bends the forearm by 90 degrees.
  // Joint 3 axis is -Y, so negative q3 makes link3/4 point horizontally.
  {"CENTER_FOREARM_LEVEL", {0.0,       0.0, -HALF_PI, 0.0}},
  {"LEFT",                 {-HALF_PI,  0.0, -HALF_PI, 0.0}},
  {"RIGHT",                { HALF_PI,  0.0, -HALF_PI, 0.0}},
  {"CENTER_RETURN",        {0.0,       0.0, -HALF_PI, 0.0}},
  {"HOME",              {0.0,     -1.16,    -2.01, -0.73}},
};
const std::array<double, 4> HOME = {0.0, -1.16, -2.01, -0.73};
const std::array<double, 4> UPRIGHT = {0.0, 0.0, 0.0, 0.0};
const Phase PREP_HOME = {"PREP_HOME", HOME};

std::map<std::string, double> to_map(const Phase& phase) {
  std::map<std::string, double> goal;
  for (std::size_t i = 0; i < JOINTS.size(); ++i) goal[JOINTS[i]] = phase.q[i];
  return goal;
}

// Accept only identified postures: vendor home or upright, with a bounded
// base yaw. If not centered at home, a separate, preplanned home preparation
// is required before the visible six-phase performance.
enum class StartMode { INVALID, HOME_READY, PREP_HOME };

StartMode classify_start(const moveit::core::RobotState& state) {
  double yaw = state.getVariablePosition(JOINTS[0]);
  if (!std::isfinite(yaw)) return StartMode::INVALID;
  double home_err = 0.0, upright_err = 0.0;
  for (std::size_t i = 1; i < JOINTS.size(); ++i) {
    const double v = state.getVariablePosition(JOINTS[i]);
    if (!std::isfinite(v)) return StartMode::INVALID;
    home_err = std::max(home_err, std::abs(v - HOME[i]));
    upright_err = std::max(upright_err, std::abs(v - UPRIGHT[i]));
  }
  StartMode mode = StartMode::INVALID;
  // +0.1 allowance for motor tracking near the deliberately bounded sweep.
  if (std::abs(yaw) <= HALF_PI + START_TOLERANCE_RAD) {
    if (home_err <= START_TOLERANCE_RAD && std::abs(yaw) <= START_TOLERANCE_RAD)
      mode = StartMode::HOME_READY;
    else if (home_err <= START_TOLERANCE_RAD || upright_err <= START_TOLERANCE_RAD)
      mode = StartMode::PREP_HOME;
  }
  std::cout << std::fixed << std::setprecision(5)
            << "PAIA_SWEEP_START state="
            << (mode == StartMode::HOME_READY ? "HOME" :
                mode == StartMode::PREP_HOME ? "PREP_REQUIRED" : "UNRECOGNIZED")
            << " yaw=" << yaw << " home_error=" << home_err
            << " upright_error=" << upright_err << std::endl;
  return mode;
}

// All planning from hypothetical successive states must succeed, including
// any positioning-to-home motion, before the robot receives a command.
bool plan_transition(MoveGroup& group, moveit::core::RobotState& hypothetical,
                     const Phase& p, bool preparation) {
  const auto* jmg = group.getRobotModel()->getJointModelGroup("arm");
  if (!jmg) return false;
  moveit::core::RobotState goal_state(hypothetical);
  for (std::size_t i = 0; i < JOINTS.size(); ++i)
    goal_state.setVariablePosition(JOINTS[i], p.q[i]);
  goal_state.update();
  if (!goal_state.satisfiesBounds(jmg)) {
    std::cerr << "PAIA_SWEEP_STOP bounds phase=" << p.name << std::endl;
    return false;
  }
  group.setStartState(hypothetical);
  if (!group.setJointValueTarget(to_map(p))) {
    std::cerr << "PAIA_SWEEP_STOP invalid_target phase=" << p.name << std::endl;
    return false;
  }
  MoveGroup::Plan planned;
  if (group.plan(planned) != MoveItCode::SUCCESS ||
      planned.trajectory_.joint_trajectory.points.empty()) {
    std::cerr << "PAIA_SWEEP_STOP planning_failed phase=" << p.name
              << " (scene, self-collision, table, or joint limits)" << std::endl;
    return false;
  }
  std::cout << std::fixed << std::setprecision(5)
            << (preparation ? "PAIA_SWEEP_PREP_PLAN_OK" : "PAIA_SWEEP_PLAN_OK")
            << " phase=" << p.name
            << " q1=" << p.q[0] << " q2=" << p.q[1]
            << " q3=" << p.q[2] << " q4=" << p.q[3]
            << " points=" << planned.trajectory_.joint_trajectory.points.size()
            << std::endl;
  hypothetical = goal_state;
  return true;
}

bool plan_all(MoveGroup& group, const moveit::core::RobotState& current,
              StartMode start_mode) {
  moveit::core::RobotState hypothetical(current);
  if (start_mode == StartMode::PREP_HOME &&
      !plan_transition(group, hypothetical, PREP_HOME, true)) return false;
  for (const Phase& p : PHASES)
    if (!plan_transition(group, hypothetical, p, false)) return false;
  group.setStartStateToCurrentState();
  std::cout << "PAIA_SWEEP_PREFLIGHT_SUCCESS phases=" << PHASES.size()
            << " prep=" << (start_mode == StartMode::PREP_HOME ? 1 : 0) << std::endl;
  return true;
}

bool execute_phase(MoveGroup& group, const Phase& p) {
  group.setStartStateToCurrentState();
  if (!group.setJointValueTarget(to_map(p))) {
    std::cerr << "PAIA_SWEEP_STOP invalid_live_target phase=" << p.name << std::endl;
    return false;
  }
  MoveGroup::Plan planned;
  if (group.plan(planned) != MoveItCode::SUCCESS ||
      planned.trajectory_.joint_trajectory.points.empty()) {
    std::cerr << "PAIA_SWEEP_STOP live_plan_failed phase=" << p.name << std::endl;
    return false;
  }
  if (group.execute(planned) != MoveItCode::SUCCESS) {
    std::cerr << "PAIA_SWEEP_STOP execution_failed phase=" << p.name << std::endl;
    return false;
  }
  auto joint_names = group.getJointNames();
  auto joint_values = group.getCurrentJointValues();
  if (joint_names.size() != joint_values.size() || joint_names.empty()) {
    std::cerr << "PAIA_SWEEP_STOP missing_joint_state phase=" << p.name << std::endl;
    return false;
  }
  std::map<std::string, double> measured;
  for (std::size_t i = 0; i < joint_names.size(); ++i)
    measured[joint_names[i]] = joint_values[i];
  std::array<double, 4> q{};
  double error = 0.0;
  for (std::size_t i = 0; i < JOINTS.size(); ++i) {
    auto it = measured.find(JOINTS[i]);
    if (it == measured.end() || !std::isfinite(it->second)) {
      std::cerr << "PAIA_SWEEP_STOP invalid_joint_state phase=" << p.name << std::endl;
      return false;
    }
    q[i] = it->second;
    error = std::max(error, std::abs(q[i] - p.q[i]));
  }
  std::cout << std::fixed << std::setprecision(5)
            << (p.name == "PREP_HOME" ? "PAIA_SWEEP_PREP_POSE phase=" : "PAIA_SWEEP_POSE phase=") << p.name
            << " q1=" << q[0] << " q2=" << q[1]
            << " q3=" << q[2] << " q4=" << q[3]
            << " max_error=" << error << std::endl;
  if (error > MAX_ERROR_RAD) {
    std::cerr << "PAIA_SWEEP_STOP measured_error phase=" << p.name << std::endl;
    return false;
  }
  if (p.name == "PREP_HOME")
    std::cout << "PAIA_SWEEP_PREP_DONE" << std::endl;
  else
    std::cout << "PAIA_SWEEP_PHASE_PASS " << p.name << std::endl;
  // Brief visual pause between steps so the horizontal yaw sweep is recognizable.
  std::this_thread::sleep_for(std::chrono::milliseconds(850));
  return true;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 2 || (std::string(argv[1]) != "plan" && std::string(argv[1]) != "execute")) {
    std::cerr << "Usage: sweep_move plan|execute (plan is non-moving)" << std::endl;
    return 2;
  }
  const bool execute = std::string(argv[1]) == "execute";
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "paia26_sweep_move",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor spinner;
  spinner.add_node(node);
  std::thread worker([&spinner]() { spinner.spin(); });
  int rc = 0;
  try {
    MoveGroup group(node, "arm");
    group.setPlanningTime(12.0);
    group.setNumPlanningAttempts(5);
    group.setMaxVelocityScalingFactor(0.35);
    group.setMaxAccelerationScalingFactor(0.35);
    const auto current = group.getCurrentState(8.0);
    const StartMode start_mode = current ? classify_start(*current) : StartMode::INVALID;
    if (start_mode == StartMode::INVALID) {
      std::cerr << "PAIA_SWEEP_STOP unrecognized_start_state; NO MOVEMENT" << std::endl;
      rc = 3;
    } else if (!plan_all(group, *current, start_mode)) {
      std::cerr << "PAIA_SWEEP_STOP preflight_failed; NO MOVEMENT" << std::endl;
      rc = 4;
    } else if (!execute) {
      std::cout << "PAIA_SWEEP_PLAN_ONLY_DONE phases=" << PHASES.size()
                << " (NO ROBOT MOVEMENT)" << std::endl;
    } else {
      if (start_mode == StartMode::PREP_HOME) {
        std::cout << "PAIA_SWEEP_BEGIN PREP_HOME" << std::endl;
        if (!execute_phase(group, PREP_HOME)) rc = 5;
      }
      if (!rc) {
        for (const Phase& p : PHASES) {
          std::cout << "PAIA_SWEEP_BEGIN " << p.name << std::endl;
          if (!execute_phase(group, p)) { rc = 5; break; }
        }
      }
      if (!rc)
        std::cout << "PAIA_SWEEP_SUCCESS phases=" << PHASES.size()
                  << " left=-1.57080 right=1.57080 center=0.00000" << std::endl;
    }
  } catch (const std::exception& e) {
    std::cerr << "PAIA_SWEEP_ERROR " << e.what() << std::endl;
    rc = 6;
  }
  spinner.cancel();
  if (worker.joinable()) worker.join();
  rclcpp::shutdown();
  return rc;
}
