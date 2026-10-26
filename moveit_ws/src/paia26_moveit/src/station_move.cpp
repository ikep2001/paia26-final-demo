// PAIA26 final project: collision-checked MoveIt motion between fixed CRANE+ poses.
// LLM chooses ONLY station names. Joint positions are fixed, audited constants here.
// SIMULATOR ONLY. Does not sense or physically inspect objects.

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

namespace {
using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using MoveItCode = moveit::core::MoveItErrorCode;

const std::vector<std::string> JOINT_NAMES = {
  "crane_plus_joint1", "crane_plus_joint2", "crane_plus_joint3", "crane_plus_joint4"
};

std::map<std::string, double> target_values(double yaw, bool extended) {
  // 'vertical' and 'home' are the two verified vendor SRDF positions at yaw=0.
  // Only base yaw differs by station. Max |yaw| is 0.40 rad, simulator only.
  const std::vector<double> vals = extended
      ? std::vector<double>{yaw, 0.0, 0.0, 0.0}
      : std::vector<double>{yaw, -1.16, -2.01, -0.73};
  std::map<std::string, double> result;
  for (std::size_t i = 0; i < JOINT_NAMES.size(); ++i) result[JOINT_NAMES[i]] = vals[i];
  return result;
}

bool plan_execute_verify(MoveGroup &group, const std::string &station,
                         const std::string &phase,
                         const std::map<std::string,double> &goal) {
  group.setStartStateToCurrentState();
  if (!group.setJointValueTarget(goal)) {
    std::cerr << "STOP: target violates model joint bounds" << std::endl;
    return false;
  }
  MoveGroup::Plan trajectory;
  const auto planned = group.plan(trajectory);
  if (planned != MoveItCode::SUCCESS) {
    std::cerr << "STOP: MoveIt planning failed at " << station << " " << phase
              << " (collision/limits/IK/planner)" << std::endl;
    return false;
  }
  const auto executed = group.execute(trajectory);
  if (executed != MoveItCode::SUCCESS) {
    std::cerr << "STOP: trajectory execution failed at " << station << " " << phase << std::endl;
    return false;
  }
  const auto measured = group.getCurrentJointValues();
  const auto names = group.getJointNames();
  if (measured.size() != names.size() || measured.empty()) {
    std::cerr << "STOP: no measured MoveIt joint state" << std::endl;
    return false;
  }
  double max_err = 0;
  std::map<std::string,double> by_name;
  for (std::size_t i = 0; i < names.size(); ++i) by_name[names[i]] = measured[i];
  for (const auto &kv : goal) {
    const auto it = by_name.find(kv.first);
    if (it == by_name.end() || !std::isfinite(it->second)) return false;
    max_err = std::max(max_err, std::abs(it->second - kv.second));
  }
  // Provide unambiguous machine-readable output for the Python orchestrator.
  std::cout << std::fixed << std::setprecision(5)
            << "PAIA_POSE station=" << station << " phase=" << phase
            << " yaw=" << by_name["crane_plus_joint1"]
            << " q2=" << by_name["crane_plus_joint2"]
            << " q3=" << by_name["crane_plus_joint3"]
            << " q4=" << by_name["crane_plus_joint4"]
            << " max_error=" << max_err << std::endl;
  if (max_err > 0.07) {
    std::cerr << "STOP: measured joint error too large" << std::endl;
    return false;
  }
  return true;
}
} // namespace

int main(int argc, char *argv[]) {
  // Reject anything except these three labels, BEFORE ROS or moving the arm.
  if (argc < 2 || std::string(argv[1]) == "--ros-args") {
    std::cerr << "Usage: station_move LEFT|CENTER|RIGHT" << std::endl;
    return 2;
  }
  const std::string station = argv[1];
  const std::map<std::string,double> YAW = {
    {"LEFT", -0.40}, {"CENTER", 0.0}, {"RIGHT", 0.40}
  };
  if (YAW.count(station) != 1) {
    std::cerr << "STOP: station not allowed" << std::endl;
    return 2;
  }
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("paia26_station_move",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread worker([&executor]() { executor.spin(); });
  int rc = 0;
  try {
    MoveGroup group(node, "arm");
    group.setPlanningTime(8.0);
    group.setNumPlanningAttempts(3);
    group.setMaxVelocityScalingFactor(0.35);
    group.setMaxAccelerationScalingFactor(0.35);
    std::cout << "PAIA_STATION_BEGIN " << station << std::endl;
    // Each station performs a visible extension/retraction of the entire arm,
    // with a bounded base direction. Not just a small joint-1 swivel.
    if (!plan_execute_verify(group, station, "extended", target_values(YAW.at(station), true)) ||
        !plan_execute_verify(group, station, "retracted", target_values(YAW.at(station), false))) {
      rc = 5;
    } else {
      std::cout << "PAIA_STATION_SUCCESS " << station << std::endl;
    }
  } catch (const std::exception &e) {
    std::cerr << "PAIA_STATION_ERROR: " << e.what() << std::endl;
    rc = 6;
  }
  executor.cancel();
  if (worker.joinable()) worker.join();
  rclcpp::shutdown();
  return rc;
}
