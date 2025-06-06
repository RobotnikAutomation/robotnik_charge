#ifndef _ROBOTNIK_CHARGE_
#define _ROBOTNIK_CHARGE_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotnik_battery_msgs/msg/battery_status.hpp"
#include "robotnik_navigation_msgs/action/charge.hpp"
#include "robotnik_navigation_msgs/action/uncharge.hpp"
#include "robotnik_navigation_msgs/action/dock.hpp"
#include "robotnik_navigation_msgs/action/move.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <robotnik_common_msgs/srv/set_string.hpp>

#include "rclcpp_action/rclcpp_action.hpp"
#include "robotnik_charge/robotnik_charge_parameters.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace robotnik_charge
{
enum RobotnikChargeState
{
  Init = 0,
  Ready = 1,
  DeactivatingLasers = 2,
  ActivatingLasers = 3,
  InitDocking = 4,
  Docking = 5,
  EndDocking = 6,
  InitMoving = 7,
  Moving = 8,
  EndMoving = 9,
  ActivateRelay = 10,
  DeactivateRelay = 11,
  WaitCharging = 12,
  Charging = 13,
  Cancelled = 14,
  Aborted = 15,
  Retry = 16,
  MovingBackwards = 17,
  InitRotating = 18,
  Rotating = 19,
  Finished = 20
};
class RobotnikCharge : public rclcpp::Node
{
public:
  using BatteryStatus = robotnik_battery_msgs::msg::BatteryStatus;
  using Charge = robotnik_navigation_msgs::action::Charge;
  using Uncharge = robotnik_navigation_msgs::action::Uncharge;
  using Dock = robotnik_navigation_msgs::action::Dock;
  using Move = robotnik_navigation_msgs::action::Move;
  using GoalHandleCharge = rclcpp_action::ServerGoalHandle<Charge>;
  using GoalHandleUncharge = rclcpp_action::ServerGoalHandle<Uncharge>;
  using GoalHandleDock = rclcpp_action::ClientGoalHandle<Dock>;
  using GoalHandleMove = rclcpp_action::ClientGoalHandle<Move>;
  using SetString = robotnik_common_msgs::srv::SetString;
  using SetBool = std_srvs::srv::SetBool;
  using Pose = geometry_msgs::msg::Pose2D;
  using Twist = geometry_msgs::msg::Twist;

  explicit RobotnikCharge();

private:
  //Charge
  rclcpp_action::GoalResponse charge_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Charge::Goal> goal);
  rclcpp_action::CancelResponse charge_handle_cancel(const std::shared_ptr<GoalHandleCharge> goal_handle);
  void charge_handle_accepted(const std::shared_ptr<GoalHandleCharge> goal_handle);
  void execute_charge(const std::shared_ptr<GoalHandleCharge> goal_handle);

  //Uncharge
  rclcpp_action::GoalResponse uncharge_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Uncharge::Goal> goal);
  rclcpp_action::CancelResponse uncharge_handle_cancel(const std::shared_ptr<GoalHandleUncharge> goal_handle);
  void uncharge_handle_accepted(const std::shared_ptr<GoalHandleUncharge> goal_handle);
  void execute_uncharge(const std::shared_ptr<GoalHandleUncharge> goal_handle);

  // Callback
  void dock_goal_callback(const GoalHandleDock::SharedPtr & goal_handle);
  void dock_feedback_callback(const GoalHandleDock::SharedPtr & goal_handle, const std::shared_ptr<const Dock::Feedback> feedback);
  void dock_result_callback(const GoalHandleDock::WrappedResult & result);

  void move_goal_callback(const GoalHandleMove::SharedPtr & goal_handle);
  void move_feedback_callback(const GoalHandleMove::SharedPtr & goal_handle, const std::shared_ptr<const Move::Feedback> feedback);
  void move_result_callback(const GoalHandleMove::WrappedResult & result);

  void battery_status_callback(const BatteryStatus::SharedPtr msg);

  // Atomic Actions
  void send_charge_feedback();
  void send_charge_result(bool success);
  void send_uncharge_feedback();
  void send_uncharge_result(bool success);
  void wait_charging();
  void send_dock_goal();
  void send_move_goal();
  void send_move_backwards();
  void send_rotation();
  void change_relay_mode(bool activate);
  void change_laser_mode(bool activate);
  void retry();
  void charge_abort();
  void uncharge_abort();

  std::string state_to_text(RobotnikChargeState state);

  // Params
  rclcpp::TimerBase::SharedPtr params_timer_;
  std::shared_ptr<robotnik_charge::ParamListener> param_listener_;
  robotnik_charge::Params params_;
  void params_timer_callback();

  // Interfaces
  rclcpp::Subscription<BatteryStatus>::SharedPtr battery_status_subscription_;

  rclcpp::Client<SetBool>::SharedPtr set_charger_relay_;
  rclcpp::Client<SetString>::SharedPtr set_laser_mode_;

  rclcpp_action::Client<Dock>::SharedPtr dock_action_client_;
  rclcpp_action::Client<Dock>::SendGoalOptions dock_send_goal_options_;

  rclcpp_action::Client<Move>::SharedPtr move_action_client_;
  rclcpp_action::Client<Move>::SendGoalOptions move_send_goal_options_;

  rclcpp_action::Server<Charge>::SharedPtr charge_action_server_;
  rclcpp_action::Server<Uncharge>::SharedPtr uncharge_action_server_;

  //Variables
  int try_number_;
  rclcpp::Time init_charging_time_;
  bool is_charging_;
  Pose remaining_;

  geometry_msgs::msg::TransformStamped dock_frame_transform_;

  RobotnikChargeState charge_manager_state_;
  rclcpp::Time last_battery_msg_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


  std::shared_ptr<GoalHandleUncharge> current_uncharge_handle_;
  std::shared_ptr<GoalHandleCharge> current_charge_handle_;
  Charge::Goal current_goal_;

  bool dock_finished_;
  bool move_finished_;

};

}

#endif  // _ROBOTNIK_CHARGE_