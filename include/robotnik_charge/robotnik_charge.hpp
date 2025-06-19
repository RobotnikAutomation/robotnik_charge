#ifndef _ROBOTNIK_CHARGE_
#define _ROBOTNIK_CHARGE_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robotnik_battery_msgs/msg/battery_status.hpp"
#include "robotnik_battery_msgs/msg/docking_station_status_stamped.hpp"
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

#include "robotnik_charge/magic_enum.hpp"
#include "robotnik_charge/custom_timer.hpp"

namespace robotnik_charge
{
enum RobotnikChargeState
{
  Init,
  Ready,
  DeactivatingLasers,
  ActivatingLasers,
  InitDocking,
  Docking,
  InitMoving,
  Moving,
  ActivateRelay,
  DeactivateRelay,
  WaitCharging,
  Charging,
  Cancelled,
  Aborted,
  Retry,
  MovingBackwards,
  InitRotating,
  Rotating,
  Finished
};
class RobotnikCharge : public rclcpp::Node
{
public:
  using BatteryStatus = robotnik_battery_msgs::msg::BatteryStatus;
  using DockingStatus = robotnik_battery_msgs::msg::DockingStationStatusStamped;
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
  bool can_charge_be_accepted(std::shared_ptr<const Charge::Goal> goal, std::string & response);
  rclcpp_action::GoalResponse charge_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Charge::Goal> goal);
  rclcpp_action::CancelResponse charge_handle_cancel(const std::shared_ptr<GoalHandleCharge> goal_handle);
  void charge_handle_accepted(const std::shared_ptr<GoalHandleCharge> goal_handle);
  void execute_charge(const std::shared_ptr<GoalHandleCharge> goal_handle);
  void handle_charge_steps(std::shared_ptr<Timer>& timer);

  //Uncharge
  bool can_uncharge_be_accepted(std::shared_ptr<const Uncharge::Goal> goal, std::string & response);
  rclcpp_action::GoalResponse uncharge_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Uncharge::Goal> goal);
  rclcpp_action::CancelResponse uncharge_handle_cancel(const std::shared_ptr<GoalHandleUncharge> goal_handle);
  void uncharge_handle_accepted(const std::shared_ptr<GoalHandleUncharge> goal_handle);
  void execute_uncharge(const std::shared_ptr<GoalHandleUncharge> goal_handle);
  void handle_uncharge_steps(std::shared_ptr<Timer>& timer);

  // Charge and Uncharge
  void handle_timeout_for_steps(std::shared_ptr<Timer>& timer);

  // Callbacks
  void battery_status_callback(const BatteryStatus::SharedPtr msg);
  void docking_status_callback(const DockingStatus::SharedPtr msg);

  // Action callbacks
  template <typename T>
  void action_feedback_callback(const typename rclcpp_action::ClientGoalHandle<T>::SharedPtr &goal_handle,
                                const std::shared_ptr<const typename T::Feedback>& feedback,
                                const char* action_name);

  template <typename T>
  void action_goal_callback(const typename rclcpp_action::ClientGoalHandle<T>::SharedPtr &goal_handle,
                            const char* action_name);

  template <typename T>
  void action_result_callback(const typename rclcpp_action::ClientGoalHandle<T>::WrappedResult &result,
                              bool& finished, const char* action_name);

  // Service clinets handlers and callback
  template <typename T>
  void service_call(std::shared_ptr<rclcpp::Client<T>>& client,
                    std::shared_ptr<typename T::Request>& request,
                    std::shared_ptr<typename T::Response>& response,
                    std::shared_ptr<bool>& callback_executed);

  template <typename T>
  bool service_call_callback(const typename rclcpp::Client<T>::SharedFuture& future,
                          std::shared_ptr<typename T::Response>& response);

  template <typename T>
  void remove_pending_requests(std::shared_ptr<rclcpp::Client<T>>& client);

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
  bool set_charge_relay(bool activate);
  bool set_dock_laser_mode(bool activate);
  void retry();
  void charge_abort();
  void uncharge_abort();

  std::string state_to_text(RobotnikChargeState state);
  void switch_to_state(RobotnikChargeState new_state, std::shared_ptr<Timer> timer = nullptr);

  // Params
  rclcpp::TimerBase::SharedPtr params_timer_;
  std::shared_ptr<robotnik_charge::ParamListener> param_listener_;
  robotnik_charge::Params params_;
  void params_timer_callback();

  // Interfaces
  rclcpp::Subscription<BatteryStatus>::SharedPtr battery_status_subscription_;
  rclcpp::Subscription<DockingStatus>::SharedPtr docking_status_subscription_;

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
  bool is_charging_;
  Pose remaining_;
  std::string docking_operation_mode_;

  geometry_msgs::msg::TransformStamped dock_frame_transform_;

  RobotnikChargeState charge_manager_state_;
  rclcpp::Time last_battery_msg_;
  rclcpp::Time last_docking_status_msg_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;


  std::shared_ptr<GoalHandleUncharge> current_uncharge_handle_;
  std::shared_ptr<GoalHandleCharge> current_charge_handle_;
  Charge::Goal current_goal_;

  bool dock_finished_;
  bool move_finished_;

  std::shared_ptr<bool> service_callback_executed_;
  bool service_request_sent_;
  int64_t current_request_id_;
};

}

#include "robotnik_charge/action_clients.hpp"
#include "robotnik_charge/service_clients.hpp"


#endif  // _ROBOTNIK_CHARGE_