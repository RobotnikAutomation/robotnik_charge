#include "robotnik_charge/robotnik_charge.hpp"
#include <thread>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace robotnik_charge
{

RobotnikCharge::RobotnikCharge()
    : Node("robotnik_charge")
{
  // Start parameters and update from them
  params_timer_ = create_wall_timer(
    500ms, std::bind(&RobotnikCharge::params_timer_callback, this));
  param_listener_ =
    std::make_shared<ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // TF
  tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  dock_frame_transform_ = geometry_msgs::msg::TransformStamped();

  // Battery Subscriber
  battery_status_subscription_ = create_subscription<BatteryStatus>(
    "battery_estimator/data", 10,
    std::bind(&RobotnikCharge::battery_status_callback, this, _1));

  docking_status_subscription_ = create_subscription<DockingStatus>(
    "charge_manager/status", 10,
    std::bind(&RobotnikCharge::docking_status_callback, this, _1));

  // Service client to set relay
  set_charger_relay_ = create_client<SetBool>("~/set_charger_relay");
  while (!set_charger_relay_->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service \"%s\". Exiting.",
                   set_charger_relay_->get_service_name());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service \"%s\" not available, waiting again...", set_charger_relay_->get_service_name());
  }

  set_charger_enable_ = create_client<SetBool>("~/set_charger_enable");
  while (!set_charger_enable_->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service \"%s\". Exiting.",
                   set_charger_enable_->get_service_name());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service \"%s\" not available, waiting again...", set_charger_enable_->get_service_name());
  }

  // Service client to set laser mode
  if (params_.has_safety_lasers)
  {
    set_laser_mode_ = create_client<SetString>("~/set_laser_mode");
    while (!set_laser_mode_->wait_for_service(1s))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service \"%s\". Exiting.", set_laser_mode_->get_service_name());
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service \"%s\" not available, waiting again...", set_laser_mode_->get_service_name());
    }
  }

  //Dock Action Client
  dock_action_client_ = rclcpp_action::create_client<Dock>(this, params_.dock.action_namespace);

  dock_send_goal_options_ = rclcpp_action::Client<Dock>::SendGoalOptions();
  dock_send_goal_options_.goal_response_callback = [this](const GoalHandleDock::SharedPtr &goal_handle) {
    action_goal_callback<Dock>(goal_handle, params_.dock.action_namespace.c_str());
  };
  dock_send_goal_options_.feedback_callback = [this](const GoalHandleDock::SharedPtr &goal_handle,
                                                  const std::shared_ptr<const Dock::Feedback> feedback) {
    action_feedback_callback<Dock>(goal_handle, feedback, params_.dock.action_namespace.c_str());
    // Update remaining distance or other feedback information
    remaining_ = feedback->remaining;
  };
  dock_send_goal_options_.result_callback = [this](const GoalHandleDock::WrappedResult &result) {
    action_result_callback<Dock>(result, dock_finished_, params_.dock.action_namespace.c_str());
  };

  //Move Action Client
  move_action_client_ = rclcpp_action::create_client<Move>(this, params_.move.action_namespace);

  move_send_goal_options_ = rclcpp_action::Client<Move>::SendGoalOptions();

  move_send_goal_options_.goal_response_callback = [this](const GoalHandleMove::SharedPtr &goal_handle) {
    action_goal_callback<Move>(goal_handle, params_.move.action_namespace.c_str());
  };
  move_send_goal_options_.feedback_callback = [this](const GoalHandleMove::SharedPtr &goal_handle,
                                                  const std::shared_ptr<const Move::Feedback> feedback) {
    action_feedback_callback<Move>(goal_handle, feedback, params_.move.action_namespace.c_str());
    // Update remaining distance or other feedback information
    remaining_ = feedback->remaining;
  };
  move_send_goal_options_.result_callback = [this](const GoalHandleMove::WrappedResult &result) {
    action_result_callback<Move>(result, move_finished_, params_.move.action_namespace.c_str());
  };

  //Charge Action Server
  charge_action_server_ = rclcpp_action::create_server<Charge>(this, "charge",
                                                                std::bind(&RobotnikCharge::charge_handle_goal, this, _1, _2),
                                                                std::bind(&RobotnikCharge::charge_handle_cancel, this, _1),
                                                                std::bind(&RobotnikCharge::charge_handle_accepted, this, _1));

  //Uncharge Action Server
  uncharge_action_server_ = rclcpp_action::create_server<Uncharge>(this, "uncharge",
    std::bind(&RobotnikCharge::uncharge_handle_goal, this, _1, _2),
    std::bind(&RobotnikCharge::uncharge_handle_cancel, this, _1),
    std::bind(&RobotnikCharge::uncharge_handle_accepted, this, _1));

  last_battery_msg_ = this->get_clock()->now();
  last_docking_status_msg_ = this->get_clock()->now();
  is_charging_ = false;
  remaining_ = Pose();
  try_number_ = 0;
  time_in_action_ = 0;
  time_in_step_ = 0;
  dock_finished_ = false;
  move_finished_ = false;
  service_callback_executed_ = nullptr;
  service_request_sent_ = false;
  current_request_id_ = -1;
  docking_operation_mode_ = "";
  RCLCPP_INFO(get_logger(), "Robotnik Charge started");
  charge_manager_state_ = RobotnikChargeState::Init;
}

void RobotnikCharge::params_timer_callback()
{
  if (param_listener_->is_old(params_))
  {
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
    RCLCPP_INFO(get_logger(), "Params updated");
  }
}

bool RobotnikCharge::can_charge_be_accepted(std::shared_ptr<const Charge::Goal> goal, std::string& response)
{
  // Check if the action server is in a valid state
  if(charge_manager_state_ != RobotnikChargeState::Init and
    charge_manager_state_ != RobotnikChargeState::Finished)
  {
    response = "There is another action running";
    return false;
  }

  // Check if the battery message is recent
  if ((this->get_clock()->now() - last_battery_msg_).seconds() > 5.0)
  {
    response = "Battery msg not updated in the last 5 seconds";
    return false;
  }

  // Check if the robot is already charging
  if (is_charging_)
  {
    response = "Robot is already charging";
    return false;
  }

  // Check if the frames are empty
  if (goal->dock_frame.empty())
  {
    response = "dock_frame is empty";
    return false;
  }

  if (goal->robot_dock_frame.empty())
  {
    response = "robot_dock_frame is empty";
    return false;
  }

  // Check if the frames exist and can be transformed
  try
  {
    dock_frame_transform_ = tf_buffer_->lookupTransform(
        goal->dock_frame, goal->robot_dock_frame,
        tf2::TimePointZero);
  }
  catch (const tf2::TransformException &ex)
  {
    response = "Could not transform " + goal->dock_frame + " to " + goal->robot_dock_frame + ": " + ex.what();
    return false;
  }

  RCLCPP_INFO(get_logger(), "Received charging goal to frame %s from %s", goal->dock_frame.c_str(), goal->robot_dock_frame.c_str());

  // Check Actions status
  if (!dock_action_client_->wait_for_action_server(std::chrono::seconds(10)))
  {
    response = "Dock server not available after waiting";
    return false;
  }

  if (!move_action_client_->wait_for_action_server(std::chrono::seconds(10)))
  {
    response = "Move server not available after waiting";
    return false;
  }

  // TODO: Safety service
  if(params_.has_safety_lasers)
  {
    RCLCPP_WARN(this->get_logger(), "Changing Lasers mode");
  }

  response = "Goal accepted";
  return true;
}

// Charge Handle
rclcpp_action::GoalResponse RobotnikCharge::charge_handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                        std::shared_ptr<const Charge::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Charge action received");

  rclcpp_action::GoalResponse response;
  std::string response_message;
  if (!can_charge_be_accepted(goal, response_message))
  {
    RCLCPP_ERROR(this->get_logger(), "Charge action rejected: %s", response_message.c_str());
    response = rclcpp_action::GoalResponse::REJECT;
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Goal with uuid: %d Accepted", uuid[0]);
    response = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    switch_to_state(RobotnikChargeState::Ready);
  }
  return response;
}

rclcpp_action::CancelResponse RobotnikCharge::charge_handle_cancel(const std::shared_ptr<GoalHandleCharge> goal_handle)
{

  // Cancel actions
  dock_action_client_->async_cancel_all_goals();
  move_action_client_->async_cancel_all_goals();

  (void)goal_handle; // Unused parameter, but required by the interface
  // Change State
  switch_to_state(RobotnikChargeState::Cancelled);

  RCLCPP_WARN(this->get_logger(), "Goal Cancelled");
  return rclcpp_action::CancelResponse::ACCEPT;

}

void RobotnikCharge::charge_handle_accepted(const std::shared_ptr<GoalHandleCharge> goal_handle)
{
  if(charge_manager_state_ == RobotnikChargeState::Ready)
  {
    std::thread{std::bind(&RobotnikCharge::execute_charge, this, goal_handle)}.detach();
  }
}

void RobotnikCharge::execute_charge(const std::shared_ptr<GoalHandleCharge> goal_handle)
{
  // Goal Accepted
  RCLCPP_INFO(this->get_logger(), "GOAL ACCEPTED");
  rclcpp::Rate loop_rate(params_.rate);

  // Guardar el goal y preparar
  current_charge_handle_ = goal_handle;
  current_goal_ = *goal_handle->get_goal();
  try_number_ = 0;
  switch_to_state(RobotnikChargeState::DeactivatingLasers);
  auto step_timer = std::make_shared<Timer>(params_.step_timeout);

  while(rclcpp::ok() && try_number_ <= current_goal_.retries)
  {
    if(is_charging_)
    {
      switch_to_state(RobotnikChargeState::Charging, step_timer);
    }
    if (step_timer->is_timedout())
    {
      handle_timeout_for_steps(step_timer);
    }
    else
    {
      handle_charge_steps(step_timer);
    }

    time_in_step_ = step_timer->get_elapsed_time();
    time_in_action_ = step_timer->get_global_elapsed_time();

    if (charge_manager_state_ == RobotnikChargeState::Finished)
    {
      return;
    }

    send_charge_feedback();
    loop_rate.sleep();
  }

  // Switch back laser_docking to the original mode if retries exceeded
  if (try_number_ > current_goal_.retries)
  {
    RCLCPP_WARN(this->get_logger(), "Max retries exceeded, aborting...");
    if (params_.has_safety_lasers)
    {
      set_dock_laser_mode(false);
    }
  }

  send_charge_result(false);
  switch_to_state(RobotnikChargeState::Finished);
}

void RobotnikCharge::handle_charge_steps(std::shared_ptr<Timer>& timer)
{
  switch (charge_manager_state_)
  {
    case RobotnikChargeState::DeactivatingLasers:
      if (set_dock_laser_mode(true))
      {
        switch_to_state(RobotnikChargeState::InitDocking, timer);
      }
      break;

    case RobotnikChargeState::InitDocking:
      send_dock_goal();
      switch_to_state(RobotnikChargeState::Docking, timer);
      break;

    case RobotnikChargeState::Docking:
      if(dock_finished_)
      {
        switch_to_state(RobotnikChargeState::InitMoving, timer);
      }
      break;

    case RobotnikChargeState::InitMoving:
      send_move_goal();
      switch_to_state(RobotnikChargeState::Moving, timer);
      break;

    case RobotnikChargeState::Moving:
      if (move_finished_)
      {
        switch_to_state(RobotnikChargeState::ActivateRelay, timer);
      }
      break;

    case RobotnikChargeState::ActivateRelay:
      if (activate_charge(true))
      {
        switch_to_state(RobotnikChargeState::WaitCharging, timer);
        timer->change_duration(params_.timeout_charging_detection);
      }
      break;

    case RobotnikChargeState::WaitCharging:
      if(is_charging_)
      {
        switch_to_state(RobotnikChargeState::Charging, timer);
        timer->change_duration(params_.step_timeout);
        break;
      }
      break;

    case RobotnikChargeState::Charging:
      switch_to_state(RobotnikChargeState::Finished, timer);
      send_charge_result(true);
      break;

    case RobotnikChargeState::Retry:
      retry();
      switch_to_state(RobotnikChargeState::DeactivateRelay, timer);
      break;

    case RobotnikChargeState::DeactivateRelay:
      if (activate_charge(false))
      {
        switch_to_state(RobotnikChargeState::MovingBackwards, timer);
      }
      break;

    case RobotnikChargeState::MovingBackwards:
      if(move_finished_)
      {
        try_number_++;
        switch_to_state(RobotnikChargeState::InitDocking, timer);
      }
      break;

    case RobotnikChargeState::Cancelled:
      if (current_charge_handle_->is_canceling())
      {
        charge_cancel();
      }
      switch_to_state(RobotnikChargeState::Finished, timer);
      send_charge_feedback();
      break;

    case RobotnikChargeState::Aborted:
      charge_abort();
      switch_to_state(RobotnikChargeState::Finished, timer);
      send_charge_feedback();
      break;

    default:
      break;
  }
}

void RobotnikCharge::switch_to_state(const RobotnikChargeState new_state, std::shared_ptr<Timer> timer)
{
  if (is_state_change_possible(new_state))
  {
    if (timer)
    {
      timer->reset();
    }
    RCLCPP_INFO(this->get_logger(), "switch_to_state: switching from %s to %s",
                state_to_text(charge_manager_state_).c_str(), state_to_text(new_state).c_str());
    charge_manager_state_ = new_state;
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "switch_to_state: Invalid state change from %s to %s",
                state_to_text(charge_manager_state_).c_str(), state_to_text(new_state).c_str());
  }
}

bool RobotnikCharge::is_state_change_possible(const RobotnikChargeState new_state)
{
  // Check if the state change is valid
  switch (new_state)
  {
    case RobotnikChargeState::Charging:
      return charge_manager_state_ != RobotnikChargeState::Retry &&
            charge_manager_state_ != RobotnikChargeState::DeactivateRelay &&
            charge_manager_state_ != RobotnikChargeState::MovingBackwards;
    case RobotnikChargeState::Aborted:
      return charge_manager_state_ != RobotnikChargeState::Finished &&
            charge_manager_state_ != RobotnikChargeState::Init &&
            charge_manager_state_ != RobotnikChargeState::Cancelled;
    default:
      return magic_enum::enum_contains<RobotnikChargeState>(new_state);
  }
}

bool RobotnikCharge::can_uncharge_be_accepted(std::shared_ptr<const Uncharge::Goal>goal, std::string& response)
{
  (void)goal; // Unused parameter, can be used in the future for more complex uncharge goals

  // Check if the action server is in a valid state
  if(charge_manager_state_ != RobotnikChargeState::Init and charge_manager_state_ != RobotnikChargeState::Finished)
  {
    response = "There is another action running";
    return false;
  }

  // Check if the battery message is recent
  if ((this->get_clock()->now() - last_battery_msg_).seconds() > 5.0)
  {
    response = "Battery msg not updated in the last 5 second";
    return false;
  }

  // Check Actions status
  if (!move_action_client_->wait_for_action_server(std::chrono::seconds(10)))
  {
    response = "Move server not available after waiting";
    return false;
  }

  // TODO: Safety service
  if(params_.has_safety_lasers)
  {
    RCLCPP_WARN(this->get_logger(), "Changing Lasers mode");
  }

  response = "Goal accepted";
  return true;
}

// Uncharge Handle
rclcpp_action::GoalResponse RobotnikCharge::uncharge_handle_goal(const rclcpp_action::GoalUUID &uuid,
  std::shared_ptr<const Uncharge::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Uncharge action received");
  rclcpp_action::GoalResponse response;
  std::string response_message;

  // If everything is ok, accept the goal
  if (!can_uncharge_be_accepted(goal, response_message))
  {
    RCLCPP_ERROR(this->get_logger(), "Uncharge action rejected: %s", response_message.c_str());
    response = rclcpp_action::GoalResponse::REJECT;
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Goal with uuid: %d Accepted", uuid[0]);
    response = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    switch_to_state(RobotnikChargeState::Ready);
  }

  return response;
}

rclcpp_action::CancelResponse RobotnikCharge::uncharge_handle_cancel(const std::shared_ptr<GoalHandleUncharge> goal_handle)
{
  // Cancel actions
  move_action_client_->async_cancel_all_goals();

  (void)goal_handle; // Unused parameter, but required by the interface
  // Change State
  switch_to_state(RobotnikChargeState::Cancelled);

  RCLCPP_WARN(this->get_logger(), "Goal Cancelled");
  return rclcpp_action::CancelResponse::ACCEPT;

}

void RobotnikCharge::uncharge_handle_accepted(const std::shared_ptr<GoalHandleUncharge> goal_handle)
{
  if(charge_manager_state_ == RobotnikChargeState::Ready)
  {
    std::thread{std::bind(&RobotnikCharge::execute_uncharge, this, goal_handle)}.detach();
  }
}

void RobotnikCharge::execute_uncharge(const std::shared_ptr<GoalHandleUncharge> goal_handle)
{

  // Goal Accepted
  RCLCPP_INFO(this->get_logger(), "GOAL ACCEPTED");
  rclcpp::Rate loop_rate(params_.rate);

  current_uncharge_handle_ = goal_handle;

  switch_to_state(RobotnikChargeState::DeactivateRelay);
  auto step_timer = std::make_shared<Timer>(params_.step_timeout);

  while(rclcpp::ok())
  {
    if (step_timer->is_timedout())
    {
      handle_timeout_for_steps(step_timer);
    }
    else
    {
      handle_uncharge_steps(step_timer);
    }
    time_in_step_ = step_timer->get_elapsed_time();
    time_in_action_ = step_timer->get_global_elapsed_time();

    if (charge_manager_state_ == RobotnikChargeState::Finished)
    {
      return;
    }

    send_uncharge_feedback();
    loop_rate.sleep();
  }

  send_uncharge_result(false);
  switch_to_state(RobotnikChargeState::Finished);
}

void RobotnikCharge::handle_uncharge_steps(std::shared_ptr<Timer>& timer)
{
  switch (charge_manager_state_)
  {
    case RobotnikChargeState::DeactivateRelay:
      if (activate_charge(false))
      {
        switch_to_state(RobotnikChargeState::DeactivatingLasers, timer);
      }
      break;

    case RobotnikChargeState::DeactivatingLasers:
      if (set_dock_laser_mode(true))
      {
        switch_to_state(RobotnikChargeState::InitMoving, timer);
      }
      break;

    case RobotnikChargeState::InitMoving:
      send_move_backwards();
      switch_to_state(RobotnikChargeState::MovingBackwards, timer);
      break;

    case RobotnikChargeState::MovingBackwards:
      if(move_finished_)
      {
        switch_to_state(RobotnikChargeState::InitRotating, timer);
      }
      break;

    case RobotnikChargeState::InitRotating:
      if (params_.rotation != 0.0)
      {
        send_rotation();
        switch_to_state(RobotnikChargeState::Rotating, timer);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Rotation is %.2f rad, skipping this step.", params_.rotation);
        switch_to_state(RobotnikChargeState::ActivatingLasers, timer);
      }
      break;

    case RobotnikChargeState::Rotating:
      if(move_finished_)
      {
        switch_to_state(RobotnikChargeState::ActivatingLasers, timer);
      }
      break;

    case RobotnikChargeState::ActivatingLasers:
      if (set_dock_laser_mode(false))
      {
        switch_to_state(RobotnikChargeState::Finished, timer);
        send_uncharge_result(true);
      }
      break;

    case RobotnikChargeState::Cancelled:
      if (current_uncharge_handle_->is_canceling())
      {
        uncharge_cancel();
      }
      switch_to_state(RobotnikChargeState::Finished, timer);
      send_uncharge_feedback();
      break;

    case RobotnikChargeState::Aborted:
      uncharge_abort();
      switch_to_state(RobotnikChargeState::Finished, timer);
      send_uncharge_feedback();
      break;

    default:
      break;
  }
}

void RobotnikCharge::handle_timeout_for_steps(std::shared_ptr<Timer>& timer)
{
  // Handle timeout for the current state
  switch (charge_manager_state_)
  {
    case RobotnikChargeState::DeactivateRelay:
      remove_pending_requests(set_charger_relay_);
      remove_pending_requests(set_charger_enable_);
      switch_to_state(RobotnikChargeState::Aborted, timer);
      break;

    case RobotnikChargeState::DeactivatingLasers:
      remove_pending_requests(set_laser_mode_);
      switch_to_state(RobotnikChargeState::Aborted, timer);
      break;

    case RobotnikChargeState::ActivatingLasers:
      remove_pending_requests(set_laser_mode_);
      switch_to_state(RobotnikChargeState::Aborted, timer);
      break;

    case RobotnikChargeState::Moving:
      switch_to_state(RobotnikChargeState::Retry, timer);
      break;

    case RobotnikChargeState::ActivateRelay:
      remove_pending_requests(set_charger_relay_);
      remove_pending_requests(set_charger_enable_);
      switch_to_state(RobotnikChargeState::Retry, timer);
      break;

    case RobotnikChargeState::WaitCharging:
      RCLCPP_WARN(this->get_logger(), "Charging timeout, retrying...");
      switch_to_state(RobotnikChargeState::Retry, timer);
      timer->change_duration(params_.step_timeout);
      break;

    default:
      switch_to_state(RobotnikChargeState::Aborted, timer);
      break;
  }
}

/******* Callbacks ********/

//Battery Subscriber
void RobotnikCharge::battery_status_callback(const BatteryStatus::SharedPtr msg)
{
  last_battery_msg_ = this->get_clock()->now();
  is_charging_ = msg->is_charging;
}

void RobotnikCharge::docking_status_callback(const DockingStatus::SharedPtr msg)
{
  last_docking_status_msg_ = this->get_clock()->now();
  docking_operation_mode_ = msg->status.operation_mode;
}

}