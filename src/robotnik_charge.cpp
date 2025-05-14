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

  //TF
  tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  dock_frame_transform_ = geometry_msgs::msg::TransformStamped();
      
  //Battery Subscriber
  battery_status_subscription_ = create_subscription<BatteryStatusStamped>(
    "battery_status_stamped/data", 10,
    std::bind(&RobotnikCharge::battery_status_callback, this, _1));

  // Service client to set relay
  set_charger_relay_ = this->create_client<SetBool>("charge_manager/set_charger_relay");

  // while (!set_charger_relay_->wait_for_service(1s))
  // {
  //   if (!rclcpp::ok())
  //   {
  //     RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
  //     return;
  //   }
  //   RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  // }

  //Dock Action Client
  dock_action_client_ = rclcpp_action::create_client<Dock>(this, params_.dock_action);

  dock_send_goal_options_ = rclcpp_action::Client<Dock>::SendGoalOptions();
  dock_send_goal_options_.goal_response_callback = std::bind(&RobotnikCharge::dock_goal_callback, this, _1);
  dock_send_goal_options_.feedback_callback = std::bind(&RobotnikCharge::dock_feedback_callback, this, _1, _2);
  dock_send_goal_options_.result_callback = std::bind(&RobotnikCharge::dock_result_callback, this, _1);


  //Move Action Client
  move_action_client_ = rclcpp_action::create_client<Move>(this, params_.move_action);

  move_send_goal_options_ = rclcpp_action::Client<Move>::SendGoalOptions();
  move_send_goal_options_.goal_response_callback = std::bind(&RobotnikCharge::move_goal_callback, this, _1);
  move_send_goal_options_.feedback_callback = std::bind(&RobotnikCharge::move_feedback_callback, this, _1, _2);
  move_send_goal_options_.result_callback = std::bind(&RobotnikCharge::move_result_callback, this, _1);

  //Charge Action Server
  charge_action_server_ = rclcpp_action::create_server<Charge>(this, "charge",
                                                                std::bind(&RobotnikCharge::handle_goal, this, _1, _2),
                                                                std::bind(&RobotnikCharge::handle_cancel, this, _1),
                                                                std::bind(&RobotnikCharge::handle_accepted, this, _1));


  last_battery_msg = this->get_clock()->now();
  is_charging = false;
  remaining_from_docking = Pose();
  try_number = 0;
                                                              
  RCLCPP_INFO(get_logger(), "Robotnik Charge started");
  charge_manager_state_ = RobotnikChargeState::Init;

}

void RobotnikCharge::params_timer_callback() {
  if (param_listener_->is_old(params_)) {
    param_listener_->refresh_dynamic_parameters();
    params_ = param_listener_->get_params();
    RCLCPP_INFO(get_logger(), "Params updated");
  }
}

// Charge Handle
rclcpp_action::GoalResponse RobotnikCharge::handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                        std::shared_ptr<const Charge::Goal> goal)
{
  if (goal->dock_frame.empty())
  {
    rclcpp_action::GoalResponse response = rclcpp_action::GoalResponse::REJECT;
    RCLCPP_ERROR(this->get_logger(), "dock_frame is empty");
    return response;
  }

  RCLCPP_INFO(get_logger(), "Received charging goal to frame %s", goal->dock_frame.c_str());

  rclcpp::Time now = this->get_clock()->now();

  rclcpp::Duration last_battery_msg_timeout = now - last_battery_msg;

  if (last_battery_msg_timeout > rclcpp::Duration(1, 0))
  {
    rclcpp_action::GoalResponse response = rclcpp_action::GoalResponse::REJECT;
    RCLCPP_ERROR(this->get_logger(), "Battery msg not updated in the last 1 second");
    return response;
  }

  if (!dock_action_client_->wait_for_action_server(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(this->get_logger(), "Dock server not available after waiting");
    rclcpp_action::GoalResponse response = rclcpp_action::GoalResponse::REJECT;
    return response;
  }

  if (!move_action_client_->wait_for_action_server(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(this->get_logger(), "Move server not available after waiting");
    rclcpp_action::GoalResponse response = rclcpp_action::GoalResponse::REJECT;
    return response;
  }

  // TODO: Safety service

  if (is_charging)
  {
    RCLCPP_WARN(this->get_logger(), "Robot is already charging");
    rclcpp_action::GoalResponse response = rclcpp_action::GoalResponse::REJECT;
    return response;
  }

  try
  {
    dock_frame_transform_ = tf_buffer_->lookupTransform(
        goal->dock_frame, goal->robot_dock_frame,
        tf2::TimePointZero);
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_ERROR(
        this->get_logger(), "Could not transform %s to %s: %s",
        goal->dock_frame.c_str(), goal->robot_dock_frame.c_str(), ex.what());
    rclcpp_action::GoalResponse response = rclcpp_action::GoalResponse::REJECT;
    return response;
  }


  rclcpp_action::GoalResponse response = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  charge_manager_state_ = RobotnikChargeState::Ready;
  return response;
}

rclcpp_action::CancelResponse RobotnikCharge::handle_cancel(const std::shared_ptr<GoalHandleCharge> goal_handle)
{
  charge_manager_state_ = RobotnikChargeState::Cancelled;

  dock_action_client_->async_cancel_all_goals();
  move_action_client_->async_cancel_all_goals();

  auto result = std::make_shared<Charge::Result>();

  result->response.message = "Charge cancelled by user";
  result->response.success = false;

  goal_handle->abort(result);

  RCLCPP_WARN(this->get_logger(), "Goal Cancelled");

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotnikCharge::handle_accepted(const std::shared_ptr<GoalHandleCharge> goal_handle)
{
  if(charge_manager_state_ == RobotnikChargeState::Ready)
  {
    std::thread{std::bind(&RobotnikCharge::execute_charging, this, goal_handle)}.detach();
  }
}

void RobotnikCharge::execute_charging(const std::shared_ptr<GoalHandleCharge> goal_handle)
{

  RCLCPP_INFO(this->get_logger(), "GOAL ACCEPTED");
  rclcpp::Rate loop_rate(params_.rate);
  //Check params if has safety lasers or not
  charge_manager_state_ = RobotnikChargeState::ReadyForDocking;

  // Guardar el goal y preparar
  current_charge_goal_ = goal_handle;
  current_goal_ = *goal_handle->get_goal();
  try_number = 0;

  while(rclcpp::ok() && try_number <= current_goal_.retries)
  {

    switch (charge_manager_state_)
    {
    case RobotnikChargeState::DeactivatingLasers:      
      // deactivate_lasers(goal_handle);
      break;
    
    case RobotnikChargeState::ReadyForDocking:   
      send_dock_goal(goal_handle);
      break;

    case RobotnikChargeState::Docking:  
      break;
    
    case RobotnikChargeState::AfterDocking:      
      charge_manager_state_ = RobotnikChargeState::ReadyForMoving;
      break;

    case RobotnikChargeState::ReadyForMoving: 
      send_move_goal(goal_handle);
      break;

    case RobotnikChargeState::Moving:
      break;

    case RobotnikChargeState::AfterMoving:    
      charge_manager_state_ = RobotnikChargeState::ActivateRelay;
      break;
  
    case RobotnikChargeState::ActivateRelay:
      activate_relay(goal_handle);
      break;
    
    case RobotnikChargeState::Charging:      
      send_result(goal_handle);
      break;

    case RobotnikChargeState::Cancelled:      
      return;
      break;
    
    default:
      break;
    }

    send_feedback(goal_handle);
    loop_rate.sleep();

  }

}

/******* Callbacks ********/

//Battery Subscriber
void RobotnikCharge::battery_status_callback(const BatteryStatusStamped::SharedPtr msg)
{
  last_battery_msg = this->get_clock()->now();
  is_charging = msg->status.is_charging;

  if(is_charging)
  {
    dock_action_client_->async_cancel_all_goals();
    move_action_client_->async_cancel_all_goals();
  }
}

//Dock Action Client
void RobotnikCharge::dock_feedback_callback(const GoalHandleDock::SharedPtr &goal_handle, const std::shared_ptr<const Dock::Feedback> feedback)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Dock was rejected by server");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Dock accepted by server, waiting for result");
  }

  remaining_from_docking = feedback->remaining;
}

void RobotnikCharge::dock_goal_callback(const GoalHandleDock::SharedPtr &goal_handle)
{

  charge_manager_state_ = RobotnikChargeState::Docking;
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Dock was rejected by server");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Dock accepted by server, waiting for result");
  }
}

void RobotnikCharge::dock_result_callback(const GoalHandleDock::WrappedResult &result)
{
  charge_manager_state_ = RobotnikChargeState::AfterDocking;
  RCLCPP_INFO(this->get_logger(), "Docking finished");

  switch (result.code)
  {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(this->get_logger(), "Dock succeeded. Proceeding to move.");
    break;
  case rclcpp_action::ResultCode::ABORTED:
  case rclcpp_action::ResultCode::CANCELED:
  default:
    RCLCPP_ERROR(this->get_logger(), "Docking failed.");
    current_charge_goal_->abort(std::make_shared<Charge::Result>());
    break;
  }
}

//Move Action Client
void RobotnikCharge::move_feedback_callback(const GoalHandleMove::SharedPtr &goal_handle, const std::shared_ptr<const Move::Feedback> feedback)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Move was rejected by server");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Move accepted by server, waiting for result");
  }

  remaining_from_docking = feedback->remaining;
}

void RobotnikCharge::move_goal_callback(const GoalHandleMove::SharedPtr &goal_handle)
{
  charge_manager_state_ = RobotnikChargeState::Moving;
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Move was rejected by server");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Move accepted by server, waiting for result");
  }
}

void RobotnikCharge::move_result_callback(const GoalHandleMove::WrappedResult &result)
{
  charge_manager_state_ = RobotnikChargeState::AfterMoving;

  switch (result.code)
  {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(this->get_logger(), "Move succeeded.");
    break;
  case rclcpp_action::ResultCode::ABORTED:
  case rclcpp_action::ResultCode::CANCELED:
  default:
    RCLCPP_ERROR(this->get_logger(), "Move failed.");
    break;
  }

}

/******* Actions *******/
void RobotnikCharge::send_feedback(const std::shared_ptr<GoalHandleCharge> goal_handle)
{
  auto feedback = std::make_shared<Charge::Feedback>();
  auto result = std::make_shared<Charge::Result>();

  feedback->remaining_distance = remaining_from_docking;
  feedback->status = state_to_text(charge_manager_state_);
  feedback->try_number = try_number;
  goal_handle->publish_feedback(feedback);
}

void RobotnikCharge::send_result(const std::shared_ptr<GoalHandleCharge> goal_handle)
{
  auto result = std::make_shared<Charge::Result>();

  result->response.message = "Robot is Charging";
  result->response.success = true;
  goal_handle->succeed(result);
}

void RobotnikCharge::send_dock_goal(const std::shared_ptr<GoalHandleCharge> goal_handle)
{
  charge_manager_state_ = RobotnikChargeState::Docking;
  RCLCPP_INFO(this->get_logger(), "Sending dock goal (attempt %d)", try_number);

  Dock::Goal dock_goal;
  dock_goal.dock_frame = current_goal_.dock_frame;
  dock_goal.robot_dock_frame = current_goal_.robot_dock_frame;

  dock_goal.maximum_velocity.linear.x = params_.max_velocity_x;
  dock_goal.maximum_velocity.linear.y = params_.max_velocity_y;
  dock_goal.maximum_velocity.angular.z = params_.max_velocity_yaw;

  Pose offset;

  offset.x = -current_goal_.dock_offset;
  dock_goal.dock_offset = offset;

  dock_action_client_->async_send_goal(dock_goal, dock_send_goal_options_);
}

void RobotnikCharge::send_move_goal(const std::shared_ptr<GoalHandleCharge> goal_handle)
{
  charge_manager_state_ = RobotnikChargeState::Moving;
  RCLCPP_INFO(this->get_logger(), "Sending move goal");

  try
  {
    dock_frame_transform_ = tf_buffer_->lookupTransform(
        goal_handle->get_goal().get()->dock_frame, goal_handle->get_goal().get()->robot_dock_frame,
        tf2::TimePointZero);
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_ERROR(
        this->get_logger(), "Could not transform %s to %s: %s",
        goal_handle->get_goal().get()->dock_frame.c_str(), goal_handle->get_goal().get()->robot_dock_frame.c_str(), ex.what());
  }

  Move::Goal move_goal;
  Pose target;
  target.x = dock_frame_transform_.transform.translation.x;
  move_goal.goal = target;

  move_action_client_->async_send_goal(move_goal, move_send_goal_options_);
  
}

void RobotnikCharge::activate_relay(const std::shared_ptr<GoalHandleCharge> goal_handle)
{
  auto request = std::make_shared<SetBool::Request>();

  request->data = true;

  auto result = set_charger_relay_->async_send_request(request);

  if(not is_charging)
  {
    charge_manager_state_ = RobotnikChargeState::ReadyForDocking;
    try_number++;
  }

}

std::string RobotnikCharge::state_to_text(RobotnikChargeState state)
{
  switch (state)
    {
    case RobotnikChargeState::Init:      
      return "DeactivatingLasers";
      break;

    case RobotnikChargeState::Ready:      
      return "DeactivatingLasers";
      break;

    case RobotnikChargeState::DeactivatingLasers:      
      return "DeactivatingLasers";
      break;
    
    case RobotnikChargeState::ReadyForDocking:      
      return "ReadyForDocking";
      break;

    case RobotnikChargeState::Docking:  
      return "Docking";   
      break;
    
    case RobotnikChargeState::AfterDocking:      
      return "AfterDocking";
      break;

    case RobotnikChargeState::ReadyForMoving:      
      return "ReadyForMoving";
      break;

    case RobotnikChargeState::Moving:      
      return "Moving";
      break;

    case RobotnikChargeState::AfterMoving:      
      return "AfterMoving";
      break;
  
    case RobotnikChargeState::ActivateRelay:      
      return "ActivateRelay";
      break;
    
    case RobotnikChargeState::Charging:      
      return "Charging";
      break;
    
    case RobotnikChargeState::Cancelled:      
      return "Cancelled";
      break;

    default:
      break;
  } 
}
}
