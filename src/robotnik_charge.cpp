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

  // Service client to set relay
  set_charger_relay_ = this->create_client<SetBool>("charge_manager/set_charger_relay");

  while (!set_charger_relay_->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service charge_manager/set_charger_relay. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "charge_manager/set_charger_relay not available, waiting again...");
  }

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


  last_battery_msg_ = this->get_clock()->now();
  is_charging_ = false;
  remaining_ = Pose();
  try_number_ = 0;
  dock_finished_ = false;
  move_finished_ = false;
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

  // Check if robot is already charging
  if ((this->get_clock()->now() - last_battery_msg_).seconds() > 5.0)
  {
    rclcpp_action::GoalResponse response = rclcpp_action::GoalResponse::REJECT;
    RCLCPP_ERROR(this->get_logger(), "Battery msg not updated in the last 5 second");
    return response;
  }

  if (is_charging_)
  {
    RCLCPP_WARN(this->get_logger(), "Robot is already charging");
    rclcpp_action::GoalResponse response = rclcpp_action::GoalResponse::REJECT;
    return response;
  }

  // Check if frames are empty and exists
  if (goal->dock_frame.empty())
  {
    rclcpp_action::GoalResponse response = rclcpp_action::GoalResponse::REJECT;
    RCLCPP_ERROR(this->get_logger(), "dock_frame is empty");
    return response;
  }

  if (goal->robot_dock_frame.empty())
  {
    rclcpp_action::GoalResponse response = rclcpp_action::GoalResponse::REJECT;
    RCLCPP_ERROR(this->get_logger(), "robot_dock_frame is empty");
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

  RCLCPP_INFO(get_logger(), "Received charging goal to frame %s from %s", goal->dock_frame.c_str(), goal->robot_dock_frame.c_str());

  // Check Actions status
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
  if(params_.has_safety_lasers)
  {
    RCLCPP_WARN(this->get_logger(), "Changing Lasers mode");
  }
  
  RCLCPP_WARN(this->get_logger(), "Goal with uuid: %d Accepted", uuid[0]);

  rclcpp_action::GoalResponse response = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  charge_manager_state_ = RobotnikChargeState::Ready;
  return response;
}

rclcpp_action::CancelResponse RobotnikCharge::handle_cancel(const std::shared_ptr<GoalHandleCharge> goal_handle)
{

  // Cancel actions
  dock_action_client_->async_cancel_all_goals();
  move_action_client_->async_cancel_all_goals();

  //Change Laser Mode
  change_laser_mode();

  (void)goal_handle;
  // Change State
  charge_manager_state_ = RobotnikChargeState::Cancelled;

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

  // Goal Accepted
  RCLCPP_INFO(this->get_logger(), "GOAL ACCEPTED");
  rclcpp::Rate loop_rate(params_.rate);

  // Guardar el goal y preparar
  current_charge_handle_ = goal_handle;
  current_goal_ = *goal_handle->get_goal();
  try_number_ = 0;
  init_charging_time_ = this->get_clock()->now();
  
  if(params_.has_safety_lasers)
  {
    charge_manager_state_ = RobotnikChargeState::DeactivatingLasers;
  }else
  {
    charge_manager_state_ = RobotnikChargeState::InitDocking;
  }

  while(rclcpp::ok() && try_number_ <= current_goal_.retries)
  {

    switch (charge_manager_state_)
    {
    case RobotnikChargeState::DeactivatingLasers:      
      change_laser_mode();
      charge_manager_state_ = RobotnikChargeState::InitDocking;
      break;
    
    case RobotnikChargeState::InitDocking:   
      send_dock_goal();
      charge_manager_state_ = RobotnikChargeState::Docking;
      break;

    case RobotnikChargeState::Docking:
      if(dock_finished_)
      {
        charge_manager_state_ = RobotnikChargeState::EndDocking;
      }
      break;
    
    case RobotnikChargeState::EndDocking:      
      charge_manager_state_ = RobotnikChargeState::InitMoving;
      break;

    case RobotnikChargeState::InitMoving: 
      send_move_goal();
      charge_manager_state_ = RobotnikChargeState::Moving;
      break;

    case RobotnikChargeState::Moving:
      if(move_finished_)
      {
        charge_manager_state_ = RobotnikChargeState::EndMoving;
      }
      break;

    case RobotnikChargeState::EndMoving:    
      charge_manager_state_ = RobotnikChargeState::ActivateRelay;
      break;
  
    case RobotnikChargeState::ActivateRelay:
      activate_relay();
      charge_manager_state_ = RobotnikChargeState::WaitCharging;
      break;

    case RobotnikChargeState::WaitCharging:
      wait_charging();
      if(is_charging_)
      {
        charge_manager_state_ = RobotnikChargeState::Charging;
      }
      else
      {
        charge_manager_state_ = RobotnikChargeState::Retry;
      }
      break;
    
    case RobotnikChargeState::Charging:      
      send_result(true);
      return;
      break;
    
    case RobotnikChargeState::Retry:     
      retry(); 
      charge_manager_state_ = RobotnikChargeState::MovingBackwards;
      break;

    case RobotnikChargeState::MovingBackwards:     
      if(move_finished_){
        try_number_++;
        charge_manager_state_ = RobotnikChargeState::InitDocking;
      }
      break;

    case RobotnikChargeState::Cancelled:      
      return;
      break;

    case RobotnikChargeState::Aborted:   
      abort();   
      return;
      break;
    
    default:
      break;
    }

    if((this->get_clock()->now()-init_charging_time_).seconds() > params_.charge_action_timeout)
    {
      RCLCPP_ERROR(this->get_logger(), "Charging timeout, ABORT! (Current action time: %f)", init_charging_time_.seconds());
      charge_manager_state_ = RobotnikChargeState::Aborted;
    }

    if(is_charging_)
    {
      RCLCPP_INFO(this->get_logger(), "Charging detected while action");
      charge_manager_state_ = RobotnikChargeState::Charging;
    }

    send_feedback();
    loop_rate.sleep();

  }

  send_result(false);

}

/******* Callbacks ********/

//Battery Subscriber
void RobotnikCharge::battery_status_callback(const BatteryStatus::SharedPtr msg)
{
  last_battery_msg_ = this->get_clock()->now();
  is_charging_ = msg->is_charging;

}

//Dock Action Client
void RobotnikCharge::dock_feedback_callback(const GoalHandleDock::SharedPtr &goal_handle, const std::shared_ptr<const Dock::Feedback> feedback)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Dock was rejected by server");
    charge_manager_state_ = RobotnikChargeState::Aborted;
  }

  remaining_ = feedback->remaining;
}

void RobotnikCharge::dock_goal_callback(const GoalHandleDock::SharedPtr &goal_handle)
{

  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Dock was rejected by server");
    charge_manager_state_ = RobotnikChargeState::Aborted;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Dock accepted by server, waiting for result");
  }
}

void RobotnikCharge::dock_result_callback(const GoalHandleDock::WrappedResult &result)
{
  RCLCPP_INFO(this->get_logger(), "Docking finished");
  dock_finished_ = true;

  switch (result.code)
  {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(this->get_logger(), "Dock succeeded. Proceeding to move.");
    break;
  case rclcpp_action::ResultCode::ABORTED:
  case rclcpp_action::ResultCode::CANCELED:
  default:
    RCLCPP_ERROR(this->get_logger(), "Docking failed.");
    charge_manager_state_ = RobotnikChargeState::Aborted;
    break;
  }
}

//Move Action Client
void RobotnikCharge::move_feedback_callback(const GoalHandleMove::SharedPtr &goal_handle, const std::shared_ptr<const Move::Feedback> feedback)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Move was rejected by server");
    charge_manager_state_ = RobotnikChargeState::Aborted;
  }

  remaining_ = feedback->remaining;
}

void RobotnikCharge::move_goal_callback(const GoalHandleMove::SharedPtr &goal_handle)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Move was rejected by server");
    charge_manager_state_ = RobotnikChargeState::Aborted;
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Move accepted by server, waiting for result");
  }
}

void RobotnikCharge::move_result_callback(const GoalHandleMove::WrappedResult &result)
{
  move_finished_ = true;

  switch (result.code)
  {
  case rclcpp_action::ResultCode::SUCCEEDED:
    RCLCPP_INFO(this->get_logger(), "Move succeeded.");
    break;
  case rclcpp_action::ResultCode::ABORTED:
  case rclcpp_action::ResultCode::CANCELED:
  default:
    RCLCPP_ERROR(this->get_logger(), "Move failed.");
    charge_manager_state_ = RobotnikChargeState::Aborted;
    break;
  }

}

/******* Atomic Actions *******/
void RobotnikCharge::change_laser_mode()
{
  RCLCPP_INFO(this->get_logger(), "Changing Laser mode");

  //Change Mode
  /*TO DO*/


}

void RobotnikCharge::send_dock_goal()
{
  // Prepare docking
  RCLCPP_INFO(this->get_logger(), "Sending dock goal");
  dock_finished_ = false;

  Dock::Goal dock_goal;
  dock_goal.dock_frame = current_goal_.dock_frame;
  dock_goal.robot_dock_frame = current_goal_.robot_dock_frame;

  dock_goal.maximum_velocity.linear.x = params_.max_velocity_x;
  dock_goal.maximum_velocity.linear.y = params_.max_velocity_y;
  dock_goal.maximum_velocity.angular.z = params_.max_velocity_yaw;

  Pose offset;
  offset.x = -params_.charge_contact_distance_from_marker-current_goal_.dock_offset;
  dock_goal.dock_offset = offset;

  dock_action_client_->async_send_goal(dock_goal, dock_send_goal_options_);
  
}

void RobotnikCharge::send_move_goal()
{
  RCLCPP_INFO(this->get_logger(), "Sending move goal");
  move_finished_ = false;

  Move::Goal move_goal;
  Pose target;

  try
  {
    dock_frame_transform_ = tf_buffer_->lookupTransform(
      current_charge_handle_->get_goal().get()->robot_dock_frame, current_charge_handle_->get_goal().get()->dock_frame,
        tf2::TimePointZero);
    target.x = dock_frame_transform_.transform.translation.x - params_.charge_contact_distance_from_marker;
  }
  catch (const tf2::TransformException &ex)
  {
    RCLCPP_ERROR(
        this->get_logger(), "Could not transform %s to %s: %s",
        current_charge_handle_->get_goal().get()->dock_frame.c_str(), current_charge_handle_->get_goal().get()->robot_dock_frame.c_str(), ex.what());
    target.x = current_goal_.dock_offset;
  }

  move_goal.goal = target;

  move_action_client_->async_send_goal(move_goal, move_send_goal_options_);
  
}

void RobotnikCharge::activate_relay()
{
  RCLCPP_INFO(this->get_logger(), "Activating charge relay");
  
  auto request = std::make_shared<SetBool::Request>();

  request->data = true;

  while (!set_charger_relay_->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service charge_manager/set_charger_relay. Exiting.");
      charge_manager_state_ = RobotnikChargeState::Aborted;
      return;
    }
    RCLCPP_INFO(this->get_logger(), "charge_manager/set_charger_relay not available, waiting again...");
  }

  set_charger_relay_->async_send_request(request);

}

void RobotnikCharge::wait_charging()
{
  RCLCPP_INFO(this->get_logger(), "Waiting charge...");
  if(try_number_ < current_goal_.retries)
  {
    rclcpp::sleep_for(std::chrono::seconds(params_.timeout_charging_detection));
  }
  else{
    RCLCPP_INFO(this->get_logger(), "Last try, waiting 10s");
    rclcpp::sleep_for(10s);
  }

}

void RobotnikCharge::retry()
{
  RCLCPP_INFO(this->get_logger(), "Retrying. Attempt: %d", try_number_);
  init_charging_time_ = this->get_clock()->now();

  move_finished_ = false;
  Move::Goal move_goal;
  Pose target;
  target.x = -0.5;
  move_goal.goal = target;

  move_action_client_->async_send_goal(move_goal, move_send_goal_options_);

}

void RobotnikCharge::send_feedback()
{
  auto feedback = std::make_shared<Charge::Feedback>();
  auto result = std::make_shared<Charge::Result>();

  feedback->remaining_distance = remaining_;
  feedback->status = state_to_text(charge_manager_state_);
  feedback->try_number = try_number_;
  current_charge_handle_->publish_feedback(feedback);
}

void RobotnikCharge::send_result(bool success)
{
  auto result = std::make_shared<Charge::Result>();

  if(success)
  {
    result->response.message = "Robot is Charging";
    result->response.success = true;
    current_charge_handle_->succeed(result);
  }else
  {
    result->response.message = "Robot is not Charging after action finished";
    result->response.success = false;
    current_charge_handle_->abort(result);
  }
}

void RobotnikCharge::abort()
{
  RCLCPP_WARN(this->get_logger(), "Aborting");
  
  // Cancel actions
  dock_action_client_->async_cancel_all_goals();
  move_action_client_->async_cancel_all_goals();

  //Send result
  auto result = std::make_shared<Charge::Result>();
  result->response.message = "Charge aborted, there are some failures during the procedure";
  result->response.success = false;

  current_charge_handle_->abort(result);


}

std::string RobotnikCharge::state_to_text(RobotnikChargeState state)
{
  switch (state)
    {
    case RobotnikChargeState::Init:      
      return "Init";
      break;

    case RobotnikChargeState::Ready:      
      return "Ready";
      break;

    case RobotnikChargeState::DeactivatingLasers:      
      return "DeactivatingLasers";
      break;
    
    case RobotnikChargeState::InitDocking:      
      return "InitDocking";
      break;

    case RobotnikChargeState::Docking:  
      return "Docking";   
      break;
    
    case RobotnikChargeState::EndDocking:      
      return "EndDocking";
      break;

    case RobotnikChargeState::InitMoving:      
      return "InitMoving";
      break;

    case RobotnikChargeState::Moving:      
      return "Moving";
      break;

    case RobotnikChargeState::EndMoving:      
      return "EndMoving";
      break;
  
    case RobotnikChargeState::ActivateRelay:      
      return "ActivateRelay";
      break;
    
    case RobotnikChargeState::WaitCharging:      
      return "WaitCharging";
      break;

    case RobotnikChargeState::Charging:      
      return "Charging";
      break;
    
    case RobotnikChargeState::Cancelled:      
      return "Cancelled";
      break;

    case RobotnikChargeState::Aborted:      
      return "Aborted";
      break;

    case RobotnikChargeState::Retry:      
      return "Retry";
      break;

    case RobotnikChargeState::MovingBackwards:      
      return "MovingBackwards";
      break;

    default:
      return "Invalid State";
      break;
  } 
}
}
