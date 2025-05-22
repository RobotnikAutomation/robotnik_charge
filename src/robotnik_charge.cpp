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
                                                                std::bind(&RobotnikCharge::charge_handle_goal, this, _1, _2),
                                                                std::bind(&RobotnikCharge::charge_handle_cancel, this, _1),
                                                                std::bind(&RobotnikCharge::charge_handle_accepted, this, _1));


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
rclcpp_action::GoalResponse RobotnikCharge::charge_handle_goal(const rclcpp_action::GoalUUID &uuid,
                                                        std::shared_ptr<const Charge::Goal> goal)
{
  if(not (charge_manager_state_ == RobotnikChargeState::Init) and not (charge_manager_state_ == RobotnikChargeState::Finished))
  {
    rclcpp_action::GoalResponse response = rclcpp_action::GoalResponse::REJECT;
    RCLCPP_ERROR(this->get_logger(), "There is any other action working");
    return response;
  }

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

rclcpp_action::CancelResponse RobotnikCharge::charge_handle_cancel(const std::shared_ptr<GoalHandleCharge> goal_handle)
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
      charge_manager_state_ = RobotnikChargeState::Finished;        
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
      charge_manager_state_ = RobotnikChargeState::Finished;     
      send_feedback();
      return;
      break;

    case RobotnikChargeState::Aborted:   
      abort();   
      charge_manager_state_ = RobotnikChargeState::Finished;
      send_feedback();
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
  charge_manager_state_ = RobotnikChargeState::Finished;

}

/******* Callbacks ********/

//Battery Subscriber
void RobotnikCharge::battery_status_callback(const BatteryStatus::SharedPtr msg)
{
  last_battery_msg_ = this->get_clock()->now();
  is_charging_ = msg->is_charging;

}

}
