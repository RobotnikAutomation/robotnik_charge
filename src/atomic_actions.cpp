#include "robotnik_charge/robotnik_charge.hpp"

#include <robotnik_common_msgs/srv/set_string.hpp>
#include <thread>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace robotnik_charge
{

/******* Atomic Actions *******/
void RobotnikCharge::change_laser_mode(bool activate)
{
  //Change Mode
  auto request = std::make_shared<robotnik_common_msgs::srv::SetString::Request>();
  request->data = activate ? params_.laser_mode_during_action : params_.laser_mode_after_action;

  RCLCPP_INFO(this->get_logger(), "Changing laser mode to %s", request->data.c_str());
  while (!set_laser_mode_->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service charge_manager/set_laser_mode. Exiting.");
      charge_manager_state_ = RobotnikChargeState::Aborted;
      return;
    }
    RCLCPP_INFO(this->get_logger(), "charge_manager/set_laser_mode not available, waiting again...");
  }

  set_laser_mode_->async_send_request(request);

  RCLCPP_INFO(this->get_logger(), "Laser mode changed to %s", request->data.c_str());
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
  offset.x = -params_.charge_contact_distance_from_marker - current_goal_.dock_offset;
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

void RobotnikCharge::change_relay_mode(bool activate)
{
  if(activate)
  {
    RCLCPP_INFO(this->get_logger(), "Charge relay mode to true");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Charge relay mode to false");
  }

  auto request = std::make_shared<SetBool::Request>();

  request->data = activate;

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
  else
  {
    RCLCPP_INFO(this->get_logger(), "Last try, waiting 10s");
    rclcpp::sleep_for(10s);
  }

}

void RobotnikCharge::retry()
{
  RCLCPP_INFO(this->get_logger(), "Retrying. Attempt: %d", try_number_);
  init_charging_time_ = this->get_clock()->now();

  send_move_backwards();

}

void RobotnikCharge::send_move_backwards()
{

  move_finished_ = false;
  Move::Goal move_goal;
  Pose target;
  target.x = -params_.step_back_distance;
  move_goal.goal = target;

  move_action_client_->async_send_goal(move_goal, move_send_goal_options_);
}

void RobotnikCharge::send_rotation()
{
  move_finished_ = false;
  Move::Goal move_goal;
  Pose target;
  target.theta = params_.rotation;

  move_goal.goal = target;

  move_action_client_->async_send_goal(move_goal, move_send_goal_options_);
}

void RobotnikCharge::send_charge_feedback()
{
  auto feedback = std::make_shared<Charge::Feedback>();

  feedback->remaining_distance = remaining_;
  feedback->status = state_to_text(charge_manager_state_);
  feedback->try_number = try_number_;
  current_charge_handle_->publish_feedback(feedback);
}

void RobotnikCharge::send_uncharge_feedback()
{
  auto feedback = std::make_shared<Uncharge::Feedback>();

  feedback->remaining_distance = remaining_;
  feedback->status = state_to_text(charge_manager_state_);
  current_uncharge_handle_->publish_feedback(feedback);
}

void RobotnikCharge::send_charge_result(bool success)
{
  auto result = std::make_shared<Charge::Result>();

  if(success)
  {
    result->response.message = "Robot is Charging";
    result->response.success = true;
    current_charge_handle_->succeed(result);
  }
  else
  {
    result->response.message = "Robot is not Charging after action finished";
    result->response.success = false;
    current_charge_handle_->abort(result);
  }
}

void RobotnikCharge::send_uncharge_result(bool success)
{
  auto result = std::make_shared<Uncharge::Result>();

  if(success)
  {
    result->response.message = "Robot is Uncharged";
    result->response.success = true;
    current_uncharge_handle_->succeed(result);
  }
  else
  {
    result->response.message = "Robot is Charging or something is wrong";
    result->response.success = false;
    current_uncharge_handle_->abort(result);
  }
}

void RobotnikCharge::charge_abort()
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

void RobotnikCharge::uncharge_abort()
{
  RCLCPP_WARN(this->get_logger(), "Aborting");

  // Cancel actions
  move_action_client_->async_cancel_all_goals();

  //Send result
  auto result = std::make_shared<Uncharge::Result>();
  result->response.message = "Uncharge aborted, there are some failures during the procedure";
  result->response.success = false;

  current_uncharge_handle_->abort(result);
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

      case RobotnikChargeState::Finished:
      return "Finished";
      break;

      default:
      return "Invalid State";
      break;
  }
}
}
