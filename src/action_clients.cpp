#include "robotnik_charge/robotnik_charge.hpp"
#include <thread>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace robotnik_charge
{

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

}
