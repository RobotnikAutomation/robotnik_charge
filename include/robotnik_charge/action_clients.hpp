#pragma once
#include "rclcpp_action/rclcpp_action.hpp"

namespace robotnik_charge
{
template <typename T>
void RobotnikCharge::action_feedback_callback(const typename rclcpp_action::ClientGoalHandle<T>::SharedPtr &goal_handle,
                              const std::shared_ptr<const typename T::Feedback>& feedback,
                              const char* action_name)
{
  (void)feedback; // Unused parameter, can be used in the future for more complex feedback handling
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Action %s was rejected by server", action_name);
    if (charge_manager_state_ != RobotnikChargeState::Finished && charge_manager_state_ != RobotnikChargeState::Init)
    {
      switch_to_state(RobotnikChargeState::Aborted);
    }
  }
}

template <typename T>
void RobotnikCharge::action_goal_callback(const typename rclcpp_action::ClientGoalHandle<T>::SharedPtr &goal_handle,
                              const char* action_name)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Action %s was rejected by server", action_name);
    if (charge_manager_state_ != RobotnikChargeState::Finished && charge_manager_state_ != RobotnikChargeState::Init)
    {
      switch_to_state(RobotnikChargeState::Aborted);
    }
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Action %s accepted by server, waiting for result", action_name);
  }
}

template <typename T>
void RobotnikCharge::action_result_callback(const typename rclcpp_action::ClientGoalHandle<T>::WrappedResult &result,
                            bool& finished, const char* action_name)
{
  RCLCPP_INFO(this->get_logger(), "Action %s finished", action_name);
  finished = true;

  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Action %s succeeded. Proceeding to next step.", action_name);
      break;
    case rclcpp_action::ResultCode::ABORTED:
    case rclcpp_action::ResultCode::CANCELED:
    default:
      RCLCPP_ERROR(this->get_logger(), "Action %s failed.", action_name);
      if (charge_manager_state_ != RobotnikChargeState::Finished && charge_manager_state_ != RobotnikChargeState::Init)
      {
        switch_to_state(RobotnikChargeState::Aborted);
      }
      break;
  }
}

}; // namespace robotnik_charge
