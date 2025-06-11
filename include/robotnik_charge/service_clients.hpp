namespace robotnik_charge
{
template <typename T>
std::shared_ptr<bool> RobotnikCharge::service_client_handler(std::shared_ptr<rclcpp::Client<T>> client,
                            std::shared_ptr<typename T::Request> request,
                            std::shared_ptr<typename T::Response> response)
{
    const char* service_name = client->get_service_name();
    if (!client->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service %s. Exiting.", service_name);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service %s not available", service_name);
        }
        // Delete current request and response to avoid memory leaks
        return std::make_shared<bool>(false);
    }

    std::shared_ptr<bool> success = nullptr;
    auto future = client->async_send_request(request, [this, response, success, &service_name](const typename rclcpp::Client<T>::SharedFuture shared_future)
    {
        *success = service_call_callback<T>(shared_future, response);
        if (!(*success))
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed for %s", service_name);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Service call successful for %s", service_name);
        }
    });

    return success;
}

template <typename T>
bool RobotnikCharge::service_call_callback(const typename rclcpp::Client<T>::SharedFuture future,
                          std::shared_ptr<typename T::Response> response)
{
    if (future.valid())
    {
        std::shared_ptr<typename T::Response> response_future = future.get();
        if (response_future)
        {
            response = response_future;
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service call returned null response");
            return false;
        }
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Service call future is not valid");
        return false;
    }
}

template <typename T>
void RobotnikCharge::remove_pending_requests(std::shared_ptr<rclcpp::Client<T>> client)
{
    client->prune_pending_requests();
}


}; // namespace robotnik_charge
