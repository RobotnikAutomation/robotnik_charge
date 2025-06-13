namespace robotnik_charge
{
template <typename T>
void RobotnikCharge::service_client_handler(std::shared_ptr<rclcpp::Client<T>>& client,
                            std::shared_ptr<typename T::Request>& request,
                            std::shared_ptr<typename T::Response>& response,
                            std::shared_ptr<bool>& success)
{
    success = nullptr; // Init success variable. Only set if error calling service or a response has been received
                        // This prevents success to have a value if it was set before and the service did not return a response yet
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

        success = std::make_shared<bool>(false);
        return;
    }

    auto future = client->async_send_request(request, [this, &response, &success, service_name](const typename rclcpp::Client<T>::SharedFuture shared_future)
    {
        success = std::make_shared<bool>(service_call_callback<T>(shared_future, response));

        if (!(*success))
        {
            RCLCPP_ERROR(this->get_logger(), "Service %s call failed", service_name);
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Service %s called successfully", service_name);
        }
    });

    current_request_id_ = future.request_id;
}

template <typename T>
bool RobotnikCharge::service_call_callback(const typename rclcpp::Client<T>::SharedFuture& future,
                          std::shared_ptr<typename T::Response>& response)
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
void RobotnikCharge::remove_pending_requests(std::shared_ptr<rclcpp::Client<T>>& client)
{
    service_request_sent_ = false;
    if (current_request_id_ >= 0)
    {
        client->remove_pending_request(current_request_id_);
        current_request_id_ = -1; // Reset current request ID after removing it
    }
}

}; // namespace robotnik_charge
