namespace robotnik_charge
{

/*! \brief
Template function to handle service calls with a client and request/response objects.
    * \tparam T The service type.
    * \param client Shared pointer to the service client.
    * \param request Shared pointer to the service request.
    * \param response Shared pointer to the service response.
    * \param callback_executed Shared pointer to a boolean that indicates if the callback has been executed.
    * 
    * This function sends a service request and waits for the response. It uses a callback
    * to handle the response asynchronously and set callbak_executed.
    * Is meant to be called serveral times in a loop until a response is received.
    * response and callback_executed must live until the callback is executed.
    * callback_executed will be set to true if the service call was successful, or false if it failed or the service is not available.
    * callback_executed will be nullptr if the service was called and the callback was not executed yet.
*/
template <typename T>
void RobotnikCharge::service_call(std::shared_ptr<rclcpp::Client<T>>& client,
                                    std::shared_ptr<typename T::Request>& request,
                                    std::shared_ptr<typename T::Response>& response,
                                    std::shared_ptr<bool>& callback_executed)
{
    if (!service_request_sent_)
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

            callback_executed = std::make_shared<bool>(false); // callback_executed false if service is not available
            return;
        }

        callback_executed = nullptr; // Init callback_executed variable. If service request was not sent yet, this should be null
        // Service call and callback defined
        auto future = client->async_send_request(request, [this, &response, &callback_executed, service_name](const typename rclcpp::Client<T>::SharedFuture shared_future)
        {
            callback_executed = std::make_shared<bool>(service_call_callback<T>(shared_future, response));

            if (!(*callback_executed))
            {
                RCLCPP_ERROR(this->get_logger(), "Service %s call failed", service_name);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Service %s called successfully", service_name);
            }
        });

        current_request_id_ = future.request_id;
        service_request_sent_ = true;
    }

    if (callback_executed) // callback_executed != nullptr; Callback executed
    {
        service_request_sent_ = false; // Reset the request sent flag after callback has been executed
    }
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
