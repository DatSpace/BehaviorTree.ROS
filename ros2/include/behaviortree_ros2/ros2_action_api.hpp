/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <chrono>  // for milliseconds
#include <future>  // for future
#include <memory>  // for std::shared_ptr

#include <rclcpp/rclcpp.hpp>                // for rclcpp::Node
#include <rclcpp_action/rclcpp_action.hpp>  // for rclcpp_action::ClientGoalHandle

#include <behaviortree_ros2/action_client_interface.hpp>  // for ActionClientInterface

namespace BT::ros2 {
/// \brief Generic ROS2 action client API. This class is a wrapper around the
/// actual ROS2 action client and implements a lot boilerplate code around it to
/// make it usable in a BT node.
template <class ActionT>
class ActionClientApi final : public ActionClientInterface<ActionT> {
 public:
  // Declare the typenames needed
  using ActionClient = typename rclcpp_action::Client<ActionT>;
  using ActionGoalOptions =
      typename rclcpp_action::Client<ActionT>::SendGoalOptions;
  using ActionGoal = typename ActionT::Goal;
  using ActionGoalHandle = typename rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult =
      typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;
  using ActionFeedback = typename ActionT::Feedback;
  using CancelResponse = typename ActionT::Impl::CancelGoalService::Response;
  using FeedbackCallback =
      std::function<void(std::shared_ptr<ActionGoalHandle> goal_handle,
                         std::shared_ptr<ActionFeedback const> const feedback)>;
  using ResultCallback =
      std::function<void(WrappedResult const& wrapped_result)>;

  ActionClientApi(std::shared_ptr<rclcpp::Node> node,
                  std::string const& action_topic_name,
                  std::chrono::milliseconds server_response_time)
      : node_{node},
        server_response_time_{server_response_time},
        timeout_point_goal_request_{std::chrono::system_clock::now()},
        timeout_point_cancel_request_{std::chrono::system_clock::now()},
        future_goal_handle_{nullptr},
        future_cancel_response_{nullptr},
        action_client_{
            rclcpp_action::create_client<ActionT>(node_, action_topic_name)},
        goal_handle_{nullptr},
        action_topic_name_{action_topic_name}

  {}

  /// \brief This class sends a goal to the ROS2 action server with it's ROS2
  /// action client member and defines response, feedback and result callback.
  /// \param [in] goal ROS2 action goal to be send to the server
  /// \param [in] feedback_callback Callback that is executed each time the
  /// action server returns feedback to the client \param [in] result_callback
  /// Callback that is executed when the action server sends a result to the
  /// client \return 'True' if the goal was successfully send to the action
  /// server (This does not mean it got accepted!), 'False' if not.
  [[nodiscard]] bool sendGoal(ActionGoal const& goal,
                              FeedbackCallback const& feedback_callback,
                              ResultCallback const& result_callback) override {
    // Reset goal handle
    goal_handle_.reset();

    // Reset futures
    future_goal_handle_.reset();
    future_cancel_response_.reset();

    // Check if action server is initialized
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "Action server '"
                                                   << action_topic_name_
                                                   << "' is not ready");
      return false;
    }

    // Create goal options
    auto const send_goal_options = [&, this] {
      auto options = ActionGoalOptions{};
      // Create response callback
      options.goal_response_callback =
          [=](std::shared_ptr<ActionGoalHandle> const goal_handle) {
            if (goal_handle == nullptr) {
              // If the goal is not accepted we mark the action as inactive
              RCLCPP_ERROR_STREAM(node_->get_logger(),
                                  "Goal was rejected by server");
            }
            // Save goal handle
            goal_handle_ = goal_handle;
          };
      //// Feedback callback is received as function argument and forwarded to
      /// the action client
      options.feedback_callback = feedback_callback;
      // Set result callback
      options.result_callback = [=](WrappedResult const& wrapped_result) {
        // Handle the result with the function provided by the entity which
        // calls sendGoal(..)
        result_callback(wrapped_result);
      };
      return options;
    }();

    future_goal_handle_ =
        std::make_unique<std::shared_future<std::shared_ptr<ActionGoalHandle>>>(
            action_client_->async_send_goal(goal, send_goal_options));
    timeout_point_goal_request_ =
        std::chrono::system_clock::now() + server_response_time_;
    return true;
  };

  /// \brief Abort the currently active goal
  /// \return 'True' if an action cancel request is successfully send to the
  /// action server (This does not mean that the action is successfully
  /// canceled!), 'False' if not.
  [[nodiscard]] bool cancelGoal() override {
    // Check if goal_handle exists
    if (goal_handle_ == nullptr) {
      {
        // Without a goal handle the goal is currently not accepted by the
        // server and thus does not need to be canceled
        return true;
      }
    };
    // Check goal status
    auto const status = goal_handle_->get_status();
    if (status != action_msgs::msg::GoalStatus::STATUS_ACCEPTED &&
        status != action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
      // No need to cancel the goal since it is currently cancelling or in a
      // terminal state
      return true;
    }

    // Cancel goal
    future_cancel_response_ =
        std::make_unique<std::shared_future<std::shared_ptr<CancelResponse>>>(
            action_client_->async_cancel_goal(goal_handle_));
    timeout_point_cancel_request_ =
        std::chrono::system_clock::now() + server_response_time_;
    return true;
  };

  /// \brief Return whether the action is currently active or not
  /// \return 'True' if the action is active, 'False' if not
  [[nodiscard]] bool isActionActive() const override {
    // If goal handle is a nullptr, no answer from the action server is received
    // by now
    if (goal_handle_ == nullptr) {
      // Check if action request is send
      if (future_goal_handle_ == nullptr) {
        return false;
      }
      // Verify that future is valid
      if (!future_goal_handle_->valid()) {
        return false;
      }
      // Check if the response timeout is reached
      if (std::chrono::system_clock::now() > timeout_point_goal_request_) {
        RCLCPP_WARN(node_->get_logger(), "Action goal request timed out");
        return false;
      }
      // Waiting for server response
      return true;
    } else {  // Check if goal_handle is already in terminate state
      auto const goal_status = goal_handle_->get_status();
      if (goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
          goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING) {
        // Check if cancel request is send
        if (future_cancel_response_ != nullptr &&
            future_cancel_response_->valid()) {
          // Check if the response timeout is reached
          if (std::chrono::system_clock::now() >
              timeout_point_cancel_request_) {
            RCLCPP_WARN(node_->get_logger(), "Action cancel request timed out");
            return false;
          }
        }
        // Waiting for result
        return true;
      }
    }
    return false;
  };

 private:
  /// shared ROS2 node
  std::shared_ptr<rclcpp::Node> node_;

  std::chrono::milliseconds server_response_time_;

  std::chrono::time_point<std::chrono::system_clock>
      timeout_point_goal_request_;
  std::chrono::time_point<std::chrono::system_clock>
      timeout_point_cancel_request_;

  // Futures to wait for server responses
  std::unique_ptr<
      std::shared_future<typename std::shared_ptr<ActionGoalHandle>>>
      future_goal_handle_;
  std::unique_ptr<std::shared_future<typename std::shared_ptr<CancelResponse>>>
      future_cancel_response_;

  // ROS2 action client
  std::shared_ptr<ActionClient> action_client_;
  // ROS2 action goal handle
  std::shared_ptr<ActionGoalHandle> goal_handle_;
  // Name of the action server
  std::string action_topic_name_;
};
}  // namespace BT::ros2
