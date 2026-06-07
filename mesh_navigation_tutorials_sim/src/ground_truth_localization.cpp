// Copyright 2024 Nature Robots GmbH
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Nature Robots GmbH nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include <geometry_msgs/msg/transform_stamped.hpp>


#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/convert.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include <tf2_msgs/msg/tf_message.hpp>


#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace mesh_navigation_tutorials_sim
{

class GroundTruthLocalizationNode : public rclcpp::Node
{
public:
  explicit GroundTruthLocalizationNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("ground_truth_localization_node", options)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing GroundTruthLocalizationNode");

    auto gz_parent_desc = rcl_interfaces::msg::ParameterDescriptor{};
    gz_parent_desc.description = "The parent frame in Gazebo (e.g., world or map frame)";
    this->declare_parameter("gz_parent_frame", "tray", gz_parent_desc);

    auto gz_child_desc = rcl_interfaces::msg::ParameterDescriptor{};
    gz_child_desc.description = "The child frame in Gazebo representing the robot";
    this->declare_parameter("gz_child_frame", "robot", gz_child_desc);

    auto ros_parent_desc = rcl_interfaces::msg::ParameterDescriptor{};
    ros_parent_desc.description = "The parent frame in ROS (usually 'map')";
    this->declare_parameter("ros_parent_frame", "map", ros_parent_desc);

    auto ros_child_desc = rcl_interfaces::msg::ParameterDescriptor{};
    ros_child_desc.description = "The child frame in ROS representing the robot (e.g., 'base_footprint')";
    this->declare_parameter("ros_child_frame", "base_footprint", ros_child_desc);

    auto ros_odom_desc = rcl_interfaces::msg::ParameterDescriptor{};
    ros_odom_desc.description = "The odom frame in ROS. If provided, the node publishes map -> odom transform.";
    this->declare_parameter("ros_odom_frame", "odom", ros_odom_desc);

    gz_parent_frame_ = this->get_parameter("gz_parent_frame").as_string();
    gz_child_frame_ = this->get_parameter("gz_child_frame").as_string();
    ros_parent_frame_ = this->get_parameter("ros_parent_frame").as_string();
    ros_child_frame_ = this->get_parameter("ros_child_frame").as_string();
    ros_odom_frame_ = this->get_parameter("ros_odom_frame").as_string();
    publish_map_to_odom_ = (ros_odom_frame_ != "");

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // subscribe to tf messages coming from gazebo
    sub_tf_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_gt", 10,
      [this](const tf2_msgs::msg::TFMessage::ConstSharedPtr & msgs)
      {
        tf_cb(msgs);
      });

    RCLCPP_INFO(this->get_logger(), "Searching for Gazebo Transform %s -> %s",
      gz_child_frame_.c_str(), gz_parent_frame_.c_str());
  }

  void tf_cb(const tf2_msgs::msg::TFMessage::ConstSharedPtr & msgs)
  {
    for (const auto & Tbm : msgs->transforms) {
      if (Tbm.header.frame_id == gz_parent_frame_ && Tbm.child_frame_id == gz_child_frame_) {
        if (publish_map_to_odom_) {
          // get last tf from base_footprint -> odom
          geometry_msgs::msg::TransformStamped Tbo;

          try {
            Tbo = tf_buffer_->lookupTransform(
              ros_odom_frame_, ros_child_frame_,
              tf2::TimePointZero);
          } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(
              this->get_logger(), *this->get_clock(), 1000,
              "Could not transform %s to %s: %s",
              ros_child_frame_.c_str(), ros_odom_frame_.c_str(), ex.what());
            return;
          }

          geometry_msgs::msg::TransformStamped Tom;
          Tom.header.frame_id = ros_parent_frame_;
          Tom.header.stamp = Tbm.header.stamp; // Use the ground truth time
          Tom.child_frame_id = ros_odom_frame_;

          tf2::Transform Tbm_tf;
          tf2::fromMsg(Tbm.transform, Tbm_tf);

          tf2::Transform Tbo_tf;
          tf2::fromMsg(Tbo.transform, Tbo_tf);

          tf2::Transform Tom_tf = Tbm_tf * Tbo_tf.inverse();
          tf2::toMsg(Tom_tf, Tom.transform);

          // send T odom -> map as correction of odom only
          tf_broadcaster_->sendTransform(Tom);
        } else {
          // if odom doesnt exist, send T base -> map
          geometry_msgs::msg::TransformStamped Tbm_ros = Tbm;
          Tbm_ros.header.frame_id = ros_parent_frame_;
          Tbm_ros.child_frame_id = ros_child_frame_;
          tf_broadcaster_->sendTransform(Tbm_ros);
        }
      }
    }
  }


  std::string gz_parent_frame_;
  std::string gz_child_frame_;
  std::string ros_parent_frame_;
  std::string ros_child_frame_;

  // frame to skip
  std::string ros_odom_frame_;
  bool publish_map_to_odom_;

  // TF stuff
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_;

};

} // namespace mesh_navigation_tutorials_sim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mesh_navigation_tutorials_sim::GroundTruthLocalizationNode)
