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
    std::cout << "GroundTruthLocalizationNode" << std::endl;

    this->declare_parameter("gz_parent_frame", "tray");
    this->declare_parameter("gz_child_frame", "robot");
    this->declare_parameter("ros_parent_frame", "map");
    this->declare_parameter("ros_child_frame", "base_footprint");
    this->declare_parameter("ros_odom_frame", "odom");

    gz_parent_frame_ = this->get_parameter("gz_parent_frame").as_string();
    gz_child_frame_ = this->get_parameter("gz_child_frame").as_string();
    ros_parent_frame_ = this->get_parameter("ros_parent_frame").as_string();
    ros_child_frame_ = this->get_parameter("ros_child_frame").as_string();
    ros_odom_frame_ = this->get_parameter("ros_odom_frame").as_string();
    skip_odom_frame_ = (ros_odom_frame_ != "");

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
      [ = ](const tf2_msgs::msg::TFMessage::ConstSharedPtr & msgs) -> void
      {
        tf_cb(msgs);
      });

    std::cout << "Searching for Gazebo Transform " << gz_child_frame_ << " -> " <<
      gz_parent_frame_ << std::endl;
  }

  void tf_cb(const tf2_msgs::msg::TFMessage::ConstSharedPtr & msgs)
  {
    for (geometry_msgs::msg::TransformStamped Tbm : msgs->transforms) {
      if (Tbm.header.frame_id == gz_parent_frame_) {
        if (Tbm.child_frame_id == gz_child_frame_) {
          if (skip_odom_frame_) {
            // plan:
            // get last tf from base_footprint -> odom
            geometry_msgs::msg::TransformStamped Tbo;

            // Look up for the transformation between target_frame and turtle2 frames
            // and send velocity commands for turtle2 to reach target_frame
            try {
              Tbo = tf_buffer_->lookupTransform(
                ros_odom_frame_, ros_child_frame_,
                tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
              RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                ros_child_frame_.c_str(), ros_odom_frame_.c_str(), ex.what());
              return;
            }

            geometry_msgs::msg::TransformStamped Tom;
            // Tom = Tbm * ~Tbo;
            Tom.header.frame_id = ros_parent_frame_;
            Tom.header.stamp = Tbo.header.stamp; // what time to use? I think it should be the odom time since we are correcting it
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
            Tbm.header.frame_id = ros_parent_frame_;
            Tbm.child_frame_id = ros_child_frame_;
            tf_broadcaster_->sendTransform(Tbm);
          }
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
  bool skip_odom_frame_;

  // TF stuff
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_tf_;

};

} // namespace mesh_navigation_tutorials_sim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(mesh_navigation_tutorials_sim::GroundTruthLocalizationNode)
