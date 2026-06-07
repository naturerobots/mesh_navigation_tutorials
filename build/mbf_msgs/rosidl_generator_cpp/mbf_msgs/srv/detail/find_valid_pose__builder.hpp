// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mbf_msgs:srv/FindValidPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/srv/find_valid_pose.hpp"


#ifndef MBF_MSGS__SRV__DETAIL__FIND_VALID_POSE__BUILDER_HPP_
#define MBF_MSGS__SRV__DETAIL__FIND_VALID_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mbf_msgs/srv/detail/find_valid_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mbf_msgs
{

namespace srv
{

namespace builder
{

class Init_FindValidPose_Request_use_padded_fp
{
public:
  explicit Init_FindValidPose_Request_use_padded_fp(::mbf_msgs::srv::FindValidPose_Request & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::srv::FindValidPose_Request use_padded_fp(::mbf_msgs::srv::FindValidPose_Request::_use_padded_fp_type arg)
  {
    msg_.use_padded_fp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::srv::FindValidPose_Request msg_;
};

class Init_FindValidPose_Request_current_pose
{
public:
  explicit Init_FindValidPose_Request_current_pose(::mbf_msgs::srv::FindValidPose_Request & msg)
  : msg_(msg)
  {}
  Init_FindValidPose_Request_use_padded_fp current_pose(::mbf_msgs::srv::FindValidPose_Request::_current_pose_type arg)
  {
    msg_.current_pose = std::move(arg);
    return Init_FindValidPose_Request_use_padded_fp(msg_);
  }

private:
  ::mbf_msgs::srv::FindValidPose_Request msg_;
};

class Init_FindValidPose_Request_costmap
{
public:
  explicit Init_FindValidPose_Request_costmap(::mbf_msgs::srv::FindValidPose_Request & msg)
  : msg_(msg)
  {}
  Init_FindValidPose_Request_current_pose costmap(::mbf_msgs::srv::FindValidPose_Request::_costmap_type arg)
  {
    msg_.costmap = std::move(arg);
    return Init_FindValidPose_Request_current_pose(msg_);
  }

private:
  ::mbf_msgs::srv::FindValidPose_Request msg_;
};

class Init_FindValidPose_Request_angle_tolerance
{
public:
  explicit Init_FindValidPose_Request_angle_tolerance(::mbf_msgs::srv::FindValidPose_Request & msg)
  : msg_(msg)
  {}
  Init_FindValidPose_Request_costmap angle_tolerance(::mbf_msgs::srv::FindValidPose_Request::_angle_tolerance_type arg)
  {
    msg_.angle_tolerance = std::move(arg);
    return Init_FindValidPose_Request_costmap(msg_);
  }

private:
  ::mbf_msgs::srv::FindValidPose_Request msg_;
};

class Init_FindValidPose_Request_dist_tolerance
{
public:
  explicit Init_FindValidPose_Request_dist_tolerance(::mbf_msgs::srv::FindValidPose_Request & msg)
  : msg_(msg)
  {}
  Init_FindValidPose_Request_angle_tolerance dist_tolerance(::mbf_msgs::srv::FindValidPose_Request::_dist_tolerance_type arg)
  {
    msg_.dist_tolerance = std::move(arg);
    return Init_FindValidPose_Request_angle_tolerance(msg_);
  }

private:
  ::mbf_msgs::srv::FindValidPose_Request msg_;
};

class Init_FindValidPose_Request_safety_dist
{
public:
  explicit Init_FindValidPose_Request_safety_dist(::mbf_msgs::srv::FindValidPose_Request & msg)
  : msg_(msg)
  {}
  Init_FindValidPose_Request_dist_tolerance safety_dist(::mbf_msgs::srv::FindValidPose_Request::_safety_dist_type arg)
  {
    msg_.safety_dist = std::move(arg);
    return Init_FindValidPose_Request_dist_tolerance(msg_);
  }

private:
  ::mbf_msgs::srv::FindValidPose_Request msg_;
};

class Init_FindValidPose_Request_pose
{
public:
  Init_FindValidPose_Request_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FindValidPose_Request_safety_dist pose(::mbf_msgs::srv::FindValidPose_Request::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_FindValidPose_Request_safety_dist(msg_);
  }

private:
  ::mbf_msgs::srv::FindValidPose_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::srv::FindValidPose_Request>()
{
  return mbf_msgs::srv::builder::Init_FindValidPose_Request_pose();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace srv
{

namespace builder
{

class Init_FindValidPose_Response_pose
{
public:
  explicit Init_FindValidPose_Response_pose(::mbf_msgs::srv::FindValidPose_Response & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::srv::FindValidPose_Response pose(::mbf_msgs::srv::FindValidPose_Response::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::srv::FindValidPose_Response msg_;
};

class Init_FindValidPose_Response_cost
{
public:
  explicit Init_FindValidPose_Response_cost(::mbf_msgs::srv::FindValidPose_Response & msg)
  : msg_(msg)
  {}
  Init_FindValidPose_Response_pose cost(::mbf_msgs::srv::FindValidPose_Response::_cost_type arg)
  {
    msg_.cost = std::move(arg);
    return Init_FindValidPose_Response_pose(msg_);
  }

private:
  ::mbf_msgs::srv::FindValidPose_Response msg_;
};

class Init_FindValidPose_Response_state
{
public:
  Init_FindValidPose_Response_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FindValidPose_Response_cost state(::mbf_msgs::srv::FindValidPose_Response::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_FindValidPose_Response_cost(msg_);
  }

private:
  ::mbf_msgs::srv::FindValidPose_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::srv::FindValidPose_Response>()
{
  return mbf_msgs::srv::builder::Init_FindValidPose_Response_state();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace srv
{

namespace builder
{

class Init_FindValidPose_Event_response
{
public:
  explicit Init_FindValidPose_Event_response(::mbf_msgs::srv::FindValidPose_Event & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::srv::FindValidPose_Event response(::mbf_msgs::srv::FindValidPose_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::srv::FindValidPose_Event msg_;
};

class Init_FindValidPose_Event_request
{
public:
  explicit Init_FindValidPose_Event_request(::mbf_msgs::srv::FindValidPose_Event & msg)
  : msg_(msg)
  {}
  Init_FindValidPose_Event_response request(::mbf_msgs::srv::FindValidPose_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_FindValidPose_Event_response(msg_);
  }

private:
  ::mbf_msgs::srv::FindValidPose_Event msg_;
};

class Init_FindValidPose_Event_info
{
public:
  Init_FindValidPose_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FindValidPose_Event_request info(::mbf_msgs::srv::FindValidPose_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_FindValidPose_Event_request(msg_);
  }

private:
  ::mbf_msgs::srv::FindValidPose_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::srv::FindValidPose_Event>()
{
  return mbf_msgs::srv::builder::Init_FindValidPose_Event_info();
}

}  // namespace mbf_msgs

#endif  // MBF_MSGS__SRV__DETAIL__FIND_VALID_POSE__BUILDER_HPP_
