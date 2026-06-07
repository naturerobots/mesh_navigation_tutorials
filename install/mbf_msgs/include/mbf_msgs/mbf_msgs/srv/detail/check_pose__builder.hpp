// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mbf_msgs:srv/CheckPose.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/srv/check_pose.hpp"


#ifndef MBF_MSGS__SRV__DETAIL__CHECK_POSE__BUILDER_HPP_
#define MBF_MSGS__SRV__DETAIL__CHECK_POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mbf_msgs/srv/detail/check_pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mbf_msgs
{

namespace srv
{

namespace builder
{

class Init_CheckPose_Request_use_padded_fp
{
public:
  explicit Init_CheckPose_Request_use_padded_fp(::mbf_msgs::srv::CheckPose_Request & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::srv::CheckPose_Request use_padded_fp(::mbf_msgs::srv::CheckPose_Request::_use_padded_fp_type arg)
  {
    msg_.use_padded_fp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPose_Request msg_;
};

class Init_CheckPose_Request_current_pose
{
public:
  explicit Init_CheckPose_Request_current_pose(::mbf_msgs::srv::CheckPose_Request & msg)
  : msg_(msg)
  {}
  Init_CheckPose_Request_use_padded_fp current_pose(::mbf_msgs::srv::CheckPose_Request::_current_pose_type arg)
  {
    msg_.current_pose = std::move(arg);
    return Init_CheckPose_Request_use_padded_fp(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPose_Request msg_;
};

class Init_CheckPose_Request_costmap
{
public:
  explicit Init_CheckPose_Request_costmap(::mbf_msgs::srv::CheckPose_Request & msg)
  : msg_(msg)
  {}
  Init_CheckPose_Request_current_pose costmap(::mbf_msgs::srv::CheckPose_Request::_costmap_type arg)
  {
    msg_.costmap = std::move(arg);
    return Init_CheckPose_Request_current_pose(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPose_Request msg_;
};

class Init_CheckPose_Request_unknown_cost_mult
{
public:
  explicit Init_CheckPose_Request_unknown_cost_mult(::mbf_msgs::srv::CheckPose_Request & msg)
  : msg_(msg)
  {}
  Init_CheckPose_Request_costmap unknown_cost_mult(::mbf_msgs::srv::CheckPose_Request::_unknown_cost_mult_type arg)
  {
    msg_.unknown_cost_mult = std::move(arg);
    return Init_CheckPose_Request_costmap(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPose_Request msg_;
};

class Init_CheckPose_Request_inscrib_cost_mult
{
public:
  explicit Init_CheckPose_Request_inscrib_cost_mult(::mbf_msgs::srv::CheckPose_Request & msg)
  : msg_(msg)
  {}
  Init_CheckPose_Request_unknown_cost_mult inscrib_cost_mult(::mbf_msgs::srv::CheckPose_Request::_inscrib_cost_mult_type arg)
  {
    msg_.inscrib_cost_mult = std::move(arg);
    return Init_CheckPose_Request_unknown_cost_mult(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPose_Request msg_;
};

class Init_CheckPose_Request_lethal_cost_mult
{
public:
  explicit Init_CheckPose_Request_lethal_cost_mult(::mbf_msgs::srv::CheckPose_Request & msg)
  : msg_(msg)
  {}
  Init_CheckPose_Request_inscrib_cost_mult lethal_cost_mult(::mbf_msgs::srv::CheckPose_Request::_lethal_cost_mult_type arg)
  {
    msg_.lethal_cost_mult = std::move(arg);
    return Init_CheckPose_Request_inscrib_cost_mult(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPose_Request msg_;
};

class Init_CheckPose_Request_safety_dist
{
public:
  explicit Init_CheckPose_Request_safety_dist(::mbf_msgs::srv::CheckPose_Request & msg)
  : msg_(msg)
  {}
  Init_CheckPose_Request_lethal_cost_mult safety_dist(::mbf_msgs::srv::CheckPose_Request::_safety_dist_type arg)
  {
    msg_.safety_dist = std::move(arg);
    return Init_CheckPose_Request_lethal_cost_mult(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPose_Request msg_;
};

class Init_CheckPose_Request_pose
{
public:
  Init_CheckPose_Request_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CheckPose_Request_safety_dist pose(::mbf_msgs::srv::CheckPose_Request::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_CheckPose_Request_safety_dist(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPose_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::srv::CheckPose_Request>()
{
  return mbf_msgs::srv::builder::Init_CheckPose_Request_pose();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace srv
{

namespace builder
{

class Init_CheckPose_Response_cost
{
public:
  explicit Init_CheckPose_Response_cost(::mbf_msgs::srv::CheckPose_Response & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::srv::CheckPose_Response cost(::mbf_msgs::srv::CheckPose_Response::_cost_type arg)
  {
    msg_.cost = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPose_Response msg_;
};

class Init_CheckPose_Response_state
{
public:
  Init_CheckPose_Response_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CheckPose_Response_cost state(::mbf_msgs::srv::CheckPose_Response::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_CheckPose_Response_cost(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPose_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::srv::CheckPose_Response>()
{
  return mbf_msgs::srv::builder::Init_CheckPose_Response_state();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace srv
{

namespace builder
{

class Init_CheckPose_Event_response
{
public:
  explicit Init_CheckPose_Event_response(::mbf_msgs::srv::CheckPose_Event & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::srv::CheckPose_Event response(::mbf_msgs::srv::CheckPose_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPose_Event msg_;
};

class Init_CheckPose_Event_request
{
public:
  explicit Init_CheckPose_Event_request(::mbf_msgs::srv::CheckPose_Event & msg)
  : msg_(msg)
  {}
  Init_CheckPose_Event_response request(::mbf_msgs::srv::CheckPose_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_CheckPose_Event_response(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPose_Event msg_;
};

class Init_CheckPose_Event_info
{
public:
  Init_CheckPose_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CheckPose_Event_request info(::mbf_msgs::srv::CheckPose_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_CheckPose_Event_request(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPose_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::srv::CheckPose_Event>()
{
  return mbf_msgs::srv::builder::Init_CheckPose_Event_info();
}

}  // namespace mbf_msgs

#endif  // MBF_MSGS__SRV__DETAIL__CHECK_POSE__BUILDER_HPP_
