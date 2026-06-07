// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mbf_msgs:srv/CheckPath.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/srv/check_path.hpp"


#ifndef MBF_MSGS__SRV__DETAIL__CHECK_PATH__BUILDER_HPP_
#define MBF_MSGS__SRV__DETAIL__CHECK_PATH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mbf_msgs/srv/detail/check_path__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mbf_msgs
{

namespace srv
{

namespace builder
{

class Init_CheckPath_Request_path_cells_only
{
public:
  explicit Init_CheckPath_Request_path_cells_only(::mbf_msgs::srv::CheckPath_Request & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::srv::CheckPath_Request path_cells_only(::mbf_msgs::srv::CheckPath_Request::_path_cells_only_type arg)
  {
    msg_.path_cells_only = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPath_Request msg_;
};

class Init_CheckPath_Request_use_padded_fp
{
public:
  explicit Init_CheckPath_Request_use_padded_fp(::mbf_msgs::srv::CheckPath_Request & msg)
  : msg_(msg)
  {}
  Init_CheckPath_Request_path_cells_only use_padded_fp(::mbf_msgs::srv::CheckPath_Request::_use_padded_fp_type arg)
  {
    msg_.use_padded_fp = std::move(arg);
    return Init_CheckPath_Request_path_cells_only(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPath_Request msg_;
};

class Init_CheckPath_Request_skip_poses
{
public:
  explicit Init_CheckPath_Request_skip_poses(::mbf_msgs::srv::CheckPath_Request & msg)
  : msg_(msg)
  {}
  Init_CheckPath_Request_use_padded_fp skip_poses(::mbf_msgs::srv::CheckPath_Request::_skip_poses_type arg)
  {
    msg_.skip_poses = std::move(arg);
    return Init_CheckPath_Request_use_padded_fp(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPath_Request msg_;
};

class Init_CheckPath_Request_return_on
{
public:
  explicit Init_CheckPath_Request_return_on(::mbf_msgs::srv::CheckPath_Request & msg)
  : msg_(msg)
  {}
  Init_CheckPath_Request_skip_poses return_on(::mbf_msgs::srv::CheckPath_Request::_return_on_type arg)
  {
    msg_.return_on = std::move(arg);
    return Init_CheckPath_Request_skip_poses(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPath_Request msg_;
};

class Init_CheckPath_Request_costmap
{
public:
  explicit Init_CheckPath_Request_costmap(::mbf_msgs::srv::CheckPath_Request & msg)
  : msg_(msg)
  {}
  Init_CheckPath_Request_return_on costmap(::mbf_msgs::srv::CheckPath_Request::_costmap_type arg)
  {
    msg_.costmap = std::move(arg);
    return Init_CheckPath_Request_return_on(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPath_Request msg_;
};

class Init_CheckPath_Request_unknown_cost_mult
{
public:
  explicit Init_CheckPath_Request_unknown_cost_mult(::mbf_msgs::srv::CheckPath_Request & msg)
  : msg_(msg)
  {}
  Init_CheckPath_Request_costmap unknown_cost_mult(::mbf_msgs::srv::CheckPath_Request::_unknown_cost_mult_type arg)
  {
    msg_.unknown_cost_mult = std::move(arg);
    return Init_CheckPath_Request_costmap(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPath_Request msg_;
};

class Init_CheckPath_Request_inscrib_cost_mult
{
public:
  explicit Init_CheckPath_Request_inscrib_cost_mult(::mbf_msgs::srv::CheckPath_Request & msg)
  : msg_(msg)
  {}
  Init_CheckPath_Request_unknown_cost_mult inscrib_cost_mult(::mbf_msgs::srv::CheckPath_Request::_inscrib_cost_mult_type arg)
  {
    msg_.inscrib_cost_mult = std::move(arg);
    return Init_CheckPath_Request_unknown_cost_mult(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPath_Request msg_;
};

class Init_CheckPath_Request_lethal_cost_mult
{
public:
  explicit Init_CheckPath_Request_lethal_cost_mult(::mbf_msgs::srv::CheckPath_Request & msg)
  : msg_(msg)
  {}
  Init_CheckPath_Request_inscrib_cost_mult lethal_cost_mult(::mbf_msgs::srv::CheckPath_Request::_lethal_cost_mult_type arg)
  {
    msg_.lethal_cost_mult = std::move(arg);
    return Init_CheckPath_Request_inscrib_cost_mult(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPath_Request msg_;
};

class Init_CheckPath_Request_safety_dist
{
public:
  explicit Init_CheckPath_Request_safety_dist(::mbf_msgs::srv::CheckPath_Request & msg)
  : msg_(msg)
  {}
  Init_CheckPath_Request_lethal_cost_mult safety_dist(::mbf_msgs::srv::CheckPath_Request::_safety_dist_type arg)
  {
    msg_.safety_dist = std::move(arg);
    return Init_CheckPath_Request_lethal_cost_mult(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPath_Request msg_;
};

class Init_CheckPath_Request_path
{
public:
  Init_CheckPath_Request_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CheckPath_Request_safety_dist path(::mbf_msgs::srv::CheckPath_Request::_path_type arg)
  {
    msg_.path = std::move(arg);
    return Init_CheckPath_Request_safety_dist(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPath_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::srv::CheckPath_Request>()
{
  return mbf_msgs::srv::builder::Init_CheckPath_Request_path();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace srv
{

namespace builder
{

class Init_CheckPath_Response_cost
{
public:
  explicit Init_CheckPath_Response_cost(::mbf_msgs::srv::CheckPath_Response & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::srv::CheckPath_Response cost(::mbf_msgs::srv::CheckPath_Response::_cost_type arg)
  {
    msg_.cost = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPath_Response msg_;
};

class Init_CheckPath_Response_state
{
public:
  explicit Init_CheckPath_Response_state(::mbf_msgs::srv::CheckPath_Response & msg)
  : msg_(msg)
  {}
  Init_CheckPath_Response_cost state(::mbf_msgs::srv::CheckPath_Response::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_CheckPath_Response_cost(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPath_Response msg_;
};

class Init_CheckPath_Response_last_checked
{
public:
  Init_CheckPath_Response_last_checked()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CheckPath_Response_state last_checked(::mbf_msgs::srv::CheckPath_Response::_last_checked_type arg)
  {
    msg_.last_checked = std::move(arg);
    return Init_CheckPath_Response_state(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPath_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::srv::CheckPath_Response>()
{
  return mbf_msgs::srv::builder::Init_CheckPath_Response_last_checked();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace srv
{

namespace builder
{

class Init_CheckPath_Event_response
{
public:
  explicit Init_CheckPath_Event_response(::mbf_msgs::srv::CheckPath_Event & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::srv::CheckPath_Event response(::mbf_msgs::srv::CheckPath_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPath_Event msg_;
};

class Init_CheckPath_Event_request
{
public:
  explicit Init_CheckPath_Event_request(::mbf_msgs::srv::CheckPath_Event & msg)
  : msg_(msg)
  {}
  Init_CheckPath_Event_response request(::mbf_msgs::srv::CheckPath_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_CheckPath_Event_response(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPath_Event msg_;
};

class Init_CheckPath_Event_info
{
public:
  Init_CheckPath_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CheckPath_Event_request info(::mbf_msgs::srv::CheckPath_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_CheckPath_Event_request(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPath_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::srv::CheckPath_Event>()
{
  return mbf_msgs::srv::builder::Init_CheckPath_Event_info();
}

}  // namespace mbf_msgs

#endif  // MBF_MSGS__SRV__DETAIL__CHECK_PATH__BUILDER_HPP_
