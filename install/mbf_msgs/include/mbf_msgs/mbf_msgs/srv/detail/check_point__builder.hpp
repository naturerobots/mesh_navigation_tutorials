// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mbf_msgs:srv/CheckPoint.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/srv/check_point.hpp"


#ifndef MBF_MSGS__SRV__DETAIL__CHECK_POINT__BUILDER_HPP_
#define MBF_MSGS__SRV__DETAIL__CHECK_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mbf_msgs/srv/detail/check_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mbf_msgs
{

namespace srv
{

namespace builder
{

class Init_CheckPoint_Request_costmap
{
public:
  explicit Init_CheckPoint_Request_costmap(::mbf_msgs::srv::CheckPoint_Request & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::srv::CheckPoint_Request costmap(::mbf_msgs::srv::CheckPoint_Request::_costmap_type arg)
  {
    msg_.costmap = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPoint_Request msg_;
};

class Init_CheckPoint_Request_point
{
public:
  Init_CheckPoint_Request_point()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CheckPoint_Request_costmap point(::mbf_msgs::srv::CheckPoint_Request::_point_type arg)
  {
    msg_.point = std::move(arg);
    return Init_CheckPoint_Request_costmap(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPoint_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::srv::CheckPoint_Request>()
{
  return mbf_msgs::srv::builder::Init_CheckPoint_Request_point();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace srv
{

namespace builder
{

class Init_CheckPoint_Response_cost
{
public:
  explicit Init_CheckPoint_Response_cost(::mbf_msgs::srv::CheckPoint_Response & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::srv::CheckPoint_Response cost(::mbf_msgs::srv::CheckPoint_Response::_cost_type arg)
  {
    msg_.cost = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPoint_Response msg_;
};

class Init_CheckPoint_Response_state
{
public:
  Init_CheckPoint_Response_state()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CheckPoint_Response_cost state(::mbf_msgs::srv::CheckPoint_Response::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_CheckPoint_Response_cost(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPoint_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::srv::CheckPoint_Response>()
{
  return mbf_msgs::srv::builder::Init_CheckPoint_Response_state();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace srv
{

namespace builder
{

class Init_CheckPoint_Event_response
{
public:
  explicit Init_CheckPoint_Event_response(::mbf_msgs::srv::CheckPoint_Event & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::srv::CheckPoint_Event response(::mbf_msgs::srv::CheckPoint_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPoint_Event msg_;
};

class Init_CheckPoint_Event_request
{
public:
  explicit Init_CheckPoint_Event_request(::mbf_msgs::srv::CheckPoint_Event & msg)
  : msg_(msg)
  {}
  Init_CheckPoint_Event_response request(::mbf_msgs::srv::CheckPoint_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_CheckPoint_Event_response(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPoint_Event msg_;
};

class Init_CheckPoint_Event_info
{
public:
  Init_CheckPoint_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CheckPoint_Event_request info(::mbf_msgs::srv::CheckPoint_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_CheckPoint_Event_request(msg_);
  }

private:
  ::mbf_msgs::srv::CheckPoint_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::srv::CheckPoint_Event>()
{
  return mbf_msgs::srv::builder::Init_CheckPoint_Event_info();
}

}  // namespace mbf_msgs

#endif  // MBF_MSGS__SRV__DETAIL__CHECK_POINT__BUILDER_HPP_
