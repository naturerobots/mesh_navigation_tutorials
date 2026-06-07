// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mbf_msgs:srv/SetTestRobotState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/srv/set_test_robot_state.hpp"


#ifndef MBF_MSGS__SRV__DETAIL__SET_TEST_ROBOT_STATE__BUILDER_HPP_
#define MBF_MSGS__SRV__DETAIL__SET_TEST_ROBOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mbf_msgs/srv/detail/set_test_robot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mbf_msgs
{

namespace srv
{

namespace builder
{

class Init_SetTestRobotState_Request_velocity_robot
{
public:
  explicit Init_SetTestRobotState_Request_velocity_robot(::mbf_msgs::srv::SetTestRobotState_Request & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::srv::SetTestRobotState_Request velocity_robot(::mbf_msgs::srv::SetTestRobotState_Request::_velocity_robot_type arg)
  {
    msg_.velocity_robot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::srv::SetTestRobotState_Request msg_;
};

class Init_SetTestRobotState_Request_set_velocity
{
public:
  explicit Init_SetTestRobotState_Request_set_velocity(::mbf_msgs::srv::SetTestRobotState_Request & msg)
  : msg_(msg)
  {}
  Init_SetTestRobotState_Request_velocity_robot set_velocity(::mbf_msgs::srv::SetTestRobotState_Request::_set_velocity_type arg)
  {
    msg_.set_velocity = std::move(arg);
    return Init_SetTestRobotState_Request_velocity_robot(msg_);
  }

private:
  ::mbf_msgs::srv::SetTestRobotState_Request msg_;
};

class Init_SetTestRobotState_Request_transform
{
public:
  Init_SetTestRobotState_Request_transform()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetTestRobotState_Request_set_velocity transform(::mbf_msgs::srv::SetTestRobotState_Request::_transform_type arg)
  {
    msg_.transform = std::move(arg);
    return Init_SetTestRobotState_Request_set_velocity(msg_);
  }

private:
  ::mbf_msgs::srv::SetTestRobotState_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::srv::SetTestRobotState_Request>()
{
  return mbf_msgs::srv::builder::Init_SetTestRobotState_Request_transform();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace srv
{

namespace builder
{

class Init_SetTestRobotState_Response_success
{
public:
  Init_SetTestRobotState_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mbf_msgs::srv::SetTestRobotState_Response success(::mbf_msgs::srv::SetTestRobotState_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::srv::SetTestRobotState_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::srv::SetTestRobotState_Response>()
{
  return mbf_msgs::srv::builder::Init_SetTestRobotState_Response_success();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace srv
{

namespace builder
{

class Init_SetTestRobotState_Event_response
{
public:
  explicit Init_SetTestRobotState_Event_response(::mbf_msgs::srv::SetTestRobotState_Event & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::srv::SetTestRobotState_Event response(::mbf_msgs::srv::SetTestRobotState_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::srv::SetTestRobotState_Event msg_;
};

class Init_SetTestRobotState_Event_request
{
public:
  explicit Init_SetTestRobotState_Event_request(::mbf_msgs::srv::SetTestRobotState_Event & msg)
  : msg_(msg)
  {}
  Init_SetTestRobotState_Event_response request(::mbf_msgs::srv::SetTestRobotState_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_SetTestRobotState_Event_response(msg_);
  }

private:
  ::mbf_msgs::srv::SetTestRobotState_Event msg_;
};

class Init_SetTestRobotState_Event_info
{
public:
  Init_SetTestRobotState_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetTestRobotState_Event_request info(::mbf_msgs::srv::SetTestRobotState_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_SetTestRobotState_Event_request(msg_);
  }

private:
  ::mbf_msgs::srv::SetTestRobotState_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::srv::SetTestRobotState_Event>()
{
  return mbf_msgs::srv::builder::Init_SetTestRobotState_Event_info();
}

}  // namespace mbf_msgs

#endif  // MBF_MSGS__SRV__DETAIL__SET_TEST_ROBOT_STATE__BUILDER_HPP_
