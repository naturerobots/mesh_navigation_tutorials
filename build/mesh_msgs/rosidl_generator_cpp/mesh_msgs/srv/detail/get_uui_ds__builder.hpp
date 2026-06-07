// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:srv/GetUUIDs.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/srv/get_uui_ds.hpp"


#ifndef MESH_MSGS__SRV__DETAIL__GET_UUI_DS__BUILDER_HPP_
#define MESH_MSGS__SRV__DETAIL__GET_UUI_DS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/srv/detail/get_uui_ds__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::srv::GetUUIDs_Request>()
{
  return ::mesh_msgs::srv::GetUUIDs_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace mesh_msgs


namespace mesh_msgs
{

namespace srv
{

namespace builder
{

class Init_GetUUIDs_Response_uuids
{
public:
  Init_GetUUIDs_Response_uuids()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mesh_msgs::srv::GetUUIDs_Response uuids(::mesh_msgs::srv::GetUUIDs_Response::_uuids_type arg)
  {
    msg_.uuids = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::srv::GetUUIDs_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::srv::GetUUIDs_Response>()
{
  return mesh_msgs::srv::builder::Init_GetUUIDs_Response_uuids();
}

}  // namespace mesh_msgs


namespace mesh_msgs
{

namespace srv
{

namespace builder
{

class Init_GetUUIDs_Event_response
{
public:
  explicit Init_GetUUIDs_Event_response(::mesh_msgs::srv::GetUUIDs_Event & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::srv::GetUUIDs_Event response(::mesh_msgs::srv::GetUUIDs_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::srv::GetUUIDs_Event msg_;
};

class Init_GetUUIDs_Event_request
{
public:
  explicit Init_GetUUIDs_Event_request(::mesh_msgs::srv::GetUUIDs_Event & msg)
  : msg_(msg)
  {}
  Init_GetUUIDs_Event_response request(::mesh_msgs::srv::GetUUIDs_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetUUIDs_Event_response(msg_);
  }

private:
  ::mesh_msgs::srv::GetUUIDs_Event msg_;
};

class Init_GetUUIDs_Event_info
{
public:
  Init_GetUUIDs_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetUUIDs_Event_request info(::mesh_msgs::srv::GetUUIDs_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetUUIDs_Event_request(msg_);
  }

private:
  ::mesh_msgs::srv::GetUUIDs_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::srv::GetUUIDs_Event>()
{
  return mesh_msgs::srv::builder::Init_GetUUIDs_Event_info();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__SRV__DETAIL__GET_UUI_DS__BUILDER_HPP_
