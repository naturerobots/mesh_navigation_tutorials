// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:srv/GetGeometry.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/srv/get_geometry.hpp"


#ifndef MESH_MSGS__SRV__DETAIL__GET_GEOMETRY__BUILDER_HPP_
#define MESH_MSGS__SRV__DETAIL__GET_GEOMETRY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/srv/detail/get_geometry__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace srv
{

namespace builder
{

class Init_GetGeometry_Request_uuid
{
public:
  Init_GetGeometry_Request_uuid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mesh_msgs::srv::GetGeometry_Request uuid(::mesh_msgs::srv::GetGeometry_Request::_uuid_type arg)
  {
    msg_.uuid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::srv::GetGeometry_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::srv::GetGeometry_Request>()
{
  return mesh_msgs::srv::builder::Init_GetGeometry_Request_uuid();
}

}  // namespace mesh_msgs


namespace mesh_msgs
{

namespace srv
{

namespace builder
{

class Init_GetGeometry_Response_mesh_geometry_stamped
{
public:
  Init_GetGeometry_Response_mesh_geometry_stamped()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mesh_msgs::srv::GetGeometry_Response mesh_geometry_stamped(::mesh_msgs::srv::GetGeometry_Response::_mesh_geometry_stamped_type arg)
  {
    msg_.mesh_geometry_stamped = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::srv::GetGeometry_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::srv::GetGeometry_Response>()
{
  return mesh_msgs::srv::builder::Init_GetGeometry_Response_mesh_geometry_stamped();
}

}  // namespace mesh_msgs


namespace mesh_msgs
{

namespace srv
{

namespace builder
{

class Init_GetGeometry_Event_response
{
public:
  explicit Init_GetGeometry_Event_response(::mesh_msgs::srv::GetGeometry_Event & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::srv::GetGeometry_Event response(::mesh_msgs::srv::GetGeometry_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::srv::GetGeometry_Event msg_;
};

class Init_GetGeometry_Event_request
{
public:
  explicit Init_GetGeometry_Event_request(::mesh_msgs::srv::GetGeometry_Event & msg)
  : msg_(msg)
  {}
  Init_GetGeometry_Event_response request(::mesh_msgs::srv::GetGeometry_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetGeometry_Event_response(msg_);
  }

private:
  ::mesh_msgs::srv::GetGeometry_Event msg_;
};

class Init_GetGeometry_Event_info
{
public:
  Init_GetGeometry_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetGeometry_Event_request info(::mesh_msgs::srv::GetGeometry_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetGeometry_Event_request(msg_);
  }

private:
  ::mesh_msgs::srv::GetGeometry_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::srv::GetGeometry_Event>()
{
  return mesh_msgs::srv::builder::Init_GetGeometry_Event_info();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__SRV__DETAIL__GET_GEOMETRY__BUILDER_HPP_
