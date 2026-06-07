// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:srv/GetVertexCosts.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/srv/get_vertex_costs.hpp"


#ifndef MESH_MSGS__SRV__DETAIL__GET_VERTEX_COSTS__BUILDER_HPP_
#define MESH_MSGS__SRV__DETAIL__GET_VERTEX_COSTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/srv/detail/get_vertex_costs__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace srv
{

namespace builder
{

class Init_GetVertexCosts_Request_layer
{
public:
  explicit Init_GetVertexCosts_Request_layer(::mesh_msgs::srv::GetVertexCosts_Request & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::srv::GetVertexCosts_Request layer(::mesh_msgs::srv::GetVertexCosts_Request::_layer_type arg)
  {
    msg_.layer = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::srv::GetVertexCosts_Request msg_;
};

class Init_GetVertexCosts_Request_uuid
{
public:
  Init_GetVertexCosts_Request_uuid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetVertexCosts_Request_layer uuid(::mesh_msgs::srv::GetVertexCosts_Request::_uuid_type arg)
  {
    msg_.uuid = std::move(arg);
    return Init_GetVertexCosts_Request_layer(msg_);
  }

private:
  ::mesh_msgs::srv::GetVertexCosts_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::srv::GetVertexCosts_Request>()
{
  return mesh_msgs::srv::builder::Init_GetVertexCosts_Request_uuid();
}

}  // namespace mesh_msgs


namespace mesh_msgs
{

namespace srv
{

namespace builder
{

class Init_GetVertexCosts_Response_mesh_vertex_costs_stamped
{
public:
  Init_GetVertexCosts_Response_mesh_vertex_costs_stamped()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mesh_msgs::srv::GetVertexCosts_Response mesh_vertex_costs_stamped(::mesh_msgs::srv::GetVertexCosts_Response::_mesh_vertex_costs_stamped_type arg)
  {
    msg_.mesh_vertex_costs_stamped = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::srv::GetVertexCosts_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::srv::GetVertexCosts_Response>()
{
  return mesh_msgs::srv::builder::Init_GetVertexCosts_Response_mesh_vertex_costs_stamped();
}

}  // namespace mesh_msgs


namespace mesh_msgs
{

namespace srv
{

namespace builder
{

class Init_GetVertexCosts_Event_response
{
public:
  explicit Init_GetVertexCosts_Event_response(::mesh_msgs::srv::GetVertexCosts_Event & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::srv::GetVertexCosts_Event response(::mesh_msgs::srv::GetVertexCosts_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::srv::GetVertexCosts_Event msg_;
};

class Init_GetVertexCosts_Event_request
{
public:
  explicit Init_GetVertexCosts_Event_request(::mesh_msgs::srv::GetVertexCosts_Event & msg)
  : msg_(msg)
  {}
  Init_GetVertexCosts_Event_response request(::mesh_msgs::srv::GetVertexCosts_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetVertexCosts_Event_response(msg_);
  }

private:
  ::mesh_msgs::srv::GetVertexCosts_Event msg_;
};

class Init_GetVertexCosts_Event_info
{
public:
  Init_GetVertexCosts_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetVertexCosts_Event_request info(::mesh_msgs::srv::GetVertexCosts_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetVertexCosts_Event_request(msg_);
  }

private:
  ::mesh_msgs::srv::GetVertexCosts_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::srv::GetVertexCosts_Event>()
{
  return mesh_msgs::srv::builder::Init_GetVertexCosts_Event_info();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__SRV__DETAIL__GET_VERTEX_COSTS__BUILDER_HPP_
