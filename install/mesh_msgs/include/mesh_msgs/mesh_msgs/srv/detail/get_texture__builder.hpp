// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:srv/GetTexture.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/srv/get_texture.hpp"


#ifndef MESH_MSGS__SRV__DETAIL__GET_TEXTURE__BUILDER_HPP_
#define MESH_MSGS__SRV__DETAIL__GET_TEXTURE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/srv/detail/get_texture__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace srv
{

namespace builder
{

class Init_GetTexture_Request_texture_index
{
public:
  explicit Init_GetTexture_Request_texture_index(::mesh_msgs::srv::GetTexture_Request & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::srv::GetTexture_Request texture_index(::mesh_msgs::srv::GetTexture_Request::_texture_index_type arg)
  {
    msg_.texture_index = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::srv::GetTexture_Request msg_;
};

class Init_GetTexture_Request_uuid
{
public:
  Init_GetTexture_Request_uuid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetTexture_Request_texture_index uuid(::mesh_msgs::srv::GetTexture_Request::_uuid_type arg)
  {
    msg_.uuid = std::move(arg);
    return Init_GetTexture_Request_texture_index(msg_);
  }

private:
  ::mesh_msgs::srv::GetTexture_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::srv::GetTexture_Request>()
{
  return mesh_msgs::srv::builder::Init_GetTexture_Request_uuid();
}

}  // namespace mesh_msgs


namespace mesh_msgs
{

namespace srv
{

namespace builder
{

class Init_GetTexture_Response_texture
{
public:
  Init_GetTexture_Response_texture()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mesh_msgs::srv::GetTexture_Response texture(::mesh_msgs::srv::GetTexture_Response::_texture_type arg)
  {
    msg_.texture = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::srv::GetTexture_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::srv::GetTexture_Response>()
{
  return mesh_msgs::srv::builder::Init_GetTexture_Response_texture();
}

}  // namespace mesh_msgs


namespace mesh_msgs
{

namespace srv
{

namespace builder
{

class Init_GetTexture_Event_response
{
public:
  explicit Init_GetTexture_Event_response(::mesh_msgs::srv::GetTexture_Event & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::srv::GetTexture_Event response(::mesh_msgs::srv::GetTexture_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::srv::GetTexture_Event msg_;
};

class Init_GetTexture_Event_request
{
public:
  explicit Init_GetTexture_Event_request(::mesh_msgs::srv::GetTexture_Event & msg)
  : msg_(msg)
  {}
  Init_GetTexture_Event_response request(::mesh_msgs::srv::GetTexture_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetTexture_Event_response(msg_);
  }

private:
  ::mesh_msgs::srv::GetTexture_Event msg_;
};

class Init_GetTexture_Event_info
{
public:
  Init_GetTexture_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetTexture_Event_request info(::mesh_msgs::srv::GetTexture_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetTexture_Event_request(msg_);
  }

private:
  ::mesh_msgs::srv::GetTexture_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::srv::GetTexture_Event>()
{
  return mesh_msgs::srv::builder::Init_GetTexture_Event_info();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__SRV__DETAIL__GET_TEXTURE__BUILDER_HPP_
