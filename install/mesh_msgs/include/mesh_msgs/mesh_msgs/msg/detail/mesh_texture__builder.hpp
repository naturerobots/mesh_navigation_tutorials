// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:msg/MeshTexture.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/mesh_texture.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__MESH_TEXTURE__BUILDER_HPP_
#define MESH_MSGS__MSG__DETAIL__MESH_TEXTURE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/msg/detail/mesh_texture__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace msg
{

namespace builder
{

class Init_MeshTexture_image
{
public:
  explicit Init_MeshTexture_image(::mesh_msgs::msg::MeshTexture & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::msg::MeshTexture image(::mesh_msgs::msg::MeshTexture::_image_type arg)
  {
    msg_.image = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::msg::MeshTexture msg_;
};

class Init_MeshTexture_texture_index
{
public:
  explicit Init_MeshTexture_texture_index(::mesh_msgs::msg::MeshTexture & msg)
  : msg_(msg)
  {}
  Init_MeshTexture_image texture_index(::mesh_msgs::msg::MeshTexture::_texture_index_type arg)
  {
    msg_.texture_index = std::move(arg);
    return Init_MeshTexture_image(msg_);
  }

private:
  ::mesh_msgs::msg::MeshTexture msg_;
};

class Init_MeshTexture_uuid
{
public:
  Init_MeshTexture_uuid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MeshTexture_texture_index uuid(::mesh_msgs::msg::MeshTexture::_uuid_type arg)
  {
    msg_.uuid = std::move(arg);
    return Init_MeshTexture_texture_index(msg_);
  }

private:
  ::mesh_msgs::msg::MeshTexture msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::msg::MeshTexture>()
{
  return mesh_msgs::msg::builder::Init_MeshTexture_uuid();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__MESH_TEXTURE__BUILDER_HPP_
