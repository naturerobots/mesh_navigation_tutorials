// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:msg/VectorFieldStamped.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/vector_field_stamped.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__VECTOR_FIELD_STAMPED__BUILDER_HPP_
#define MESH_MSGS__MSG__DETAIL__VECTOR_FIELD_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/msg/detail/vector_field_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace msg
{

namespace builder
{

class Init_VectorFieldStamped_vector_field
{
public:
  explicit Init_VectorFieldStamped_vector_field(::mesh_msgs::msg::VectorFieldStamped & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::msg::VectorFieldStamped vector_field(::mesh_msgs::msg::VectorFieldStamped::_vector_field_type arg)
  {
    msg_.vector_field = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::msg::VectorFieldStamped msg_;
};

class Init_VectorFieldStamped_header
{
public:
  Init_VectorFieldStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VectorFieldStamped_vector_field header(::mesh_msgs::msg::VectorFieldStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VectorFieldStamped_vector_field(msg_);
  }

private:
  ::mesh_msgs::msg::VectorFieldStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::msg::VectorFieldStamped>()
{
  return mesh_msgs::msg::builder::Init_VectorFieldStamped_header();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__VECTOR_FIELD_STAMPED__BUILDER_HPP_
