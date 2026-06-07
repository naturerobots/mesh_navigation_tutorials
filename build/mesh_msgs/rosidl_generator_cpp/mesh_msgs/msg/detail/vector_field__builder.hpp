// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:msg/VectorField.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/msg/vector_field.hpp"


#ifndef MESH_MSGS__MSG__DETAIL__VECTOR_FIELD__BUILDER_HPP_
#define MESH_MSGS__MSG__DETAIL__VECTOR_FIELD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/msg/detail/vector_field__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace msg
{

namespace builder
{

class Init_VectorField_vectors
{
public:
  explicit Init_VectorField_vectors(::mesh_msgs::msg::VectorField & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::msg::VectorField vectors(::mesh_msgs::msg::VectorField::_vectors_type arg)
  {
    msg_.vectors = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::msg::VectorField msg_;
};

class Init_VectorField_positions
{
public:
  Init_VectorField_positions()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VectorField_vectors positions(::mesh_msgs::msg::VectorField::_positions_type arg)
  {
    msg_.positions = std::move(arg);
    return Init_VectorField_vectors(msg_);
  }

private:
  ::mesh_msgs::msg::VectorField msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::msg::VectorField>()
{
  return mesh_msgs::msg::builder::Init_VectorField_positions();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__MSG__DETAIL__VECTOR_FIELD__BUILDER_HPP_
