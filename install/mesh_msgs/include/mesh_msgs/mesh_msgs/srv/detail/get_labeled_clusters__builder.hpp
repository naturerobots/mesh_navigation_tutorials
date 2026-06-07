// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mesh_msgs:srv/GetLabeledClusters.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mesh_msgs/srv/get_labeled_clusters.hpp"


#ifndef MESH_MSGS__SRV__DETAIL__GET_LABELED_CLUSTERS__BUILDER_HPP_
#define MESH_MSGS__SRV__DETAIL__GET_LABELED_CLUSTERS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mesh_msgs/srv/detail/get_labeled_clusters__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mesh_msgs
{

namespace srv
{

namespace builder
{

class Init_GetLabeledClusters_Request_uuid
{
public:
  Init_GetLabeledClusters_Request_uuid()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mesh_msgs::srv::GetLabeledClusters_Request uuid(::mesh_msgs::srv::GetLabeledClusters_Request::_uuid_type arg)
  {
    msg_.uuid = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::srv::GetLabeledClusters_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::srv::GetLabeledClusters_Request>()
{
  return mesh_msgs::srv::builder::Init_GetLabeledClusters_Request_uuid();
}

}  // namespace mesh_msgs


namespace mesh_msgs
{

namespace srv
{

namespace builder
{

class Init_GetLabeledClusters_Response_clusters
{
public:
  Init_GetLabeledClusters_Response_clusters()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mesh_msgs::srv::GetLabeledClusters_Response clusters(::mesh_msgs::srv::GetLabeledClusters_Response::_clusters_type arg)
  {
    msg_.clusters = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::srv::GetLabeledClusters_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::srv::GetLabeledClusters_Response>()
{
  return mesh_msgs::srv::builder::Init_GetLabeledClusters_Response_clusters();
}

}  // namespace mesh_msgs


namespace mesh_msgs
{

namespace srv
{

namespace builder
{

class Init_GetLabeledClusters_Event_response
{
public:
  explicit Init_GetLabeledClusters_Event_response(::mesh_msgs::srv::GetLabeledClusters_Event & msg)
  : msg_(msg)
  {}
  ::mesh_msgs::srv::GetLabeledClusters_Event response(::mesh_msgs::srv::GetLabeledClusters_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mesh_msgs::srv::GetLabeledClusters_Event msg_;
};

class Init_GetLabeledClusters_Event_request
{
public:
  explicit Init_GetLabeledClusters_Event_request(::mesh_msgs::srv::GetLabeledClusters_Event & msg)
  : msg_(msg)
  {}
  Init_GetLabeledClusters_Event_response request(::mesh_msgs::srv::GetLabeledClusters_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetLabeledClusters_Event_response(msg_);
  }

private:
  ::mesh_msgs::srv::GetLabeledClusters_Event msg_;
};

class Init_GetLabeledClusters_Event_info
{
public:
  Init_GetLabeledClusters_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetLabeledClusters_Event_request info(::mesh_msgs::srv::GetLabeledClusters_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetLabeledClusters_Event_request(msg_);
  }

private:
  ::mesh_msgs::srv::GetLabeledClusters_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::mesh_msgs::srv::GetLabeledClusters_Event>()
{
  return mesh_msgs::srv::builder::Init_GetLabeledClusters_Event_info();
}

}  // namespace mesh_msgs

#endif  // MESH_MSGS__SRV__DETAIL__GET_LABELED_CLUSTERS__BUILDER_HPP_
