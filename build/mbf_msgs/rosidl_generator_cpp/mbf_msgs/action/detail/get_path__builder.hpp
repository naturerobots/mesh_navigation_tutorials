// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mbf_msgs:action/GetPath.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/action/get_path.hpp"


#ifndef MBF_MSGS__ACTION__DETAIL__GET_PATH__BUILDER_HPP_
#define MBF_MSGS__ACTION__DETAIL__GET_PATH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mbf_msgs/action/detail/get_path__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_GetPath_Goal_concurrency_slot
{
public:
  explicit Init_GetPath_Goal_concurrency_slot(::mbf_msgs::action::GetPath_Goal & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::GetPath_Goal concurrency_slot(::mbf_msgs::action::GetPath_Goal::_concurrency_slot_type arg)
  {
    msg_.concurrency_slot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_Goal msg_;
};

class Init_GetPath_Goal_planner
{
public:
  explicit Init_GetPath_Goal_planner(::mbf_msgs::action::GetPath_Goal & msg)
  : msg_(msg)
  {}
  Init_GetPath_Goal_concurrency_slot planner(::mbf_msgs::action::GetPath_Goal::_planner_type arg)
  {
    msg_.planner = std::move(arg);
    return Init_GetPath_Goal_concurrency_slot(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_Goal msg_;
};

class Init_GetPath_Goal_tolerance
{
public:
  explicit Init_GetPath_Goal_tolerance(::mbf_msgs::action::GetPath_Goal & msg)
  : msg_(msg)
  {}
  Init_GetPath_Goal_planner tolerance(::mbf_msgs::action::GetPath_Goal::_tolerance_type arg)
  {
    msg_.tolerance = std::move(arg);
    return Init_GetPath_Goal_planner(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_Goal msg_;
};

class Init_GetPath_Goal_target_pose
{
public:
  explicit Init_GetPath_Goal_target_pose(::mbf_msgs::action::GetPath_Goal & msg)
  : msg_(msg)
  {}
  Init_GetPath_Goal_tolerance target_pose(::mbf_msgs::action::GetPath_Goal::_target_pose_type arg)
  {
    msg_.target_pose = std::move(arg);
    return Init_GetPath_Goal_tolerance(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_Goal msg_;
};

class Init_GetPath_Goal_start_pose
{
public:
  explicit Init_GetPath_Goal_start_pose(::mbf_msgs::action::GetPath_Goal & msg)
  : msg_(msg)
  {}
  Init_GetPath_Goal_target_pose start_pose(::mbf_msgs::action::GetPath_Goal::_start_pose_type arg)
  {
    msg_.start_pose = std::move(arg);
    return Init_GetPath_Goal_target_pose(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_Goal msg_;
};

class Init_GetPath_Goal_use_start_pose
{
public:
  Init_GetPath_Goal_use_start_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetPath_Goal_start_pose use_start_pose(::mbf_msgs::action::GetPath_Goal::_use_start_pose_type arg)
  {
    msg_.use_start_pose = std::move(arg);
    return Init_GetPath_Goal_start_pose(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::GetPath_Goal>()
{
  return mbf_msgs::action::builder::Init_GetPath_Goal_use_start_pose();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_GetPath_Result_cost
{
public:
  explicit Init_GetPath_Result_cost(::mbf_msgs::action::GetPath_Result & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::GetPath_Result cost(::mbf_msgs::action::GetPath_Result::_cost_type arg)
  {
    msg_.cost = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_Result msg_;
};

class Init_GetPath_Result_path
{
public:
  explicit Init_GetPath_Result_path(::mbf_msgs::action::GetPath_Result & msg)
  : msg_(msg)
  {}
  Init_GetPath_Result_cost path(::mbf_msgs::action::GetPath_Result::_path_type arg)
  {
    msg_.path = std::move(arg);
    return Init_GetPath_Result_cost(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_Result msg_;
};

class Init_GetPath_Result_message
{
public:
  explicit Init_GetPath_Result_message(::mbf_msgs::action::GetPath_Result & msg)
  : msg_(msg)
  {}
  Init_GetPath_Result_path message(::mbf_msgs::action::GetPath_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_GetPath_Result_path(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_Result msg_;
};

class Init_GetPath_Result_outcome
{
public:
  Init_GetPath_Result_outcome()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetPath_Result_message outcome(::mbf_msgs::action::GetPath_Result::_outcome_type arg)
  {
    msg_.outcome = std::move(arg);
    return Init_GetPath_Result_message(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::GetPath_Result>()
{
  return mbf_msgs::action::builder::Init_GetPath_Result_outcome();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{


}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::GetPath_Feedback>()
{
  return ::mbf_msgs::action::GetPath_Feedback(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_GetPath_SendGoal_Request_goal
{
public:
  explicit Init_GetPath_SendGoal_Request_goal(::mbf_msgs::action::GetPath_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::GetPath_SendGoal_Request goal(::mbf_msgs::action::GetPath_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_SendGoal_Request msg_;
};

class Init_GetPath_SendGoal_Request_goal_id
{
public:
  Init_GetPath_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetPath_SendGoal_Request_goal goal_id(::mbf_msgs::action::GetPath_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_GetPath_SendGoal_Request_goal(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::GetPath_SendGoal_Request>()
{
  return mbf_msgs::action::builder::Init_GetPath_SendGoal_Request_goal_id();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_GetPath_SendGoal_Response_stamp
{
public:
  explicit Init_GetPath_SendGoal_Response_stamp(::mbf_msgs::action::GetPath_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::GetPath_SendGoal_Response stamp(::mbf_msgs::action::GetPath_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_SendGoal_Response msg_;
};

class Init_GetPath_SendGoal_Response_accepted
{
public:
  Init_GetPath_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetPath_SendGoal_Response_stamp accepted(::mbf_msgs::action::GetPath_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_GetPath_SendGoal_Response_stamp(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::GetPath_SendGoal_Response>()
{
  return mbf_msgs::action::builder::Init_GetPath_SendGoal_Response_accepted();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_GetPath_SendGoal_Event_response
{
public:
  explicit Init_GetPath_SendGoal_Event_response(::mbf_msgs::action::GetPath_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::GetPath_SendGoal_Event response(::mbf_msgs::action::GetPath_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_SendGoal_Event msg_;
};

class Init_GetPath_SendGoal_Event_request
{
public:
  explicit Init_GetPath_SendGoal_Event_request(::mbf_msgs::action::GetPath_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_GetPath_SendGoal_Event_response request(::mbf_msgs::action::GetPath_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetPath_SendGoal_Event_response(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_SendGoal_Event msg_;
};

class Init_GetPath_SendGoal_Event_info
{
public:
  Init_GetPath_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetPath_SendGoal_Event_request info(::mbf_msgs::action::GetPath_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetPath_SendGoal_Event_request(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::GetPath_SendGoal_Event>()
{
  return mbf_msgs::action::builder::Init_GetPath_SendGoal_Event_info();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_GetPath_GetResult_Request_goal_id
{
public:
  Init_GetPath_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mbf_msgs::action::GetPath_GetResult_Request goal_id(::mbf_msgs::action::GetPath_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::GetPath_GetResult_Request>()
{
  return mbf_msgs::action::builder::Init_GetPath_GetResult_Request_goal_id();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_GetPath_GetResult_Response_result
{
public:
  explicit Init_GetPath_GetResult_Response_result(::mbf_msgs::action::GetPath_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::GetPath_GetResult_Response result(::mbf_msgs::action::GetPath_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_GetResult_Response msg_;
};

class Init_GetPath_GetResult_Response_status
{
public:
  Init_GetPath_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetPath_GetResult_Response_result status(::mbf_msgs::action::GetPath_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_GetPath_GetResult_Response_result(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::GetPath_GetResult_Response>()
{
  return mbf_msgs::action::builder::Init_GetPath_GetResult_Response_status();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_GetPath_GetResult_Event_response
{
public:
  explicit Init_GetPath_GetResult_Event_response(::mbf_msgs::action::GetPath_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::GetPath_GetResult_Event response(::mbf_msgs::action::GetPath_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_GetResult_Event msg_;
};

class Init_GetPath_GetResult_Event_request
{
public:
  explicit Init_GetPath_GetResult_Event_request(::mbf_msgs::action::GetPath_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_GetPath_GetResult_Event_response request(::mbf_msgs::action::GetPath_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_GetPath_GetResult_Event_response(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_GetResult_Event msg_;
};

class Init_GetPath_GetResult_Event_info
{
public:
  Init_GetPath_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetPath_GetResult_Event_request info(::mbf_msgs::action::GetPath_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_GetPath_GetResult_Event_request(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::GetPath_GetResult_Event>()
{
  return mbf_msgs::action::builder::Init_GetPath_GetResult_Event_info();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_GetPath_FeedbackMessage_feedback
{
public:
  explicit Init_GetPath_FeedbackMessage_feedback(::mbf_msgs::action::GetPath_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::GetPath_FeedbackMessage feedback(::mbf_msgs::action::GetPath_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_FeedbackMessage msg_;
};

class Init_GetPath_FeedbackMessage_goal_id
{
public:
  Init_GetPath_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetPath_FeedbackMessage_feedback goal_id(::mbf_msgs::action::GetPath_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_GetPath_FeedbackMessage_feedback(msg_);
  }

private:
  ::mbf_msgs::action::GetPath_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::GetPath_FeedbackMessage>()
{
  return mbf_msgs::action::builder::Init_GetPath_FeedbackMessage_goal_id();
}

}  // namespace mbf_msgs

#endif  // MBF_MSGS__ACTION__DETAIL__GET_PATH__BUILDER_HPP_
