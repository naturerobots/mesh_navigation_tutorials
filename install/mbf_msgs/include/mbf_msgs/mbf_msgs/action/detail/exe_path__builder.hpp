// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mbf_msgs:action/ExePath.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/action/exe_path.hpp"


#ifndef MBF_MSGS__ACTION__DETAIL__EXE_PATH__BUILDER_HPP_
#define MBF_MSGS__ACTION__DETAIL__EXE_PATH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mbf_msgs/action/detail/exe_path__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_ExePath_Goal_angle_tolerance
{
public:
  explicit Init_ExePath_Goal_angle_tolerance(::mbf_msgs::action::ExePath_Goal & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::ExePath_Goal angle_tolerance(::mbf_msgs::action::ExePath_Goal::_angle_tolerance_type arg)
  {
    msg_.angle_tolerance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_Goal msg_;
};

class Init_ExePath_Goal_dist_tolerance
{
public:
  explicit Init_ExePath_Goal_dist_tolerance(::mbf_msgs::action::ExePath_Goal & msg)
  : msg_(msg)
  {}
  Init_ExePath_Goal_angle_tolerance dist_tolerance(::mbf_msgs::action::ExePath_Goal::_dist_tolerance_type arg)
  {
    msg_.dist_tolerance = std::move(arg);
    return Init_ExePath_Goal_angle_tolerance(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_Goal msg_;
};

class Init_ExePath_Goal_tolerance_from_action
{
public:
  explicit Init_ExePath_Goal_tolerance_from_action(::mbf_msgs::action::ExePath_Goal & msg)
  : msg_(msg)
  {}
  Init_ExePath_Goal_dist_tolerance tolerance_from_action(::mbf_msgs::action::ExePath_Goal::_tolerance_from_action_type arg)
  {
    msg_.tolerance_from_action = std::move(arg);
    return Init_ExePath_Goal_dist_tolerance(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_Goal msg_;
};

class Init_ExePath_Goal_concurrency_slot
{
public:
  explicit Init_ExePath_Goal_concurrency_slot(::mbf_msgs::action::ExePath_Goal & msg)
  : msg_(msg)
  {}
  Init_ExePath_Goal_tolerance_from_action concurrency_slot(::mbf_msgs::action::ExePath_Goal::_concurrency_slot_type arg)
  {
    msg_.concurrency_slot = std::move(arg);
    return Init_ExePath_Goal_tolerance_from_action(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_Goal msg_;
};

class Init_ExePath_Goal_controller
{
public:
  explicit Init_ExePath_Goal_controller(::mbf_msgs::action::ExePath_Goal & msg)
  : msg_(msg)
  {}
  Init_ExePath_Goal_concurrency_slot controller(::mbf_msgs::action::ExePath_Goal::_controller_type arg)
  {
    msg_.controller = std::move(arg);
    return Init_ExePath_Goal_concurrency_slot(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_Goal msg_;
};

class Init_ExePath_Goal_path
{
public:
  Init_ExePath_Goal_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExePath_Goal_controller path(::mbf_msgs::action::ExePath_Goal::_path_type arg)
  {
    msg_.path = std::move(arg);
    return Init_ExePath_Goal_controller(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::ExePath_Goal>()
{
  return mbf_msgs::action::builder::Init_ExePath_Goal_path();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_ExePath_Result_angle_to_goal
{
public:
  explicit Init_ExePath_Result_angle_to_goal(::mbf_msgs::action::ExePath_Result & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::ExePath_Result angle_to_goal(::mbf_msgs::action::ExePath_Result::_angle_to_goal_type arg)
  {
    msg_.angle_to_goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_Result msg_;
};

class Init_ExePath_Result_dist_to_goal
{
public:
  explicit Init_ExePath_Result_dist_to_goal(::mbf_msgs::action::ExePath_Result & msg)
  : msg_(msg)
  {}
  Init_ExePath_Result_angle_to_goal dist_to_goal(::mbf_msgs::action::ExePath_Result::_dist_to_goal_type arg)
  {
    msg_.dist_to_goal = std::move(arg);
    return Init_ExePath_Result_angle_to_goal(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_Result msg_;
};

class Init_ExePath_Result_final_pose
{
public:
  explicit Init_ExePath_Result_final_pose(::mbf_msgs::action::ExePath_Result & msg)
  : msg_(msg)
  {}
  Init_ExePath_Result_dist_to_goal final_pose(::mbf_msgs::action::ExePath_Result::_final_pose_type arg)
  {
    msg_.final_pose = std::move(arg);
    return Init_ExePath_Result_dist_to_goal(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_Result msg_;
};

class Init_ExePath_Result_message
{
public:
  explicit Init_ExePath_Result_message(::mbf_msgs::action::ExePath_Result & msg)
  : msg_(msg)
  {}
  Init_ExePath_Result_final_pose message(::mbf_msgs::action::ExePath_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_ExePath_Result_final_pose(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_Result msg_;
};

class Init_ExePath_Result_outcome
{
public:
  Init_ExePath_Result_outcome()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExePath_Result_message outcome(::mbf_msgs::action::ExePath_Result::_outcome_type arg)
  {
    msg_.outcome = std::move(arg);
    return Init_ExePath_Result_message(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::ExePath_Result>()
{
  return mbf_msgs::action::builder::Init_ExePath_Result_outcome();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_ExePath_Feedback_last_cmd_vel
{
public:
  explicit Init_ExePath_Feedback_last_cmd_vel(::mbf_msgs::action::ExePath_Feedback & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::ExePath_Feedback last_cmd_vel(::mbf_msgs::action::ExePath_Feedback::_last_cmd_vel_type arg)
  {
    msg_.last_cmd_vel = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_Feedback msg_;
};

class Init_ExePath_Feedback_current_pose
{
public:
  explicit Init_ExePath_Feedback_current_pose(::mbf_msgs::action::ExePath_Feedback & msg)
  : msg_(msg)
  {}
  Init_ExePath_Feedback_last_cmd_vel current_pose(::mbf_msgs::action::ExePath_Feedback::_current_pose_type arg)
  {
    msg_.current_pose = std::move(arg);
    return Init_ExePath_Feedback_last_cmd_vel(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_Feedback msg_;
};

class Init_ExePath_Feedback_angle_to_goal
{
public:
  explicit Init_ExePath_Feedback_angle_to_goal(::mbf_msgs::action::ExePath_Feedback & msg)
  : msg_(msg)
  {}
  Init_ExePath_Feedback_current_pose angle_to_goal(::mbf_msgs::action::ExePath_Feedback::_angle_to_goal_type arg)
  {
    msg_.angle_to_goal = std::move(arg);
    return Init_ExePath_Feedback_current_pose(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_Feedback msg_;
};

class Init_ExePath_Feedback_dist_to_goal
{
public:
  explicit Init_ExePath_Feedback_dist_to_goal(::mbf_msgs::action::ExePath_Feedback & msg)
  : msg_(msg)
  {}
  Init_ExePath_Feedback_angle_to_goal dist_to_goal(::mbf_msgs::action::ExePath_Feedback::_dist_to_goal_type arg)
  {
    msg_.dist_to_goal = std::move(arg);
    return Init_ExePath_Feedback_angle_to_goal(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_Feedback msg_;
};

class Init_ExePath_Feedback_message
{
public:
  explicit Init_ExePath_Feedback_message(::mbf_msgs::action::ExePath_Feedback & msg)
  : msg_(msg)
  {}
  Init_ExePath_Feedback_dist_to_goal message(::mbf_msgs::action::ExePath_Feedback::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_ExePath_Feedback_dist_to_goal(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_Feedback msg_;
};

class Init_ExePath_Feedback_outcome
{
public:
  Init_ExePath_Feedback_outcome()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExePath_Feedback_message outcome(::mbf_msgs::action::ExePath_Feedback::_outcome_type arg)
  {
    msg_.outcome = std::move(arg);
    return Init_ExePath_Feedback_message(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::ExePath_Feedback>()
{
  return mbf_msgs::action::builder::Init_ExePath_Feedback_outcome();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_ExePath_SendGoal_Request_goal
{
public:
  explicit Init_ExePath_SendGoal_Request_goal(::mbf_msgs::action::ExePath_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::ExePath_SendGoal_Request goal(::mbf_msgs::action::ExePath_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_SendGoal_Request msg_;
};

class Init_ExePath_SendGoal_Request_goal_id
{
public:
  Init_ExePath_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExePath_SendGoal_Request_goal goal_id(::mbf_msgs::action::ExePath_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ExePath_SendGoal_Request_goal(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::ExePath_SendGoal_Request>()
{
  return mbf_msgs::action::builder::Init_ExePath_SendGoal_Request_goal_id();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_ExePath_SendGoal_Response_stamp
{
public:
  explicit Init_ExePath_SendGoal_Response_stamp(::mbf_msgs::action::ExePath_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::ExePath_SendGoal_Response stamp(::mbf_msgs::action::ExePath_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_SendGoal_Response msg_;
};

class Init_ExePath_SendGoal_Response_accepted
{
public:
  Init_ExePath_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExePath_SendGoal_Response_stamp accepted(::mbf_msgs::action::ExePath_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_ExePath_SendGoal_Response_stamp(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::ExePath_SendGoal_Response>()
{
  return mbf_msgs::action::builder::Init_ExePath_SendGoal_Response_accepted();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_ExePath_SendGoal_Event_response
{
public:
  explicit Init_ExePath_SendGoal_Event_response(::mbf_msgs::action::ExePath_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::ExePath_SendGoal_Event response(::mbf_msgs::action::ExePath_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_SendGoal_Event msg_;
};

class Init_ExePath_SendGoal_Event_request
{
public:
  explicit Init_ExePath_SendGoal_Event_request(::mbf_msgs::action::ExePath_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_ExePath_SendGoal_Event_response request(::mbf_msgs::action::ExePath_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ExePath_SendGoal_Event_response(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_SendGoal_Event msg_;
};

class Init_ExePath_SendGoal_Event_info
{
public:
  Init_ExePath_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExePath_SendGoal_Event_request info(::mbf_msgs::action::ExePath_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ExePath_SendGoal_Event_request(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::ExePath_SendGoal_Event>()
{
  return mbf_msgs::action::builder::Init_ExePath_SendGoal_Event_info();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_ExePath_GetResult_Request_goal_id
{
public:
  Init_ExePath_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mbf_msgs::action::ExePath_GetResult_Request goal_id(::mbf_msgs::action::ExePath_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::ExePath_GetResult_Request>()
{
  return mbf_msgs::action::builder::Init_ExePath_GetResult_Request_goal_id();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_ExePath_GetResult_Response_result
{
public:
  explicit Init_ExePath_GetResult_Response_result(::mbf_msgs::action::ExePath_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::ExePath_GetResult_Response result(::mbf_msgs::action::ExePath_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_GetResult_Response msg_;
};

class Init_ExePath_GetResult_Response_status
{
public:
  Init_ExePath_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExePath_GetResult_Response_result status(::mbf_msgs::action::ExePath_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_ExePath_GetResult_Response_result(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::ExePath_GetResult_Response>()
{
  return mbf_msgs::action::builder::Init_ExePath_GetResult_Response_status();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_ExePath_GetResult_Event_response
{
public:
  explicit Init_ExePath_GetResult_Event_response(::mbf_msgs::action::ExePath_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::ExePath_GetResult_Event response(::mbf_msgs::action::ExePath_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_GetResult_Event msg_;
};

class Init_ExePath_GetResult_Event_request
{
public:
  explicit Init_ExePath_GetResult_Event_request(::mbf_msgs::action::ExePath_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_ExePath_GetResult_Event_response request(::mbf_msgs::action::ExePath_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_ExePath_GetResult_Event_response(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_GetResult_Event msg_;
};

class Init_ExePath_GetResult_Event_info
{
public:
  Init_ExePath_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExePath_GetResult_Event_request info(::mbf_msgs::action::ExePath_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_ExePath_GetResult_Event_request(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::ExePath_GetResult_Event>()
{
  return mbf_msgs::action::builder::Init_ExePath_GetResult_Event_info();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_ExePath_FeedbackMessage_feedback
{
public:
  explicit Init_ExePath_FeedbackMessage_feedback(::mbf_msgs::action::ExePath_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::ExePath_FeedbackMessage feedback(::mbf_msgs::action::ExePath_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_FeedbackMessage msg_;
};

class Init_ExePath_FeedbackMessage_goal_id
{
public:
  Init_ExePath_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ExePath_FeedbackMessage_feedback goal_id(::mbf_msgs::action::ExePath_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_ExePath_FeedbackMessage_feedback(msg_);
  }

private:
  ::mbf_msgs::action::ExePath_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::ExePath_FeedbackMessage>()
{
  return mbf_msgs::action::builder::Init_ExePath_FeedbackMessage_goal_id();
}

}  // namespace mbf_msgs

#endif  // MBF_MSGS__ACTION__DETAIL__EXE_PATH__BUILDER_HPP_
