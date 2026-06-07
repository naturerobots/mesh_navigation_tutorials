// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mbf_msgs:action/MoveBase.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/action/move_base.hpp"


#ifndef MBF_MSGS__ACTION__DETAIL__MOVE_BASE__BUILDER_HPP_
#define MBF_MSGS__ACTION__DETAIL__MOVE_BASE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mbf_msgs/action/detail/move_base__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_MoveBase_Goal_recovery_behaviors
{
public:
  explicit Init_MoveBase_Goal_recovery_behaviors(::mbf_msgs::action::MoveBase_Goal & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::MoveBase_Goal recovery_behaviors(::mbf_msgs::action::MoveBase_Goal::_recovery_behaviors_type arg)
  {
    msg_.recovery_behaviors = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_Goal msg_;
};

class Init_MoveBase_Goal_planner
{
public:
  explicit Init_MoveBase_Goal_planner(::mbf_msgs::action::MoveBase_Goal & msg)
  : msg_(msg)
  {}
  Init_MoveBase_Goal_recovery_behaviors planner(::mbf_msgs::action::MoveBase_Goal::_planner_type arg)
  {
    msg_.planner = std::move(arg);
    return Init_MoveBase_Goal_recovery_behaviors(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_Goal msg_;
};

class Init_MoveBase_Goal_controller
{
public:
  explicit Init_MoveBase_Goal_controller(::mbf_msgs::action::MoveBase_Goal & msg)
  : msg_(msg)
  {}
  Init_MoveBase_Goal_planner controller(::mbf_msgs::action::MoveBase_Goal::_controller_type arg)
  {
    msg_.controller = std::move(arg);
    return Init_MoveBase_Goal_planner(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_Goal msg_;
};

class Init_MoveBase_Goal_target_pose
{
public:
  Init_MoveBase_Goal_target_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveBase_Goal_controller target_pose(::mbf_msgs::action::MoveBase_Goal::_target_pose_type arg)
  {
    msg_.target_pose = std::move(arg);
    return Init_MoveBase_Goal_controller(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::MoveBase_Goal>()
{
  return mbf_msgs::action::builder::Init_MoveBase_Goal_target_pose();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_MoveBase_Result_final_pose
{
public:
  explicit Init_MoveBase_Result_final_pose(::mbf_msgs::action::MoveBase_Result & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::MoveBase_Result final_pose(::mbf_msgs::action::MoveBase_Result::_final_pose_type arg)
  {
    msg_.final_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_Result msg_;
};

class Init_MoveBase_Result_angle_to_goal
{
public:
  explicit Init_MoveBase_Result_angle_to_goal(::mbf_msgs::action::MoveBase_Result & msg)
  : msg_(msg)
  {}
  Init_MoveBase_Result_final_pose angle_to_goal(::mbf_msgs::action::MoveBase_Result::_angle_to_goal_type arg)
  {
    msg_.angle_to_goal = std::move(arg);
    return Init_MoveBase_Result_final_pose(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_Result msg_;
};

class Init_MoveBase_Result_dist_to_goal
{
public:
  explicit Init_MoveBase_Result_dist_to_goal(::mbf_msgs::action::MoveBase_Result & msg)
  : msg_(msg)
  {}
  Init_MoveBase_Result_angle_to_goal dist_to_goal(::mbf_msgs::action::MoveBase_Result::_dist_to_goal_type arg)
  {
    msg_.dist_to_goal = std::move(arg);
    return Init_MoveBase_Result_angle_to_goal(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_Result msg_;
};

class Init_MoveBase_Result_message
{
public:
  explicit Init_MoveBase_Result_message(::mbf_msgs::action::MoveBase_Result & msg)
  : msg_(msg)
  {}
  Init_MoveBase_Result_dist_to_goal message(::mbf_msgs::action::MoveBase_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_MoveBase_Result_dist_to_goal(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_Result msg_;
};

class Init_MoveBase_Result_outcome
{
public:
  Init_MoveBase_Result_outcome()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveBase_Result_message outcome(::mbf_msgs::action::MoveBase_Result::_outcome_type arg)
  {
    msg_.outcome = std::move(arg);
    return Init_MoveBase_Result_message(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::MoveBase_Result>()
{
  return mbf_msgs::action::builder::Init_MoveBase_Result_outcome();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_MoveBase_Feedback_last_cmd_vel
{
public:
  explicit Init_MoveBase_Feedback_last_cmd_vel(::mbf_msgs::action::MoveBase_Feedback & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::MoveBase_Feedback last_cmd_vel(::mbf_msgs::action::MoveBase_Feedback::_last_cmd_vel_type arg)
  {
    msg_.last_cmd_vel = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_Feedback msg_;
};

class Init_MoveBase_Feedback_current_pose
{
public:
  explicit Init_MoveBase_Feedback_current_pose(::mbf_msgs::action::MoveBase_Feedback & msg)
  : msg_(msg)
  {}
  Init_MoveBase_Feedback_last_cmd_vel current_pose(::mbf_msgs::action::MoveBase_Feedback::_current_pose_type arg)
  {
    msg_.current_pose = std::move(arg);
    return Init_MoveBase_Feedback_last_cmd_vel(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_Feedback msg_;
};

class Init_MoveBase_Feedback_angle_to_goal
{
public:
  explicit Init_MoveBase_Feedback_angle_to_goal(::mbf_msgs::action::MoveBase_Feedback & msg)
  : msg_(msg)
  {}
  Init_MoveBase_Feedback_current_pose angle_to_goal(::mbf_msgs::action::MoveBase_Feedback::_angle_to_goal_type arg)
  {
    msg_.angle_to_goal = std::move(arg);
    return Init_MoveBase_Feedback_current_pose(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_Feedback msg_;
};

class Init_MoveBase_Feedback_dist_to_goal
{
public:
  explicit Init_MoveBase_Feedback_dist_to_goal(::mbf_msgs::action::MoveBase_Feedback & msg)
  : msg_(msg)
  {}
  Init_MoveBase_Feedback_angle_to_goal dist_to_goal(::mbf_msgs::action::MoveBase_Feedback::_dist_to_goal_type arg)
  {
    msg_.dist_to_goal = std::move(arg);
    return Init_MoveBase_Feedback_angle_to_goal(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_Feedback msg_;
};

class Init_MoveBase_Feedback_message
{
public:
  explicit Init_MoveBase_Feedback_message(::mbf_msgs::action::MoveBase_Feedback & msg)
  : msg_(msg)
  {}
  Init_MoveBase_Feedback_dist_to_goal message(::mbf_msgs::action::MoveBase_Feedback::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_MoveBase_Feedback_dist_to_goal(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_Feedback msg_;
};

class Init_MoveBase_Feedback_outcome
{
public:
  Init_MoveBase_Feedback_outcome()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveBase_Feedback_message outcome(::mbf_msgs::action::MoveBase_Feedback::_outcome_type arg)
  {
    msg_.outcome = std::move(arg);
    return Init_MoveBase_Feedback_message(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::MoveBase_Feedback>()
{
  return mbf_msgs::action::builder::Init_MoveBase_Feedback_outcome();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_MoveBase_SendGoal_Request_goal
{
public:
  explicit Init_MoveBase_SendGoal_Request_goal(::mbf_msgs::action::MoveBase_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::MoveBase_SendGoal_Request goal(::mbf_msgs::action::MoveBase_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_SendGoal_Request msg_;
};

class Init_MoveBase_SendGoal_Request_goal_id
{
public:
  Init_MoveBase_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveBase_SendGoal_Request_goal goal_id(::mbf_msgs::action::MoveBase_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MoveBase_SendGoal_Request_goal(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::MoveBase_SendGoal_Request>()
{
  return mbf_msgs::action::builder::Init_MoveBase_SendGoal_Request_goal_id();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_MoveBase_SendGoal_Response_stamp
{
public:
  explicit Init_MoveBase_SendGoal_Response_stamp(::mbf_msgs::action::MoveBase_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::MoveBase_SendGoal_Response stamp(::mbf_msgs::action::MoveBase_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_SendGoal_Response msg_;
};

class Init_MoveBase_SendGoal_Response_accepted
{
public:
  Init_MoveBase_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveBase_SendGoal_Response_stamp accepted(::mbf_msgs::action::MoveBase_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_MoveBase_SendGoal_Response_stamp(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::MoveBase_SendGoal_Response>()
{
  return mbf_msgs::action::builder::Init_MoveBase_SendGoal_Response_accepted();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_MoveBase_SendGoal_Event_response
{
public:
  explicit Init_MoveBase_SendGoal_Event_response(::mbf_msgs::action::MoveBase_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::MoveBase_SendGoal_Event response(::mbf_msgs::action::MoveBase_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_SendGoal_Event msg_;
};

class Init_MoveBase_SendGoal_Event_request
{
public:
  explicit Init_MoveBase_SendGoal_Event_request(::mbf_msgs::action::MoveBase_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_MoveBase_SendGoal_Event_response request(::mbf_msgs::action::MoveBase_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_MoveBase_SendGoal_Event_response(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_SendGoal_Event msg_;
};

class Init_MoveBase_SendGoal_Event_info
{
public:
  Init_MoveBase_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveBase_SendGoal_Event_request info(::mbf_msgs::action::MoveBase_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_MoveBase_SendGoal_Event_request(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::MoveBase_SendGoal_Event>()
{
  return mbf_msgs::action::builder::Init_MoveBase_SendGoal_Event_info();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_MoveBase_GetResult_Request_goal_id
{
public:
  Init_MoveBase_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mbf_msgs::action::MoveBase_GetResult_Request goal_id(::mbf_msgs::action::MoveBase_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::MoveBase_GetResult_Request>()
{
  return mbf_msgs::action::builder::Init_MoveBase_GetResult_Request_goal_id();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_MoveBase_GetResult_Response_result
{
public:
  explicit Init_MoveBase_GetResult_Response_result(::mbf_msgs::action::MoveBase_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::MoveBase_GetResult_Response result(::mbf_msgs::action::MoveBase_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_GetResult_Response msg_;
};

class Init_MoveBase_GetResult_Response_status
{
public:
  Init_MoveBase_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveBase_GetResult_Response_result status(::mbf_msgs::action::MoveBase_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MoveBase_GetResult_Response_result(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::MoveBase_GetResult_Response>()
{
  return mbf_msgs::action::builder::Init_MoveBase_GetResult_Response_status();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_MoveBase_GetResult_Event_response
{
public:
  explicit Init_MoveBase_GetResult_Event_response(::mbf_msgs::action::MoveBase_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::MoveBase_GetResult_Event response(::mbf_msgs::action::MoveBase_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_GetResult_Event msg_;
};

class Init_MoveBase_GetResult_Event_request
{
public:
  explicit Init_MoveBase_GetResult_Event_request(::mbf_msgs::action::MoveBase_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_MoveBase_GetResult_Event_response request(::mbf_msgs::action::MoveBase_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_MoveBase_GetResult_Event_response(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_GetResult_Event msg_;
};

class Init_MoveBase_GetResult_Event_info
{
public:
  Init_MoveBase_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveBase_GetResult_Event_request info(::mbf_msgs::action::MoveBase_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_MoveBase_GetResult_Event_request(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::MoveBase_GetResult_Event>()
{
  return mbf_msgs::action::builder::Init_MoveBase_GetResult_Event_info();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_MoveBase_FeedbackMessage_feedback
{
public:
  explicit Init_MoveBase_FeedbackMessage_feedback(::mbf_msgs::action::MoveBase_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::MoveBase_FeedbackMessage feedback(::mbf_msgs::action::MoveBase_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_FeedbackMessage msg_;
};

class Init_MoveBase_FeedbackMessage_goal_id
{
public:
  Init_MoveBase_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveBase_FeedbackMessage_feedback goal_id(::mbf_msgs::action::MoveBase_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MoveBase_FeedbackMessage_feedback(msg_);
  }

private:
  ::mbf_msgs::action::MoveBase_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::MoveBase_FeedbackMessage>()
{
  return mbf_msgs::action::builder::Init_MoveBase_FeedbackMessage_goal_id();
}

}  // namespace mbf_msgs

#endif  // MBF_MSGS__ACTION__DETAIL__MOVE_BASE__BUILDER_HPP_
