// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from mbf_msgs:action/Recovery.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mbf_msgs/action/recovery.hpp"


#ifndef MBF_MSGS__ACTION__DETAIL__RECOVERY__BUILDER_HPP_
#define MBF_MSGS__ACTION__DETAIL__RECOVERY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "mbf_msgs/action/detail/recovery__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_Recovery_Goal_concurrency_slot
{
public:
  explicit Init_Recovery_Goal_concurrency_slot(::mbf_msgs::action::Recovery_Goal & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::Recovery_Goal concurrency_slot(::mbf_msgs::action::Recovery_Goal::_concurrency_slot_type arg)
  {
    msg_.concurrency_slot = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::Recovery_Goal msg_;
};

class Init_Recovery_Goal_behavior
{
public:
  Init_Recovery_Goal_behavior()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Recovery_Goal_concurrency_slot behavior(::mbf_msgs::action::Recovery_Goal::_behavior_type arg)
  {
    msg_.behavior = std::move(arg);
    return Init_Recovery_Goal_concurrency_slot(msg_);
  }

private:
  ::mbf_msgs::action::Recovery_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::Recovery_Goal>()
{
  return mbf_msgs::action::builder::Init_Recovery_Goal_behavior();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_Recovery_Result_used_plugin
{
public:
  explicit Init_Recovery_Result_used_plugin(::mbf_msgs::action::Recovery_Result & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::Recovery_Result used_plugin(::mbf_msgs::action::Recovery_Result::_used_plugin_type arg)
  {
    msg_.used_plugin = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::Recovery_Result msg_;
};

class Init_Recovery_Result_message
{
public:
  explicit Init_Recovery_Result_message(::mbf_msgs::action::Recovery_Result & msg)
  : msg_(msg)
  {}
  Init_Recovery_Result_used_plugin message(::mbf_msgs::action::Recovery_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_Recovery_Result_used_plugin(msg_);
  }

private:
  ::mbf_msgs::action::Recovery_Result msg_;
};

class Init_Recovery_Result_outcome
{
public:
  Init_Recovery_Result_outcome()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Recovery_Result_message outcome(::mbf_msgs::action::Recovery_Result::_outcome_type arg)
  {
    msg_.outcome = std::move(arg);
    return Init_Recovery_Result_message(msg_);
  }

private:
  ::mbf_msgs::action::Recovery_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::Recovery_Result>()
{
  return mbf_msgs::action::builder::Init_Recovery_Result_outcome();
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
auto build<::mbf_msgs::action::Recovery_Feedback>()
{
  return ::mbf_msgs::action::Recovery_Feedback(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_Recovery_SendGoal_Request_goal
{
public:
  explicit Init_Recovery_SendGoal_Request_goal(::mbf_msgs::action::Recovery_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::Recovery_SendGoal_Request goal(::mbf_msgs::action::Recovery_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::Recovery_SendGoal_Request msg_;
};

class Init_Recovery_SendGoal_Request_goal_id
{
public:
  Init_Recovery_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Recovery_SendGoal_Request_goal goal_id(::mbf_msgs::action::Recovery_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Recovery_SendGoal_Request_goal(msg_);
  }

private:
  ::mbf_msgs::action::Recovery_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::Recovery_SendGoal_Request>()
{
  return mbf_msgs::action::builder::Init_Recovery_SendGoal_Request_goal_id();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_Recovery_SendGoal_Response_stamp
{
public:
  explicit Init_Recovery_SendGoal_Response_stamp(::mbf_msgs::action::Recovery_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::Recovery_SendGoal_Response stamp(::mbf_msgs::action::Recovery_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::Recovery_SendGoal_Response msg_;
};

class Init_Recovery_SendGoal_Response_accepted
{
public:
  Init_Recovery_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Recovery_SendGoal_Response_stamp accepted(::mbf_msgs::action::Recovery_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Recovery_SendGoal_Response_stamp(msg_);
  }

private:
  ::mbf_msgs::action::Recovery_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::Recovery_SendGoal_Response>()
{
  return mbf_msgs::action::builder::Init_Recovery_SendGoal_Response_accepted();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_Recovery_SendGoal_Event_response
{
public:
  explicit Init_Recovery_SendGoal_Event_response(::mbf_msgs::action::Recovery_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::Recovery_SendGoal_Event response(::mbf_msgs::action::Recovery_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::Recovery_SendGoal_Event msg_;
};

class Init_Recovery_SendGoal_Event_request
{
public:
  explicit Init_Recovery_SendGoal_Event_request(::mbf_msgs::action::Recovery_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_Recovery_SendGoal_Event_response request(::mbf_msgs::action::Recovery_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Recovery_SendGoal_Event_response(msg_);
  }

private:
  ::mbf_msgs::action::Recovery_SendGoal_Event msg_;
};

class Init_Recovery_SendGoal_Event_info
{
public:
  Init_Recovery_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Recovery_SendGoal_Event_request info(::mbf_msgs::action::Recovery_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Recovery_SendGoal_Event_request(msg_);
  }

private:
  ::mbf_msgs::action::Recovery_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::Recovery_SendGoal_Event>()
{
  return mbf_msgs::action::builder::Init_Recovery_SendGoal_Event_info();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_Recovery_GetResult_Request_goal_id
{
public:
  Init_Recovery_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::mbf_msgs::action::Recovery_GetResult_Request goal_id(::mbf_msgs::action::Recovery_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::Recovery_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::Recovery_GetResult_Request>()
{
  return mbf_msgs::action::builder::Init_Recovery_GetResult_Request_goal_id();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_Recovery_GetResult_Response_result
{
public:
  explicit Init_Recovery_GetResult_Response_result(::mbf_msgs::action::Recovery_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::Recovery_GetResult_Response result(::mbf_msgs::action::Recovery_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::Recovery_GetResult_Response msg_;
};

class Init_Recovery_GetResult_Response_status
{
public:
  Init_Recovery_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Recovery_GetResult_Response_result status(::mbf_msgs::action::Recovery_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Recovery_GetResult_Response_result(msg_);
  }

private:
  ::mbf_msgs::action::Recovery_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::Recovery_GetResult_Response>()
{
  return mbf_msgs::action::builder::Init_Recovery_GetResult_Response_status();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_Recovery_GetResult_Event_response
{
public:
  explicit Init_Recovery_GetResult_Event_response(::mbf_msgs::action::Recovery_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::Recovery_GetResult_Event response(::mbf_msgs::action::Recovery_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::Recovery_GetResult_Event msg_;
};

class Init_Recovery_GetResult_Event_request
{
public:
  explicit Init_Recovery_GetResult_Event_request(::mbf_msgs::action::Recovery_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_Recovery_GetResult_Event_response request(::mbf_msgs::action::Recovery_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Recovery_GetResult_Event_response(msg_);
  }

private:
  ::mbf_msgs::action::Recovery_GetResult_Event msg_;
};

class Init_Recovery_GetResult_Event_info
{
public:
  Init_Recovery_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Recovery_GetResult_Event_request info(::mbf_msgs::action::Recovery_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Recovery_GetResult_Event_request(msg_);
  }

private:
  ::mbf_msgs::action::Recovery_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::Recovery_GetResult_Event>()
{
  return mbf_msgs::action::builder::Init_Recovery_GetResult_Event_info();
}

}  // namespace mbf_msgs


namespace mbf_msgs
{

namespace action
{

namespace builder
{

class Init_Recovery_FeedbackMessage_feedback
{
public:
  explicit Init_Recovery_FeedbackMessage_feedback(::mbf_msgs::action::Recovery_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::mbf_msgs::action::Recovery_FeedbackMessage feedback(::mbf_msgs::action::Recovery_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::mbf_msgs::action::Recovery_FeedbackMessage msg_;
};

class Init_Recovery_FeedbackMessage_goal_id
{
public:
  Init_Recovery_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Recovery_FeedbackMessage_feedback goal_id(::mbf_msgs::action::Recovery_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Recovery_FeedbackMessage_feedback(msg_);
  }

private:
  ::mbf_msgs::action::Recovery_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::mbf_msgs::action::Recovery_FeedbackMessage>()
{
  return mbf_msgs::action::builder::Init_Recovery_FeedbackMessage_goal_id();
}

}  // namespace mbf_msgs

#endif  // MBF_MSGS__ACTION__DETAIL__RECOVERY__BUILDER_HPP_
