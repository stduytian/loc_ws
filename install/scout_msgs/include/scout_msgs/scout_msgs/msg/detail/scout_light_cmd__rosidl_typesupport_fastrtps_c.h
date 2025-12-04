// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from scout_msgs:msg/ScoutLightCmd.idl
// generated code does not contain a copyright notice
#ifndef SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_CMD__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_CMD__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "scout_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "scout_msgs/msg/detail/scout_light_cmd__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scout_msgs
size_t get_serialized_size_scout_msgs__msg__ScoutLightCmd(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scout_msgs
size_t max_serialized_size_scout_msgs__msg__ScoutLightCmd(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scout_msgs
bool cdr_serialize_key_scout_msgs__msg__ScoutLightCmd(
  const scout_msgs__msg__ScoutLightCmd * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scout_msgs
bool cdr_deserialize_key_scout_msgs__msg__ScoutLightCmd(
  eprosima::fastcdr::Cdr &,
  scout_msgs__msg__ScoutLightCmd * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scout_msgs
size_t get_serialized_size_key_scout_msgs__msg__ScoutLightCmd(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scout_msgs
size_t max_serialized_size_key_scout_msgs__msg__ScoutLightCmd(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scout_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scout_msgs, msg, ScoutLightCmd)();

#ifdef __cplusplus
}
#endif

#endif  // SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_CMD__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
