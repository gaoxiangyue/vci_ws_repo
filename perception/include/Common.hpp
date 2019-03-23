#pragma once

#define PARAM_HELPER_STRINGIFY(x) #x

#define PARAM_HELPER_TOSTRING(x) PARAM_HELPER_STRINGIFY(x)

#define READ_PARAM_BEGIN \
   std::string __param_name;

#define READ_PRIVATE_PARAM_WITH_DEFAULT(type_of_x, x, default_value)       \
  __param_name = PARAM_HELPER_TOSTRING(x);                                 \
  __param_name.pop_back();                                                 \
  nh_private.param<type_of_x>(__param_name, x, default_value);             \
  ROS_INFO_STREAM(__param_name << ": " << x);

#define READ_PARAM_WITH_DEFAULT(type_of_x, x, default_value)               \
  type_of_x x;                                                             \
  __param_name = PARAM_HELPER_TOSTRING(x);                                 \
  nh_private.param<type_of_x>(__param_name, x, default_value);             \
  ROS_INFO_STREAM(__param_name << ": " << x);

#define DEBUG_PRINT_VAR(x) \
  ROS_DEBUG_STREAM(PARAM_HELPER_TOSTRING(x) << ": " << x);

#define MRR_WARN_IF(name, err_flag) \
  ROS_WARN_STREAM_COND(err_flag, name << " radar has encountered: " << PARAM_HELPER_TOSTRING(err_flag));
  