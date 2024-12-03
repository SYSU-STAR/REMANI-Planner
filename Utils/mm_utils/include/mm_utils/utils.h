#ifndef __MM_UTILS_H
#define __MM_UTILS_H

#include <ros/ros.h>

#include <mm_utils/converters.h>
#include <mm_utils/geometry_utils.h>

namespace mm_utils
{

/* judge if value belongs to [low,high] */
template <typename T, typename T2>
bool
in_range(T value, const T2& low, const T2& high)
{
  ROS_ASSERT_MSG(low < high, "%f < %f?", low, high);
  return (low <= value) && (value <= high);
}

/* judge if value belongs to [-limit, limit] */
template <typename T, typename T2>
bool
in_range(T value, const T2& limit)
{
  ROS_ASSERT_MSG(limit > 0, "%f > 0?", limit);
  return in_range(value, -limit, limit);
}

template <typename T, typename T2>
void
limit_range(T& value, const T2& low, const T2& high)
{
  ROS_ASSERT_MSG(low < high, "%f < %f?", low, high);
  if (value < low)
  {
    value = low;
  }

  if (value > high)
  {
    value = high;
  }

  return;
}

template <typename T, typename T2>
void
limit_range(T& value, const T2& limit)
{
  ROS_ASSERT_MSG(limit > 0, "%f > 0?", limit);
  limit_range(value, -limit, limit);
}

typedef std::stringstream DebugSS_t;
} // end of namespace mm_utils

#endif
