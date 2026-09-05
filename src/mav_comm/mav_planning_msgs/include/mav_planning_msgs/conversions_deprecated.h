/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MAV_PLANNING_MSGS_CONVERSIONS_DEPRECATED_H
#define MAV_PLANNING_MSGS_CONVERSIONS_DEPRECATED_H

#include <cassert>
#include <cstdint>
#include <type_traits>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "mav_planning_msgs/msg/polynomial_segment4_d.hpp"
#include "mav_planning_msgs/msg/polynomial_trajectory4_d.hpp"
#include "mav_planning_msgs/eigen_planning_msgs.h"

namespace mav_planning_msgs {

// Keep the ROS 1-facing names available at the original package boundary
// while using the ROS 2 generated message namespace internally.  These are
// aliases, not new message types or a new package boundary.
using PolynomialSegment4D = msg::PolynomialSegment4D;
using PolynomialTrajectory4D = msg::PolynomialTrajectory4D;

namespace detail {

inline std::int64_t durationToNanoseconds(
    const builtin_interfaces::msg::Duration& duration) {
  return static_cast<std::int64_t>(duration.sec) * 1000000000LL +
         static_cast<std::int64_t>(duration.nanosec);
}

inline void durationFromNanoseconds(
    std::uint64_t nanoseconds,
    builtin_interfaces::msg::Duration* duration) {
  assert(duration != nullptr);
  duration->sec = static_cast<std::int32_t>(nanoseconds / 1000000000ULL);
  duration->nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000ULL);
}

// ROS 2 uses allocator-aware std::vector fields for unbounded arrays.  A
// template keeps the original helper names usable for both the 4D and the
// full PolynomialSegment message without relying on ROS 1 *_type typedefs.
template <typename Array>
inline void vectorFromMsgArray(const Array& array, Eigen::VectorXd* x) {
  assert(x != nullptr);
  if (array.empty()) {
    x->resize(0);
    return;
  }
  *x = Eigen::Map<const Eigen::VectorXd>(array.data(),
                                         static_cast<Eigen::Index>(array.size()));
}

template <typename Array>
inline void msgArrayFromVector(const Eigen::VectorXd& x, Array* array) {
  assert(array != nullptr);
  array->resize(static_cast<typename Array::size_type>(x.size()));
  if (!array->empty()) {
    Eigen::Map<Eigen::VectorXd> map(array->data(),
                                    static_cast<Eigen::Index>(array->size()));
    map = x;
  }
}

}  // namespace detail

/// Converts a PolynomialSegment double array to an Eigen::VectorXd.
template <typename Array>
inline void vectorFromMsgArray(const Array& array, Eigen::VectorXd* x) {
  detail::vectorFromMsgArray(array, x);
}

/// Converts an Eigen::VectorXd to a PolynomialSegment double array.
template <typename Array>
inline void msgArrayFromVector(const Eigen::VectorXd& x, Array* array) {
  detail::msgArrayFromVector(x, array);
}

/// Converts a PolynomialSegment message to an EigenPolynomialSegment structure.
inline void eigenPolynomialSegmentFromMsg(const PolynomialSegment4D& msg,
                                          EigenPolynomialSegment* segment) {
  assert(segment != NULL);

  vectorFromMsgArray(msg.x, &(segment->x));
  vectorFromMsgArray(msg.y, &(segment->y));
  vectorFromMsgArray(msg.z, &(segment->z));
  vectorFromMsgArray(msg.yaw, &(segment->yaw));

  segment->segment_time_ns =
      static_cast<std::uint64_t>(detail::durationToNanoseconds(msg.segment_time));
  segment->num_coeffs = msg.num_coeffs;
}

/// Converts a PolynomialTrajectory message to a EigenPolynomialTrajectory
inline void eigenPolynomialTrajectoryFromMsg(
    const PolynomialTrajectory4D& msg,
    EigenPolynomialTrajectory* eigen_trajectory) {
  assert(eigen_trajectory != NULL);
  eigen_trajectory->clear();
  eigen_trajectory->reserve(msg.segments.size());
  for (const auto& segment_msg : msg.segments) {
    EigenPolynomialSegment segment;
    eigenPolynomialSegmentFromMsg(segment_msg, &segment);
    eigen_trajectory->push_back(segment);
  }
}


/// Converts an EigenPolynomialSegment to a PolynomialSegment message. Does NOT
/// set the header!
inline void polynomialSegmentMsgFromEigen(const EigenPolynomialSegment& segment,
                                          PolynomialSegment4D* msg) {
  assert(msg != NULL);
  msgArrayFromVector(segment.x, &(msg->x));
  msgArrayFromVector(segment.y, &(msg->y));
  msgArrayFromVector(segment.z, &(msg->z));
  msgArrayFromVector(segment.yaw, &(msg->yaw));

  detail::durationFromNanoseconds(segment.segment_time_ns, &msg->segment_time);
  msg->num_coeffs = segment.num_coeffs;
}

/// Converts an EigenPolynomialTrajectory to a PolynomialTrajectory message.
/// Does NOT set the header!
inline void polynomialTrajectoryMsgFromEigen(
    const EigenPolynomialTrajectory& eigen_trajectory,
    PolynomialTrajectory4D* msg) {
  assert(msg != NULL);
  msg->segments.reserve(eigen_trajectory.size());
  for (const auto& eigen_segment : eigen_trajectory) {
    PolynomialSegment4D segment;
    polynomialSegmentMsgFromEigen(eigen_segment, &segment);
    msg->segments.push_back(segment);
  }
}

}  // namespace mav_planning_msgs

#endif // MAV_PLANNING_MSGS_CONVERSIONS_DEPRECATED_H
