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

#ifndef MAV_PLANNING_MSGS_CONVERSIONS_H
#define MAV_PLANNING_MSGS_CONVERSIONS_H

#include <cassert>
#include <cstdint>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "mav_planning_msgs/msg/polynomial_segment.hpp"
#include "mav_planning_msgs/msg/polynomial_trajectory.hpp"
#include "mav_planning_msgs/eigen_planning_msgs.h"

// deprecated
#include "mav_planning_msgs/conversions_deprecated.h"

namespace mav_planning_msgs {

// Preserve the original package-level type names as aliases while the actual
// generated ROS 2 messages remain in mav_planning_msgs::msg.
using PolynomialSegment = msg::PolynomialSegment;
using PolynomialTrajectory = msg::PolynomialTrajectory;

/// Converts a PolynomialSegment message to an EigenPolynomialSegment structure.
inline void eigenPolynomialSegmentFromMsg(const PolynomialSegment& msg,
                                          EigenPolynomialSegment* segment) {
  assert(segment != NULL);

  vectorFromMsgArray(msg.x, &(segment->x));
  vectorFromMsgArray(msg.y, &(segment->y));
  vectorFromMsgArray(msg.z, &(segment->z));
  vectorFromMsgArray(msg.yaw, &(segment->yaw));
  vectorFromMsgArray(msg.rx, &(segment->rx));
  vectorFromMsgArray(msg.ry, &(segment->ry));
  vectorFromMsgArray(msg.rz, &(segment->rz));

  segment->segment_time_ns =
      static_cast<std::uint64_t>(detail::durationToNanoseconds(msg.segment_time));
  segment->num_coeffs = msg.num_coeffs;
}

/// Converts a PolynomialTrajectory message to a EigenPolynomialTrajectory
inline void eigenPolynomialTrajectoryFromMsg(
    const PolynomialTrajectory& msg,
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
                                          PolynomialSegment* msg) {
  assert(msg != NULL);
  msgArrayFromVector(segment.x, &(msg->x));
  msgArrayFromVector(segment.y, &(msg->y));
  msgArrayFromVector(segment.z, &(msg->z));
  msgArrayFromVector(segment.yaw, &(msg->yaw));
  msgArrayFromVector(segment.rx, &(msg->rx));
  msgArrayFromVector(segment.ry, &(msg->ry));
  msgArrayFromVector(segment.rz, &(msg->rz));

  detail::durationFromNanoseconds(segment.segment_time_ns, &msg->segment_time);
  msg->num_coeffs = segment.num_coeffs;
}

/// Converts an EigenPolynomialTrajectory to a PolynomialTrajectory message.
/// Does NOT set the header!
inline void polynomialTrajectoryMsgFromEigen(
    const EigenPolynomialTrajectory& eigen_trajectory,
    PolynomialTrajectory* msg) {
  assert(msg != NULL);
  msg->segments.reserve(eigen_trajectory.size());
  for (const auto& eigen_segment : eigen_trajectory) {
    PolynomialSegment segment;
    polynomialSegmentMsgFromEigen(eigen_segment, &segment);
    msg->segments.push_back(segment);
  }
}

}  // namespace mav_planning_msgs

#endif // MAV_PLANNING_MSGS_CONVERSIONS_H
