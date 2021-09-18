/*
 * Copyright (c) 2020 Intel Corporation
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file ruckig_planner.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include <RTmotion/algorithm/ruckig_planner.hpp>
#include <RTmotion/logging.hpp>
#include <chrono>

using namespace RTmotion;

constexpr double EPSILON = 0.01;

namespace trajectory_processing
{
Eigen::Vector4d RuckigPlanner::getWaypoint(double t)
{
  otg_.update(input_, output_);
  Eigen::Vector4d point;
  point[POSITION_ID] = output_.new_position[0];
  point[SPEED_ID] = output_.new_velocity[0];
  point[ACCELERATION_ID] = output_.new_acceleration[0];
  point[JERK_ID] = (output_.new_acceleration[0] - input_.current_acceleration[0]) * t;

  input_.current_position = output_.new_position;
  input_.current_velocity = output_.new_velocity;
  input_.current_acceleration = output_.new_acceleration;

  // printf("Debug t %f, sk %f, vk %f, ak %f, jk %f.\n", output_.time, point[POSITION_ID], point[SPEED_ID], point[ACCELERATION_ID], point[JERK_ID]);

  pointSignTransform(point);

  return point;
}
}  // namespace trajectory_processing
