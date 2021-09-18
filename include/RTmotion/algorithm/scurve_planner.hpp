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
 * @file scurve_planner.hpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#pragma once

#include <RTmotion/global.hpp>
#include <stdlib.h>
#include <math.h>
#include <algorithm>
#include <Eigen/Core>
#include <map>
#include <RTmotion/algorithm/math_utils.hpp>

using RTmotion::MC_ERROR_CODE;

typedef enum {
  POSITION_ID = 0,
  SPEED_ID = 1,
  ACCELERATION_ID = 2,
  JERK_ID = 3
} WaypointIndex;

namespace trajectory_processing
{
struct ScurveCondition
{
  // Condition inputs for planner
  double q0 = 0.0;
  double q1 = 0.0;
  double v0 = 0.0;
  double v1 = 0.0;
  double a0 = 0.0;
  double a1 = 0.0;
  double v_max = 0.0;
  double a_max = 0.0;
  double j_max = 0.0;
};

struct ScurveProfile
{
  // Time domains for offline scurve planning results
  double Tj1 = 0.0;
  double Ta = 0.0;
  double Tj2 = 0.0;
  double Tv = 0.0;
  // Time domains for online scurve planning results
  double Tj2a = 0.0;
  double Tj2c = 0.0;
  double Tj2b = 0.0;
  double Td = 0.0;
  double Th = 0.0;
};

class ScurvePlanner
{
public:
  ScurvePlanner() = default;
  ~ScurvePlanner() = default;

  /**
   * @brief Sign transforms for being able to calculate trajectory with q1 < q0.
   *        Look at 'Trajectory planning for automatic machines and
   * robots(2008)'
   * @param condition  The input scurve condition, i.e. q0, q1, v0, v1, v_max,
   * a_max, j_max
   */
  void signTransforms(ScurveCondition& condition);

  /**
   * @brief Transforms point back to the original sign.
   * @param p Reference to Point, i.e. acc, vel, pos
   */
  void pointSignTransform(Eigen::Vector4d& p);

  virtual MC_ERROR_CODE plan() = 0;
  virtual Eigen::Vector4d getWaypoint(double t) = 0;

  ScurveProfile profile_;
  ScurveCondition condition_;
  double s_ = 0.0;  // Sign transform flag parameter
};

}  // namespace trajectory_processing