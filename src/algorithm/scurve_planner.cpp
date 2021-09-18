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
 * @file scurve_planner.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include <RTmotion/algorithm/scurve_planner.hpp>

using namespace RTmotion;

constexpr double EPSILON = 0.01;

namespace trajectory_processing
{
/**
 * @brief Get the sign of the number
 * @param num The input number
 * @return Retrun 1 if num > 0, 0 if num = 0, -1 if num < 0
 */
double sign(double num)
{
  double s = 0;
  s = (num) > 0 ? 1.0 : s;
  s = (num) < 0 ? -1.0 : s;
  return s;
}

void ScurvePlanner::signTransforms(ScurveCondition& condition)
{
  // Init condition variables for concise code
  double q0 = condition.q0;
  double q1 = condition.q1;
  double v0 = condition.v0;
  double v1 = condition.v1;
  double v_max = condition.v_max;
  double a_max = condition.a_max;
  double j_max = condition.j_max;

  double v_min = -v_max;
  double a_min = -a_max;
  double j_min = -j_max;

  s_ = isnan(q1 - q0) ? 1.0 : sign(q1 - q0);
  double vs1 = (s_ + 1) / 2;
  double vs2 = (s_ - 1) / 2;

  condition.q0 = s_ * q0;
  condition.q1 = s_ * q1;
  condition.v0 = s_ * v0;
  condition.v1 = s_ * v1;
  condition.v_max = vs1 * v_max + vs2 * v_min;
  condition.a_max = vs1 * a_max + vs2 * a_min;
  condition.j_max = vs1 * j_max + vs2 * j_min;
}

void ScurvePlanner::pointSignTransform(Eigen::Vector4d& p)
{
  p = s_ * p;
}
}  // namespace trajectory_processing
