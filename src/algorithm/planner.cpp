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
 * @file s_curve_planner.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include <RTmotion/algorithm/planner.hpp>
#include <RTmotion/logging.hpp>
#include <chrono>

constexpr double EPSILON = 0.01;

namespace trajectory_processing
{
void AxisPlanner::setCondition(double start_pos, double end_pos,
                               double start_vel, double end_vel, double vel_max,
                               double acc_max, double jerk_max)
{
  if (isnan(end_pos))
  {
    type_ = OFFLine;
    scurve_planner_ = &offline_planner_;
  }
  else
  {
    if (type_ == ONLine)
      scurve_planner_ = &online_planner_;

    if (type_ == Ruckig)
      scurve_planner_ = &ruckig_planner_;
  }

  scurve_planner_->condition_.q0 = start_pos;
  scurve_planner_->condition_.q1 = end_pos;
  scurve_planner_->condition_.v0 = start_vel;
  scurve_planner_->condition_.v1 = end_vel;
  scurve_planner_->condition_.a0 = 0.0;
  scurve_planner_->condition_.a1 = 0.0;
  scurve_planner_->condition_.v_max = vel_max;
  scurve_planner_->condition_.a_max = acc_max;
  scurve_planner_->condition_.j_max = jerk_max;
  DEBUG_PRINT(
    "AxisPlanner::setCondition:Scurve condition: q0 = %f, q1 = %f, v0 = %f, v1 = %f \n",
    scurve_planner_->condition_.q0, scurve_planner_->condition_.q1,
    scurve_planner_->condition_.v0, scurve_planner_->condition_.v1);
}

MC_ERROR_CODE AxisPlanner::planTrajectory()
{
  return scurve_planner_->plan();
}

Eigen::Vector4d AxisPlanner::getTrajectoryPoint(double t)
{
  return scurve_planner_->getWaypoint(t);
}

MC_ERROR_CODE AxisPlanner::onReplan()
{
  MC_ERROR_CODE res = planTrajectory();

  DEBUG_PRINT(
      "AxisPlanner::onReplan:Scurve profile: Ta = %f, Tv = %f, Td = %f, Tj1 = %f, Tj2 = %f \n",
      scurve_planner_->profile_.Ta, scurve_planner_->profile_.Tv, scurve_planner_->profile_.Td,
      scurve_planner_->profile_.Tj1, scurve_planner_->profile_.Tj2);
  return res;
}

MC_ERROR_CODE AxisPlanner::onExecution(double t, double* pos_cmd,
                                       double* vel_cmd)
{
  double d = t - start_time_;
  Eigen::Vector4d point = getTrajectoryPoint(d);
  *pos_cmd = point[POSITION_ID];
  *vel_cmd = point[SPEED_ID];

  DEBUG_PRINT("AxisPlanner::onExecution:time: %f, pos_cmd: %f, vel_cmd: %f\n", d, point[POSITION_ID],
              point[SPEED_ID]);
  return RTmotion::mcErrorCodeGood;
}

ScurveCondition& AxisPlanner::getScurveCondition()
{
  return scurve_planner_->condition_;
}

ScurveProfile& AxisPlanner::getScurveProfile()
{
  return scurve_planner_->profile_;
}

void AxisPlanner::setStartTime(double t)
{
  start_time_ = t;
}

}  // namespace trajectory_processing