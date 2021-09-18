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
 * @file planner.hpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#pragma once

#include <RTmotion/global.hpp>
#include <Eigen/Core>
#include <RTmotion/algorithm/offline_scurve_planner.hpp>
#include <RTmotion/algorithm/online_scurve_planner.hpp>
#include <RTmotion/algorithm/ruckig_planner.hpp>
#include <chrono>

using RTmotion::MC_ERROR_CODE;

namespace trajectory_processing
{
typedef enum { OFFLine = 0, ONLine = 1, Ruckig = 2} PlannerType;

class AxisPlanner
{
public:
  AxisPlanner(double f)
  : ruckig_planner_(f)
  {
    start_time_ = 0.0;
    type_ = Ruckig;
    scurve_planner_ = nullptr;
  }

  virtual ~AxisPlanner()
  {
    scurve_planner_ = nullptr;
  }

  void setCondition(double start_pos, double end_pos, double start_vel,
                    double end_vel, double vel_max, double acc_max,
                    double jerk_max);

  MC_ERROR_CODE planTrajectory();

  Eigen::Vector4d getTrajectoryPoint(double t);

  MC_ERROR_CODE onReplan();

  MC_ERROR_CODE onExecution(double t, double* pos_cmd, double* vel_cmd);

  ScurveCondition& getScurveCondition();

  ScurveProfile& getScurveProfile();

  void setStartTime(double t);

private:
  ScurvePlanner* scurve_planner_;
  ScurvePlannerOnLine online_planner_;
  ScurvePlannerOffLine offline_planner_;
  RuckigPlanner ruckig_planner_;
  double start_time_;
  PlannerType type_;
};

}  // namespace trajectory_processing
