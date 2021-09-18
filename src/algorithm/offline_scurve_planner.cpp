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
 * @file offline_scurve_planner.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include <RTmotion/algorithm/offline_scurve_planner.hpp>
#include <RTmotion/logging.hpp>
#include <chrono>
#include <RTmotion/algorithm/math_utils.hpp>

using namespace RTmotion;

constexpr double EPSILON = 0.01;

namespace trajectory_processing
{
MC_ERROR_CODE
ScurvePlannerOffLine::scurveCheckPosibility(const ScurveCondition& condition)
{
  // Init condition variables for concise code
  const double q0 = condition.q0;
  const double q1 = condition.q1;
  const double v0 = condition.v0;
  const double v1 = condition.v1;
  const double a_max = condition.a_max;
  const double j_max = condition.j_max;

  double dv = abs(v1 - v0);
  double dq = abs(q1 - q0);

  double time_to_reach_max_a = a_max / j_max;

  double time_to_set_set_speeds = sqrt(dv / j_max);

  double Tj = fmin(time_to_reach_max_a, time_to_set_set_speeds);

  if (Tj == time_to_reach_max_a)
  {
    if (dq > 0.5 * (v0 + v1) * (Tj + dv / a_max))
    {
      return mcErrorCodeGood;
    }
  }
  else if (Tj < time_to_reach_max_a)
  {
    if (dq > Tj * (v0 + v1))
    {
      return mcErrorCodeGood;
    }
  }

  return mcErrorCode_Scurve_NotFeasible;
}

MC_ERROR_CODE ScurvePlannerOffLine::computeAccelerationPhase(
    double v, double v_max, double a_max, double j_max, double& Tj, double& Tad)
{
  // Deceleration period
  if ((v_max - v) * j_max < __square(a_max))  // (3.19)(3.20)
  {
    // a_max is not reached
    Tj = sqrt((v_max - v) / j_max);  // (3.21)(3.23)
    Tad = 2 * Tj;                    // (3.21)(3.23)
  }
  else
  {
    // a_max is reached
    Tj = a_max / j_max;              // (3.22)(3.24)
    Tad = Tj + (v_max - v) / a_max;  // (3.22)(3.24)
  }
  return mcErrorCodeGood;
}

MC_ERROR_CODE ScurvePlannerOffLine::computeMaximumSpeedReached(
    const ScurveCondition& condition, ScurveProfile& profile)
{
  // Init condition variables for concise code
  double Tj1 = profile.Tj1, Ta = profile.Ta, Tj2 = profile.Tj2, Td = profile.Td,
         Tv = profile.Tv;

  const double q0 = condition.q0;
  const double q1 = condition.q1;
  const double v0 = condition.v0;
  const double v1 = condition.v1;
  const double v_max = condition.v_max;
  const double a_max = condition.a_max;
  const double j_max = condition.j_max;

  // Acceleration period
  computeAccelerationPhase(v0, v_max, a_max, j_max, Tj1, Ta);

  // Deceleration period
  computeAccelerationPhase(v1, v_max, a_max, j_max, Tj2, Td);

  Tv = (q1 - q0) / v_max - (Ta / 2) * (1 + v0 / v_max) -
       (Td / 2) * (1 + v1 / v_max);  // (3.25)

  profile.Tj1 = Tj1;
  profile.Ta = Ta;
  profile.Tj2 = Tj2;
  profile.Td = Td;
  profile.Tv = Tv;

  if (Tv < 0)
    return mcErrorCode_Scurve_MaxVelNotReached;

  return mcErrorCodeGood;
}

MC_ERROR_CODE ScurvePlannerOffLine::computeMaximumSpeedNotReached(
    const ScurveCondition& condition, ScurveProfile& profile)
{
  // Init condition variables for concise code
  double Tj1 = profile.Tj1, Ta = profile.Ta, Tj2 = profile.Tj2, Td = profile.Td,
         Tv = profile.Tv;

  const double q0 = condition.q0;
  const double q1 = condition.q1;
  const double v0 = condition.v0;
  const double v1 = condition.v1;
  const double a_max = condition.a_max;
  const double j_max = condition.j_max;

  // Assuming that a_max/a_min is reached
  double Tj;
  Tj1 = Tj2 = Tj = a_max / j_max;  // (3.26a)
  Tv = 0;

  double v = __square(a_max) / j_max;
  double delta =
      __square(a_max) * __square(a_max) / __square(j_max) +
      2 * (__square(v0) + __square(v1)) +
      a_max * (4 * (q1 - q0) - 2 * (a_max / j_max) * (v0 + v1));  // (3.27)

  Ta = (v - 2 * v0 + sqrt(delta)) / (2 * a_max);  // (3.26b)
  Td = (v - 2 * v1 + sqrt(delta)) / (2 * a_max);  // (3.26c)

  profile.Tj1 = Tj1;
  profile.Ta = Ta;
  profile.Tj2 = Tj2;
  profile.Td = Td;
  profile.Tv = Tv;

  if (Ta < 0 || Td < 0)
    return mcErrorCode_Scurve_MaxAccNotReached;

  return mcErrorCodeGood;
}

MC_ERROR_CODE ScurvePlannerOffLine::scurveSearchPlanning(
    const ScurveCondition& condition, ScurveProfile& profile, double T,
    double scale, size_t max_iter, double dt_thresh, int timeout)
{
  // Init condition variables for concise code
  ScurveCondition condition_copied = condition;

  size_t it = 0;

  auto start = std::chrono::high_resolution_clock::now();
  int consume_time = 0;
  MC_ERROR_CODE res = mcErrorCodeGood;
  while (it < max_iter && condition_copied.a_max > EPSILON &&
         consume_time <= timeout)
  {
    MC_ERROR_CODE res_step =
        computeMaximumSpeedNotReached(condition_copied, profile);

    double &Tj1 = profile.Tj1, Ta = profile.Ta, Tj2 = profile.Tj2,
           Td = profile.Td, Tv = profile.Tv;

    if (res_step == mcErrorCodeGood)  // Ta > 0 && Td > 0
    {
      if (T == 0)
      {
        if (Ta < 2 * Tj1 ||
            Td < 2 * Tj2)  // Max acc cannot be reached, scale down max acc
        {
          it++;
          condition_copied.a_max *= scale;
          res = mcErrorCode_Scurve_MaxAccNotReached;
        }
        else  // Max acc can be reached
        {
          return mcErrorCodeGood;
        }
      }
      else
      {
        // Sync duration T
        if (abs(T - Ta - Td - Tv) <= dt_thresh)  // Execution duration aligns to
                                                 // T
        {
          return mcErrorCodeGood;
        }
        else  // Too fast to complete before the end of duration T, scale down
              // max acc
        {
          it++;
          condition_copied.a_max *= scale;
          res = mcErrorCode_Scurve_FailToFindMaxAcc;
        }
      }
    }
    else  // Ta < 0 || Td < 0, invalid parameter
    {
      const double q0 = condition_copied.q0;
      const double q1 = condition_copied.q1;
      const double v0 = condition_copied.v0;
      const double v1 = condition_copied.v1;
      const double j_max = condition_copied.j_max;

      if (Ta < 0)
      {
        Ta = 0;
        Tj1 = 0;
        Td = 2 * (q1 - q0) / (v1 + v0);  // (3.28a)
        Tj2 = (j_max * (q1 - q0) -
               sqrt(j_max * (j_max * __square(q1 - q0) +
                             __square(v1 + v0) * (v1 - v0)))) /
              (j_max * (v1 + v0));  // (3.28b)
      }

      if (Td < 0)
      {
        Td = 0;
        Tj2 = 0;
        Ta = 2 * (q1 - q0) / (v1 + v0);  // (3.29a)
        Tj1 = (j_max * (q1 - q0) -
               sqrt(j_max * (j_max * __square(q1 - q0) -
                             __square(v1 + v0) * (v1 - v0)))) /
              (j_max * (v1 + v0));  // (3.29b)
      }
      profile.Tj1 = Tj1;
      profile.Ta = Ta;
      profile.Tj2 = Tj2;
      profile.Td = Td;
      profile.Tv = Tv;
      return mcErrorCodeGood;
    }
    auto end = std::chrono::high_resolution_clock::now();
    consume_time =
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
            .count();
  }

  return res;
}

Eigen::Vector4d ScurvePlannerOffLine::getTrajectoryFunc(
    double t, const ScurveProfile& profile, const ScurveCondition& condition)
{
  // Init condition variables for concise code
  const double Tj1 = profile.Tj1, Ta = profile.Ta, Tj2 = profile.Tj2,
               Td = profile.Td, Tv = profile.Tv;

  const double q0 = condition.q0;
  const double q1 = condition.q1;
  const double v0 = condition.v0;
  const double v1 = condition.v1;
  const double j_max = condition.j_max;

  double T = Ta + Td + Tv;

  // maximum values of velocity and acceleration
  // actually reached during the trajectory
  double a_lim_a = j_max * Tj1;
  double a_lim_d = -j_max * Tj2;
  double v_lim = v0 + (Ta - Tj1) * a_lim_a;

  // Returns trajectory parameters (Eigen::Vector3d) which contains
  // acceleration, speed and position for a given time t
  auto trajectory = [&](double t) -> Eigen::Vector4d {
    double q, v, a, j, tt, j_min = -j_max;
    // Acceleration phase
    if (0 <= t && t < Tj1)
    {
      // (3.30a)
      q = q0 + v0 * t + j_max * __cube(t) / 6;
      v = v0 + j_max * __square(t) / 2;
      a = j_max * t;
      j = j_max;
    }
    else if (Tj1 <= t && t < (Ta - Tj1))
    {
      // (3.30b)
      q = q0 + v0 * t +
          a_lim_a * (3 * __square(t) - 3 * Tj1 * t + __square(Tj1)) / 6;
      v = v0 + a_lim_a * (t - Tj1 / 2);
      a = a_lim_a = j_max * Tj1;
      j = 0;
    }
    else if ((Ta - Tj1) <= t && t < Ta)
    {
      tt = Ta - t;
      // (3.30c)
      q = q0 + (v_lim + v0) * Ta / 2 - v_lim * tt - j_min * __cube(tt) / 6;
      v = v_lim + j_min * __square(tt) / 2;
      a = -j_min * tt;
      j = j_min = -j_max;
    }
    // Constant velocity phase
    else if (Ta <= t && t < (Ta + Tv))
    {
      // (3.30d)
      q = q0 + (v_lim + v0) * Ta / 2 + v_lim * (t - Ta);
      v = v_lim;
      a = 0;
      j = 0;
    }
    // Deceleration phase
    else if ((T - Td) <= t && t < (T - Td + Tj2))
    {
      tt = t - T + Td;
      // (3.30e)
      q = q1 - (v_lim + v1) * Td / 2 + v_lim * tt - j_max * __cube(tt) / 6;
      v = v_lim - j_max * __square(tt) / 2;
      a = -j_max * tt;
      j = j_min = -j_max;
    }
    else if ((T - Td + Tj2) <= t && t < (T - Tj2))
    {
      tt = t - T + Td;
      // (3.30f)
      q = q1 - (v_lim + v1) * Td / 2 + v_lim * tt +
          a_lim_d * (3 * __square(tt) - 3 * Tj2 * tt + __square(Tj2)) / 6;
      v = v_lim + a_lim_d * (tt - Tj2 / 2);
      a = a_lim_d = -j_max * Tj2;
      j = 0;
    }
    else if ((T - Tj2) <= t && t < T)
    {
      tt = T - t;
      // (3.30g)
      q = q1 - v1 * tt - j_max * __cube(tt) / 6;
      v = v1 + j_max * __square(tt) / 2;
      a = -j_max * tt;
      j = j_max;
    }
    else
    {
      q = q1;
      v = v1;
      a = 0;
      j = 0;
    }

    Eigen::Vector4d point;
    point[POSITION_ID] = q;
    point[SPEED_ID] = v;
    point[ACCELERATION_ID] = a;
    point[JERK_ID] = j;

    return point;
  };

  return trajectory(t);
}

Eigen::Vector4d ScurvePlannerOffLine::getTrajectoryFunction(double t)
{
  Eigen::Vector4d point = getTrajectoryFunc(t, profile_, condition_);

  if (!isnan(condition_.q1))
    pointSignTransform(point);

  return point;
}

MC_ERROR_CODE ScurvePlannerOffLine::scurveProfileNoOpt(
    const ScurveCondition& condition, ScurveProfile& profile)
{

  MC_ERROR_CODE res = scurveCheckPosibility(condition);
  if (res != mcErrorCodeGood)
    return res;

  res = computeMaximumSpeedReached(condition, profile);
  if (res != mcErrorCodeGood)
  {
    res = scurveSearchPlanning(condition, profile);
    if (res != mcErrorCodeGood)
      return res;
  }

  return res;
}

MC_ERROR_CODE ScurvePlannerOffLine::planTrajectory1D(double T)
{
  signTransforms(condition_);

  MC_ERROR_CODE res;
  // Computing Optimal time profile
  if (T == 0)
    res = scurveProfileNoOpt(condition_, profile_);
  // Computing constant time profile
  else
    res = scurveSearchPlanning(condition_, profile_, T);

  return res;
}

}  // namespace trajectory_processing
