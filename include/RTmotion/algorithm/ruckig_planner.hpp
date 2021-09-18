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
 * @file ruckig_planner.hpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#pragma once

#include <RTmotion/algorithm/scurve_planner.hpp>
#include <ruckig/ruckig.hpp>

namespace trajectory_processing
{

class RuckigPlanner : public ScurvePlanner
{
public:
  RuckigPlanner(double f)
  : otg_(1/f)
  {
  }
  ~RuckigPlanner() = default;

  virtual RTmotion::MC_ERROR_CODE plan() override
  {
    signTransforms(condition_);

    // Set input parameters
    input_.max_velocity = {condition_.v_max};
    input_.max_acceleration = {condition_.a_max};
    input_.max_jerk = {condition_.j_max};

    input_.current_position = {condition_.q0};
    input_.current_velocity = {condition_.v0};
    input_.current_acceleration = {condition_.a0};

    input_.target_position = {condition_.q1};
    input_.target_velocity = {condition_.v1};
    input_.target_acceleration = {condition_.a1};

    return otg_.validate_input(input_) ? RTmotion::mcErrorCodeGood : RTmotion::mcErrorCode_Scurve_InvalidInput;
  }

  virtual Eigen::Vector4d getWaypoint(double t) override;

  ruckig::Ruckig<1> otg_;
  ruckig::InputParameter<1> input_;
  ruckig::OutputParameter<1> output_;

};

}  // namespace trajectory_processing