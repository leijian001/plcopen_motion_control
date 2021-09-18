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
 * @file fb_read_status.hpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#pragma once

#include <RTmotion/fb/fb_axis_read.hpp>

namespace RTmotion
{
/**
 * @brief Function block base for single axis motion
 */
class FbReadStatus : public FbAxisRead
{
public:
  FbReadStatus()
  {
    error_stop_ = false;
    disabled_ = false;
    stopping_ = false;
    homing_ = false;
    standstill_ = false;
    discrete_motion_ = false;
    continuous_motion_ = false;
    synchronized_motion_ = false;
  }
  virtual ~FbReadStatus() = default;

  BOOL isErrorStop() {return error_stop_;}
  BOOL isDisabled() {return disabled_;}
  BOOL isStopping() {return stopping_;}
  BOOL isHoming() {return homing_;}
  BOOL isStandStill() {return standstill_;}
  BOOL isDiscretMotion() {return discrete_motion_;}
  BOOL isContinuousMotion() {return continuous_motion_;}
  BOOL isSynchronizedMotion() {return synchronized_motion_;}

  // Functions for execution
  virtual void runCycle() override;

protected:

  VAR_OUTPUT BOOL error_stop_;
  VAR_OUTPUT BOOL disabled_;
  VAR_OUTPUT BOOL stopping_;
  VAR_OUTPUT BOOL homing_;
  VAR_OUTPUT BOOL standstill_;
  VAR_OUTPUT BOOL discrete_motion_;
  VAR_OUTPUT BOOL continuous_motion_;
  VAR_OUTPUT BOOL synchronized_motion_;
};
}  // namespace RTmotion
