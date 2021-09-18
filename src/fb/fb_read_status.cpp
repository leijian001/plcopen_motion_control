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
 * @file fb_read_status.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include <RTmotion/fb/fb_read_status.hpp>

namespace RTmotion
{
void FbReadStatus::runCycle()
{
  if (enable_)
  {
    valid_ = true;
    switch(axis_->getAxisState())
    {
      case mcErrorStop:
        {
          error_stop_ = true;
          disabled_ = stopping_ = homing_ = standstill_
                      = discrete_motion_ = continuous_motion_ = synchronized_motion_ = false;
        }
        break;
      case mcDisabled:
        {
          disabled_ = true;
          error_stop_ = stopping_ = homing_ = standstill_ 
                      = discrete_motion_ = continuous_motion_ = synchronized_motion_ = false;
        }
        break;
      case mcStopping:
        {
          stopping_ = true;
          error_stop_ = disabled_ = homing_ = standstill_ 
                      = discrete_motion_ = continuous_motion_ = synchronized_motion_ = false;
        }
        break;
      case mcHoming:
        {
          homing_ = true;
          error_stop_ = disabled_ = stopping_ = standstill_ 
                      = discrete_motion_ = continuous_motion_ = synchronized_motion_ = false;
        }
        break;
      case mcStandstill:
        {
          standstill_ = true;
          error_stop_ = disabled_ = stopping_ = homing_ 
                      = discrete_motion_ = continuous_motion_ = synchronized_motion_ = false;
        }
        break;
      case mcDiscreteMotion:
        {
          discrete_motion_ = true;
          error_stop_ = disabled_ = stopping_ = homing_ = standstill_ 
                      = continuous_motion_ = synchronized_motion_ = false;
        }
        break;
      case mcContinuousMotion:
        {
          continuous_motion_ = true;
          error_stop_ = disabled_ = stopping_ = homing_ = standstill_ 
                      = discrete_motion_ = synchronized_motion_ = false;
        }
        break;
      case mcSynchronizedMotion:
        {
          synchronized_motion_ = true;
          error_stop_ = disabled_ = stopping_ = homing_ = standstill_ 
                      = discrete_motion_ = continuous_motion_ = false;
        }
        break;
      default:
        break;
    }
  }
  else
  {
    valid_ = false;
    error_stop_ = disabled_ = stopping_ = homing_ = standstill_
                = discrete_motion_ = continuous_motion_ = synchronized_motion_ = false;
  }
}

}  // namespace RTmotion
