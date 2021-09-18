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
 * @file fb_axis_motion.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include <RTmotion/fb/fb_axis_motion.hpp>

namespace RTmotion
{
FbAxisMotion::FbAxisMotion()
  : continuous_update_(false)
  , position_(0)
  , velocity_(0)
  , acceleration_(0)
  , deceleration_(0)
  , jerk_(5000)
  , direction_(mcCurrentDirection)
  , triggered_(false)
{
}

void FbAxisMotion::runCycle()
{
  if (execute_ && !triggered_)  // `Execute` rising edge
  {
    MC_ERROR_CODE err = onRisingEdgeExecution();
    if (err)
      onError(err);
    else
    {
      active_ = done_ = error_ = command_aborted_ = false;
      busy_ = true;
    }
  }
  else if (execute_ && triggered_)  // `Execute` hold high value
  {
    MC_ERROR_CODE err = onExecution();
    if (err)
      onError(err);
  }
  else if (!execute_ && triggered_)  // `Execute` falling edge
  {
    MC_ERROR_CODE err = onFallingEdgeExecution();
    if (err)
      onError(err);
    else
    {
      if ((done_ || command_aborted_ || error_) && !busy_)
      {
        done_ = command_aborted_ = error_ = busy_ = active_ = false;
        error_id_ = mcErrorCodeGood;
      }
    }
    
    output_flag_ = false;
  }  // `Execute` hold low value
  else
  {
    if (!busy_ && !output_flag_)
    {
      active_ = done_ = command_aborted_ = error_ = false;
      error_id_ = mcErrorCodeGood;
    }
    output_flag_ = false;
  }

  triggered_ = execute_;
}

MC_ERROR_CODE FbAxisMotion::onRisingEdgeExecution()
{
  return mcErrorCodeGood;
}

MC_ERROR_CODE FbAxisMotion::onFallingEdgeExecution()
{
  return mcErrorCodeGood;
}

MC_ERROR_CODE FbAxisMotion::onExecution()
{
  return mcErrorCodeGood;
}

void FbAxisMotion::setExecute(BOOL execute)
{
  execute_ = execute;
}

void FbAxisMotion::setContinuousUpdate(BOOL continuous_update)
{
  continuous_update_ = continuous_update;
}

void FbAxisMotion::setPosition(LREAL position)
{
  position_ = position;
}

void FbAxisMotion::setVelocity(LREAL velocity)
{
  velocity_ = velocity;
}

void FbAxisMotion::setAcceleration(LREAL acceleration)
{
  acceleration_ = acceleration;
}

void FbAxisMotion::setDeceleration(LREAL deceleration)
{
  deceleration_ = deceleration;
}

void FbAxisMotion::setJerk(LREAL jerk)
{
  jerk_ = jerk;
}

void FbAxisMotion::setDirection(MC_DIRECTION direction)
{
  direction_ = direction;
}

void FbAxisMotion::setBufferMode(MC_BUFFER_MODE mode)
{
  buffer_mode_ = mode;
}

BOOL FbAxisMotion::isError()
{
  return axis_->getAxisError() ? (bool)axis_->getAxisError() : error_;
}

MC_ERROR_CODE FbAxisMotion::getErrorID()
{
  return axis_->getAxisError() ? axis_->getAxisError() : error_id_;
}

LREAL FbAxisMotion::getPosition()
{
  return position_;
}

LREAL FbAxisMotion::getVelocity()
{
  return velocity_;
}

LREAL FbAxisMotion::getAcceleration()
{
  return acceleration_;
}

LREAL FbAxisMotion::getDeceleration()
{
  return deceleration_;
}

LREAL FbAxisMotion::getJerk()
{
  return jerk_;
}

MC_DIRECTION FbAxisMotion::getDirection()
{
  return direction_;
}

MC_BUFFER_MODE FbAxisMotion::getBufferMode()
{
  return buffer_mode_;
}

}  // namespace RTmotion
