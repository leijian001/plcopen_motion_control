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
 * @file fb_reset.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include <RTmotion/fb/fb_reset.hpp>

namespace RTmotion
{
FbReset::FbReset()
  : enable_(false)
  , done_(false)
  , busy_(false)
{
}

void FbReset::setExecute(BOOL execute)
{
  enable_ = execute;
  busy_ = enable_ && !done_ ? true : false;
  done_ = enable_ ? done_ : false;
}

BOOL FbReset::isEnabled()
{
  return enable_;
}

BOOL FbReset::isDone()
{
  return done_;
}

BOOL FbReset::isBusy()
{
  return busy_;
}

BOOL FbReset::isError()
{
  return enable_ ? axis_->getAxisError() : mcErrorCodeGood;
}

MC_ERROR_CODE FbReset::getErrorID()
{
  return enable_ ? axis_->getAxisError() : mcErrorCodeGood;
}

void FbReset::runCycle()
{
  if (!enable_)
    return;

  if (!axis_->getAxisError())
  {
    done_ = true;
    return;
  }

  DEBUG_PRINT("FbReset::runCycle, reset error\n");
  axis_->resetError(enable_);

  if (axis_->powerTriggered() && axis_->powerOn())
    if (axis_->getAxisState() == mcStandstill)
      done_ = true;
  
  if (!axis_->powerOn())
    if (axis_->getAxisState() == mcDisabled)
      done_ = true;
}

}  // namespace RTmotion
