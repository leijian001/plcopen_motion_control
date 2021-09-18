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
 * @file fb_power.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include <RTmotion/fb/fb_power.hpp>

namespace RTmotion
{
FbPower::FbPower()
  : enable_(false)
  , enable_positive_(false)
  , enable_negative_(false)
{
}

void FbPower::setExecute(BOOL execute)
{
  enable_ = execute;
}

void FbPower::setEnablePositive(BOOL enable_positive)
{
  enable_positive_ = enable_positive;
}

void FbPower::setEnableNegative(BOOL enable_negative)
{
  enable_negative_ = enable_negative;
}

BOOL FbPower::isEnabled()
{
  return axis_->powerTriggered();
}

BOOL FbPower::getPowerStatus()
{
  return axis_->powerOn();
}

BOOL FbPower::getPowerStatusValid()
{
  return axis_->powerTriggered();
}

BOOL FbPower::isError()
{
  return axis_->getAxisError();
}

MC_ERROR_CODE FbPower::getErrorID()
{
  return axis_->getAxisError();
}

void FbPower::runCycle()
{
  axis_->setPower(enable_, enable_positive_, enable_negative_);
}

}  // namespace RTmotion
