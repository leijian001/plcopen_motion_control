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
 * @file fb_axis_read.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include <RTmotion/fb/fb_axis_read.hpp>

namespace RTmotion
{
FbAxisRead::FbAxisRead()
  : enable_(false)
  , valid_(false)
  , busy_(false)
  , error_(false)
  , error_id_(mcErrorCodeGood)
{
}

void FbAxisRead::setExecute(BOOL execute)
{
  enable_ = execute;
}

BOOL FbAxisRead::isEnabled()
{
  return enable_;
}

BOOL FbAxisRead::isValid()
{
  return valid_;
}

BOOL FbAxisRead::isBusy()
{
  return busy_;
}

BOOL FbAxisRead::isError()
{
  return axis_->getAxisError();
}

MC_ERROR_CODE FbAxisRead::getErrorID()
{
  return axis_->getAxisError();
}

LREAL FbAxisRead::getFloatValue()
{
  return 0.0;
}

MC_ERROR_CODE FbAxisRead::getErrorValue()
{
  return mcErrorCodeGood;
}

void FbAxisRead::runCycle()
{
  
}

}  // namespace RTmotion
