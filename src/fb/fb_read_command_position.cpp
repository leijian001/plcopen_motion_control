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
 * @file fb_read_command_position.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include <RTmotion/fb/fb_read_command_position.hpp>

namespace RTmotion
{
LREAL FbReadCommandPosition::getFloatValue()
{
  return position_;
}

void FbReadCommandPosition::runCycle()
{
  if (enable_)
  {
    valid_ = true;
    if (axis_->getAxisState() == mcErrorStop)
    {
      position_ = 0;
      error_ = true;
      error_id_ = mcErrorCode_AxisErrorStop;
      return;
    }

    error_ = false;
    position_ = axis_->toUserPosCmd();
  }
  else
  {
    valid_ = false;
    busy_ = false;
    error_ = false;
  }
}

}  // namespace RTmotion
