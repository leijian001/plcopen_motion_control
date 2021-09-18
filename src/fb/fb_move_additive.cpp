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
 * @file fb_move_additive.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include <RTmotion/fb/fb_move_additive.hpp>

namespace RTmotion
{
FbMoveAdditive::FbMoveAdditive()
{
  distance_ = 0.0;
}

MC_ERROR_CODE FbMoveAdditive::onRisingEdgeExecution()
{
  axis_->addFBToQueue(this, mcMoveAdditive);
  return mcErrorCodeGood;
}

}  // namespace RTmotion
