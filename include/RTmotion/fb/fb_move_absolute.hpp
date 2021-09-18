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
 * @file fb_move_absolute.hpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#pragma once

#include <RTmotion/global.hpp>
#include <RTmotion/fb/fb_axis_motion.hpp>

#include <chrono>

namespace RTmotion
{
/**
 * @brief Function block MC_MoveAbsolute
 * \image html MC_MoveRelative.png width=1000cm
 * \image html MoveRelative_Example.png width=1000cm
 */
class FbMoveAbsolute : public FbAxisMotion
{
public:
  FbMoveAbsolute();
  ~FbMoveAbsolute() = default;

  virtual MC_ERROR_CODE onRisingEdgeExecution() override;
};
}  // namespace RTmotion
