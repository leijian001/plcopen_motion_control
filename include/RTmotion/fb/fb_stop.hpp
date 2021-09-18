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
 * @file fb_stop.hpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#pragma once

#include <RTmotion/global.hpp>
#include <RTmotion/fb/fb_axis_motion.hpp>
#include <RTmotion/algorithm/planner.hpp>

#include <chrono>

namespace RTmotion
{
/**
 * @brief Function block MC_Stop
 */
class FbStop : public FbAxisMotion
{
public:
  FbStop();
  ~FbStop() = default;

  virtual MC_ERROR_CODE onRisingEdgeExecution() override;

  virtual MC_ERROR_CODE onFallingEdgeExecution() override;
};
}  // namespace RTmotion