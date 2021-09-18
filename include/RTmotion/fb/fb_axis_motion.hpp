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
 * @file fb_axis_motion.hpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#pragma once

#include <RTmotion/fb/fb_base.hpp>
#include <RTmotion/fb/fb_axis_node.hpp>

#include <chrono>

namespace RTmotion
{
/**
 * @brief Function block base for single axis motion
 */
class FbAxisMotion : public FunctionBlock, public FbAxisNode
{
public:
  FbAxisMotion();
  virtual ~FbAxisMotion() = default;

  virtual void runCycle() override;

  // Functions for addressing inputs
  virtual void setExecute(BOOL execute);
  virtual void setContinuousUpdate(BOOL continuous_update);
  virtual void setPosition(LREAL position);
  virtual void setVelocity(LREAL);
  virtual void setAcceleration(LREAL acceleration);
  virtual void setDeceleration(LREAL deceleration);
  virtual void setJerk(LREAL jerk);
  virtual void setDirection(MC_DIRECTION direction);
  virtual void setBufferMode(MC_BUFFER_MODE mode);

  virtual BOOL isError() override;
  virtual MC_ERROR_CODE getErrorID() override;

  virtual LREAL getPosition() override;
  virtual LREAL getVelocity() override;
  virtual LREAL getAcceleration() override;
  virtual LREAL getDeceleration() override;
  virtual LREAL getJerk() override;
  virtual MC_DIRECTION getDirection() override;
  virtual MC_BUFFER_MODE getBufferMode() override;

  // Functions for execution
  virtual MC_ERROR_CODE onRisingEdgeExecution() override;
  virtual MC_ERROR_CODE onFallingEdgeExecution() override;
  virtual MC_ERROR_CODE onExecution() override;

protected:
  /// Inputs
  VAR_INPUT BOOL continuous_update_;
  VAR_INPUT LREAL position_;
  VAR_INPUT LREAL velocity_;
  VAR_INPUT LREAL acceleration_;
  VAR_INPUT LREAL deceleration_;
  VAR_INPUT LREAL jerk_;
  VAR_INPUT MC_DIRECTION direction_;

  /// Flags to control the process
  bool triggered_;
};
}  // namespace RTmotion
