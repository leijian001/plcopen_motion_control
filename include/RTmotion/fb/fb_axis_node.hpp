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
 * @file fb_axis_node.hpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#pragma once

#include <RTmotion/fb/fb_global.hpp>

namespace RTmotion
{
/**
 * @brief Function block base for single axis motion
 */
class FbAxisNode
{
public:
  FbAxisNode();
  virtual ~FbAxisNode() = default;

  // Functions for addressing outputs
  void syncStatus(BOOL done, BOOL busy, BOOL active, BOOL cmd_aborted,
                  BOOL error, MC_ERROR_CODE error_id);

  BOOL isEnabled();
  BOOL isDone();
  BOOL isBusy();
  BOOL isActive();
  BOOL isAborted();
  virtual BOOL isError();
  virtual MC_ERROR_CODE getErrorID();

  virtual LREAL getPosition() = 0;
  virtual LREAL getVelocity() = 0;
  virtual LREAL getAcceleration() = 0;
  virtual LREAL getDeceleration() = 0;
  virtual LREAL getJerk() = 0;
  virtual MC_DIRECTION getDirection() = 0;
  virtual MC_BUFFER_MODE getBufferMode() = 0;

protected:
  VAR_INPUT BOOL execute_;

  /// Outputs
  VAR_OUTPUT BOOL done_;
  VAR_OUTPUT BOOL busy_;
  VAR_OUTPUT BOOL active_;
  VAR_OUTPUT BOOL command_aborted_;
  VAR_OUTPUT BOOL error_;
  VAR_OUTPUT MC_ERROR_CODE error_id_;
  VAR_OUTPUT MC_BUFFER_MODE buffer_mode_;

  bool output_flag_;  /// Hold the output for another cycle
};
}  // namespace RTmotion
