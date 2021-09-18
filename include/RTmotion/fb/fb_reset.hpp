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
 * @file fb_reset.hpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#pragma once

#include <RTmotion/fb/fb_base.hpp>

namespace RTmotion
{
/**
 * @brief Function block base for single axis motion
 */
class FbReset : public FunctionBlock
{
public:
  FbReset();
  virtual ~FbReset() = default;

  // Functions for addressing inputs
  void setExecute(BOOL execute);

  BOOL isEnabled();
  BOOL isDone();
  BOOL isBusy();
  BOOL isError();
  MC_ERROR_CODE getErrorID();

  // Functions for execution
  virtual void runCycle() override;

protected:
  /// Inputs
  VAR_INPUT BOOL enable_;

  VAR_OUTPUT BOOL done_;
  VAR_OUTPUT BOOL busy_;
};
}  // namespace RTmotion
