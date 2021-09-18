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
 * @file fb_base.hpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#pragma once

#include <RTmotion/fb/fb_global.hpp>
#include <RTmotion/axis.hpp>
#include <RTmotion/logging.hpp>
#include <vector>
#include <map>

namespace RTmotion
{
class Axis;
typedef std::shared_ptr<Axis> AXIS_REF;

class FunctionBlock
{
public:
  FunctionBlock();
  virtual ~FunctionBlock()
  {
    axis_.reset();
  }

  virtual void setAxis(AXIS_REF axis);

  virtual void runCycle() = 0;

  // Functions for working on different status
  virtual MC_ERROR_CODE onRisingEdgeExecution();

  virtual MC_ERROR_CODE onFallingEdgeExecution();

  virtual MC_ERROR_CODE onExecution();

  virtual void onError(MC_ERROR_CODE error_code);

  virtual void setTimeRecodPtr(std::map<std::string, double>* ptr)
  {
    time_record_ptr_ = ptr;
  };

protected:
  /// AXIS_REF
  VAR_IN_OUT AXIS_REF axis_;

  std::map<std::string, double>* time_record_ptr_;
};

}  // namespace RTmotion
