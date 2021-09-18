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
 * @file fb_axis_node.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include <RTmotion/fb/fb_axis_node.hpp>
#include <RTmotion/logging.hpp>

namespace RTmotion
{
FbAxisNode::FbAxisNode()
  : execute_(false)
  , done_(false)
  , busy_(false)
  , active_(false)
  , command_aborted_(false)
  , error_(false)
  , error_id_(mcErrorCodeGood)
  , buffer_mode_(mcBuffered)
  , output_flag_(false)
{
}

void FbAxisNode::syncStatus(BOOL done, BOOL busy, BOOL active, BOOL cmd_aborted,
                            BOOL error, MC_ERROR_CODE error_id)
{
  done_ = done;
  busy_ = busy;
  active_ = active;
  command_aborted_ = cmd_aborted;
  error_ = error;
  error_id_ = error_id;

  output_flag_ = !execute_;
  DEBUG_PRINT("FbAxisNode::syncStatus %s\n", error_ ? "true" : "false" );
}

BOOL FbAxisNode::isEnabled()
{
  return execute_;
}

BOOL FbAxisNode::isDone()
{
  return done_;
}

BOOL FbAxisNode::isBusy()
{
  return busy_;
}

BOOL FbAxisNode::isActive()
{
  return active_;
}

BOOL FbAxisNode::isAborted()
{
  return command_aborted_;
}

BOOL FbAxisNode::isError()
{
  DEBUG_PRINT("FbAxisNode::isError %s\n", error_ ? "true" : "false" );
  return error_;
}

MC_ERROR_CODE FbAxisNode::getErrorID()
{
  return error_id_;
}

}  // namespace RTmotion
