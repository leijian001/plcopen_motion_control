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
 * @file execution_node.hpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#pragma once

#include <RTmotion/algorithm/planner.hpp>
#include <RTmotion/fb/fb_axis_node.hpp>

namespace RTmotion
{
class ExecutionNode
{
public:

  ExecutionNode(double f);
  ExecutionNode(FbAxisNode* fb, MC_MOTION_MODE mode, LREAL position,
                LREAL velocity, LREAL acceleration, LREAL deceleration,
                LREAL jerk, MC_BUFFER_MODE buffer_mode, double f);
  virtual ~ExecutionNode()
  {
    fb_ = nullptr;
  }

  void set(FbAxisNode* fb, MC_MOTION_MODE mode, LREAL position,
           LREAL velocity, LREAL acceleration, LREAL deceleration,
           LREAL jerk, MC_BUFFER_MODE buffer_mode);

  void reset();

  // Functions to address inputs
  void setStartPos(double pos);
  void setStartVel(double vel);
  void setTerminateCondition(double ref_pos);
  void setEndVel(double vel);
  void setReplan();
  void setVelocity(LREAL velocity);

  // Functions to address ouputs
  MC_BUFFER_MODE getBufferMode();
  MC_MOTION_MODE getMotionMode();
  double getEndPos();
  double getEndVel();
  double getVelocity();
  void getCommands(double* pos_cmd, double* vel_cmd, double duration);

  bool isActive();
  bool isDone();
  bool isAborted();
  bool isError();

  // Functions for working on different status
  virtual void onActive(double t, MC_AXIS_STATES* current_state);
  virtual void onExecution(double t);
  virtual void onDone(MC_AXIS_STATES* current_state);
  virtual void onCommandAborted();
  virtual void onError(MC_ERROR_CODE error_code);

  // Function for checking mission done
  virtual bool checkMissionDone(const double axis_pos, const double axis_vel);

  void setPlannerStartTime(double t);

  MC_ERROR_CODE changeAxisStates(MC_AXIS_STATES* current_state, MC_AXIS_STATES set_state);

  FbAxisNode* getAxisNodeInstance();

  bool taken_ = false;
  
protected:
  VAR_INPUT LREAL position_;
  VAR_INPUT LREAL velocity_;
  VAR_INPUT LREAL acceleration_;
  VAR_INPUT LREAL deceleration_;
  VAR_INPUT LREAL jerk_;
  VAR_INPUT MC_BUFFER_MODE buffer_mode_;

  VAR_OUTPUT BOOL done_;
  VAR_OUTPUT BOOL busy_;
  VAR_OUTPUT BOOL active_;
  VAR_OUTPUT BOOL command_aborted_;
  VAR_OUTPUT BOOL error_;
  VAR_OUTPUT MC_ERROR_CODE error_id_;

  FbAxisNode* fb_;

  // Variables for trajectory planning
  double start_pos_;
  double end_pos_;
  double start_vel_;
  double end_vel_;
  double pos_cmd_;
  double vel_cmd_;
  double pos_tmp_;
  double vel_tmp_;
  bool need_plan_;
  MC_MOTION_MODE motion_mode_;

  // Trajectory generator variables
  trajectory_processing::AxisPlanner planner_;
};

}  // namespace RTmotion