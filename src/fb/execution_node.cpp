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
 * @file execution_node.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include <RTmotion/fb/execution_node.hpp>
#include <RTmotion/logging.hpp>

namespace RTmotion
{
ExecutionNode::ExecutionNode(double f)
  : done_(false)
  , busy_(false)
  , active_(false)
  , command_aborted_(false)
  , error_(false)
  , error_id_(mcErrorCodeGood)
  , start_pos_(0)
  , end_pos_(0)
  , start_vel_(0)
  , end_vel_(0)
  , need_plan_(false)
  , planner_(f)
{
  pos_cmd_ = 0;
  vel_cmd_ = 0;
  pos_tmp_ = 0;
  vel_tmp_ = 0;

  fb_ = nullptr;
  motion_mode_ = mcNoMoveType;
  position_ = 0;
  velocity_ = 0;
  acceleration_ = 0;
  deceleration_ = 0;
  jerk_ = 0;
  buffer_mode_ = mcAborting;
}

ExecutionNode::ExecutionNode(FbAxisNode* fb, MC_MOTION_MODE mode,
                             LREAL position, LREAL velocity, LREAL acceleration,
                             LREAL deceleration, LREAL jerk,
                             MC_BUFFER_MODE buffer_mode, double f)
  : done_(false)
  , busy_(false)
  , active_(false)
  , command_aborted_(false)
  , error_(false)
  , error_id_(mcErrorCodeGood)
  , start_pos_(0)
  , end_pos_(0)
  , start_vel_(0)
  , end_vel_(0)
  , need_plan_(false)
  , planner_(f)
{
  fb_ = fb;
  motion_mode_ = mode;
  position_ = position;
  velocity_ = velocity;
  acceleration_ = acceleration;
  deceleration_ = deceleration;
  jerk_ = jerk;
  buffer_mode_ = buffer_mode;
}

void ExecutionNode::set(FbAxisNode* fb, MC_MOTION_MODE mode,
          LREAL position, LREAL velocity, LREAL acceleration,
          LREAL deceleration, LREAL jerk,
          MC_BUFFER_MODE buffer_mode)
{
  done_ = false;
  busy_ = false;
  active_ = false;
  command_aborted_ = false;
  error_ = false;
  error_id_ = mcErrorCodeGood;
  start_pos_ = 0;
  end_pos_ = 0;
  start_vel_ = 0;
  end_vel_ = 0;
  need_plan_ = false;

  fb_ = fb;
  motion_mode_ = mode;
  position_ = position;
  velocity_ = velocity;
  acceleration_ = acceleration;
  deceleration_ = deceleration;
  jerk_ = jerk;
  buffer_mode_ = buffer_mode;
}

void ExecutionNode::reset()
{
  done_ = false;
  busy_ = false;
  active_ = false;
  command_aborted_ = false;
  error_ = false;
  error_id_ = mcErrorCodeGood;

  start_pos_ = 0;
  end_pos_ = 0;
  start_vel_ = 0;
  end_vel_ = 0;
  pos_cmd_ = 0;
  vel_cmd_ = 0;
  pos_tmp_ = 0;
  vel_tmp_ = 0;
  need_plan_ = false;
  motion_mode_ = mcNoMoveType;

  fb_ = nullptr;
  position_ = 0;
  velocity_ = 0;
  acceleration_ = 0;
  deceleration_ = 0;
  jerk_ = 0;
  buffer_mode_ = mcAborting;
}

void ExecutionNode::setStartPos(double pos)
{
  start_pos_ = pos;
  pos_tmp_ = pos;
  pos_cmd_ = pos;
}

void ExecutionNode::setStartVel(double vel)
{
  start_vel_ = vel;
  vel_tmp_ = vel;
  vel_cmd_ = vel;
}

void ExecutionNode::setTerminateCondition(double ref_pos)
{
  switch (motion_mode_)
  {
    case mcMoveAbsolute:
    {
      end_pos_ = position_;
      end_vel_ = 0.0;
    }
    break;
    case mcMoveRelative:
    {
      end_pos_ = start_pos_ + position_;
      end_vel_ = 0.0;
      // printf("start_pos_: %f, end_pos_: %f\n", start_pos_, end_pos_);
    }
    break;
    case mcMoveAdditive:
    {
      end_pos_ = ref_pos + position_;
      end_vel_ = 0.0;
    }
    break;
    case mcHalt:
    case mcStop:
    {
      end_pos_ = NAN;
      end_vel_ = 0.0;
    }
    break;
    case mcMoveVelocity:
    {
      end_pos_ = NAN;
      end_vel_ = velocity_;
    }
    break;
    default:
      break;
  }
}

void ExecutionNode::setEndVel(double vel)
{
  end_vel_ = vel;
}

void ExecutionNode::setReplan()
{
  need_plan_ = true;
}

void ExecutionNode::setVelocity(LREAL velocity)
{
  velocity_ = velocity;
}

MC_BUFFER_MODE ExecutionNode::getBufferMode()
{
  return buffer_mode_;
}

MC_MOTION_MODE ExecutionNode::getMotionMode()
{
  return motion_mode_;
}

double ExecutionNode::getEndPos()
{
  return end_pos_;
}

double ExecutionNode::getEndVel()
{
  return end_vel_;
}

double ExecutionNode::getVelocity()
{
  return velocity_;
}

void ExecutionNode::getCommands(double* pos_cmd, double* vel_cmd,
                                double duration)
{
  switch (motion_mode_)
  {
    case mcMoveAbsolute:
    case mcMoveAdditive:
    case mcMoveRelative:
    {
      pos_cmd_ = pos_tmp_;
      vel_cmd_ = vel_tmp_;
    }
    break;
    case mcHalt:
    case mcStop:
    case mcMoveVelocity:
    {
      vel_cmd_ = vel_tmp_;
      pos_cmd_ += vel_tmp_ * duration;
    }
    default:
      break;
  }
  *pos_cmd = pos_cmd_;
  *vel_cmd = vel_cmd_;
}

bool ExecutionNode::isActive()
{
  return active_;
}

bool ExecutionNode::isDone()
{
  return done_;
}

bool ExecutionNode::isAborted()
{
  return command_aborted_;
}

bool ExecutionNode::isError()
{
  return error_;
}

void ExecutionNode::onActive(double t, MC_AXIS_STATES* current_state)
{
  setPlannerStartTime(t);
  MC_ERROR_CODE res = mcErrorCodeGood;
  switch (motion_mode_)
  {
    case mcMoveAbsolute:
    case mcMoveAdditive:
    case mcMoveRelative:
    case mcHalt:
    {
      res = changeAxisStates(current_state, mcDiscreteMotion);
    }
    break;
    case mcMoveVelocity:
    {
      res = changeAxisStates(current_state, mcContinuousMotion);
      DEBUG_PRINT("ExecutionNode::onActive %d\n", res);
    }
    break;
    case mcStop:
    {
      res = changeAxisStates(current_state, mcStopping);
    }
    default:
      break;
  }

  if (res)
    onError(mcErrorCode_AxisStateViolation);
  else
  {
    active_ = busy_ = true;
    done_ = command_aborted_ = error_ = false;
    fb_->syncStatus(done_, busy_, active_, command_aborted_, error_, error_id_);
  }
}

void ExecutionNode::onExecution(double t)
{
  // Replan trajectory
  if (need_plan_)
  {
    need_plan_ = false;
    planner_.setCondition(start_pos_, end_pos_, start_vel_, end_vel_, velocity_,
                          acceleration_, jerk_);
    DEBUG_PRINT("ExecutionNode::onExecution:"
                "start_pos_ %f, end_pos_ %f, start_vel_ %f, end_vel_ %f,"
                "velocity_ %f, acceleration_ %f, jerk_ %f\n",
                start_pos_, end_pos_, start_vel_, end_vel_, velocity_,
                acceleration_, jerk_);
    MC_ERROR_CODE res = planner_.onReplan();
    if (res)
      onError(res);
  }

  // Compute waypoint
  planner_.onExecution(t, &pos_tmp_, &vel_tmp_);

  fb_->syncStatus(done_, busy_, active_, command_aborted_, error_, error_id_);
}

void ExecutionNode::onDone(MC_AXIS_STATES* current_state)
{
  MC_ERROR_CODE res = mcErrorCodeGood;
  switch (motion_mode_)
  {
    case mcMoveAbsolute:
    case mcMoveAdditive:
    case mcMoveRelative:
    case mcHalt:
    {
      res = changeAxisStates(current_state, mcStandstill);
    }
    break;
    case mcMoveVelocity:
    {
      res = changeAxisStates(current_state, mcContinuousMotion);
    }
    break;
    case mcStop:
    {
      if (fb_->isEnabled())
        res = changeAxisStates(current_state, mcStopping);
      else
        res = changeAxisStates(current_state, mcStandstill);
    }
    default:
      break;
  }
  
  if (res)
    onError(res);
  else
  {
    done_ = true;
    active_ = busy_ = command_aborted_ = error_ = false;
    fb_->syncStatus(done_, busy_, active_, command_aborted_, error_, error_id_);
  }
  
}

void ExecutionNode::onCommandAborted()
{
  command_aborted_ = true;
  active_ = busy_ = done_ = error_ = false;
  fb_->syncStatus(done_, busy_, active_, command_aborted_, error_, error_id_);
}

void ExecutionNode::onError(MC_ERROR_CODE error_code)
{
  error_ = true;
  active_ = busy_ = done_ = command_aborted_ = false;
  error_id_ = error_code;
  fb_->syncStatus(done_, busy_, active_, command_aborted_, error_, error_id_);
  DEBUG_PRINT("ExecutionNode::onError %s\n", fb_->isError() ? "true" : "false" );
}

bool ExecutionNode::checkMissionDone(const double axis_pos, const double axis_vel)
{
  bool res = false;
  switch (motion_mode_)
  {
    case mcMoveAbsolute:
    case mcMoveAdditive:
    case mcMoveRelative:
    {
      if (fabs(axis_pos - end_pos_) < fabs(start_pos_ -  end_pos_) * 0.015 &&
          fabs(axis_vel - end_vel_) < __EPSILON)
        res = true;
      DEBUG_PRINT("ExecutionNode::checkMissionDone: \n"
      "\tfabs(axis_pos - end_pos_): %f, fabs(start_pos_ -  end_pos_) * 0.015: %f, \n"
      "\tfabs(axis_vel - end_vel_): %f, __EPSILON: %f\n", fabs(axis_pos - end_pos_), fabs(start_pos_ -  end_pos_) * 0.015,
      fabs(axis_vel - end_vel_), __EPSILON);
    }
    break;
    case mcHalt:
    case mcStop:
    case mcMoveVelocity:
    {
      if (fabs(axis_vel - end_vel_) < 0.000001)
        res = true;
    }
    default:
      break;
  }
  return res;
}

void ExecutionNode::setPlannerStartTime(double t)
{
  planner_.setStartTime(t);
}

MC_ERROR_CODE ExecutionNode::changeAxisStates(MC_AXIS_STATES* current_state, MC_AXIS_STATES set_state)
{
  switch (*current_state)
  {
    case mcDisabled:
    case mcErrorStop:
      {
        return mcErrorCode_AxisStateViolation;
      }
      break;
    case mcStandstill:
      switch (set_state)
      {
        case mcErrorStop:
        case mcStandstill:
        case mcHoming:
        case mcStopping:
        case mcDiscreteMotion:
        case mcContinuousMotion:
        {
          *current_state = set_state;
          return mcErrorCodeGood;
        }
          break;
        default:
          return mcErrorCode_AxisStateViolation;
          break;
      }
      break;
    case mcHoming:
      switch (set_state)
      {
        case mcErrorStop:
        case mcHoming:
        case mcStopping:
        case mcStandstill:
        {
          *current_state = set_state;
          return mcErrorCodeGood;
        }
          break;
        default:
          return mcErrorCode_AxisStateViolation;
          break;
      }
      break;
    case mcStopping:
      switch (set_state)
      {
        case mcErrorStop:
        case mcStopping:
        case mcStandstill:
        {
          *current_state = set_state;
          return mcErrorCodeGood;
        }
          break;
        default:
          return mcErrorCode_AxisStateViolation;
          break;
      }
      break;
    case mcDiscreteMotion:
      switch (set_state)
      {
        case mcErrorStop:
        case mcStopping:
        case mcStandstill:
        case mcDiscreteMotion:
        case mcContinuousMotion:
          {
            *current_state = set_state;
            return mcErrorCodeGood;
          }
          break;
        default:
          return mcErrorCode_AxisStateViolation;
          break;
      }
      break;
    case mcContinuousMotion:
      switch (set_state)
        {
        case mcErrorStop:
        case mcStopping:
        case mcContinuousMotion:
        case mcDiscreteMotion:
          {
            *current_state = set_state;
            return mcErrorCodeGood;
          }
          break;
        default:
          return mcErrorCode_AxisStateViolation;
          break;
        }
        break;
      default:
        break;
  }
  return mcErrorCodeGood;
}

FbAxisNode* ExecutionNode::getAxisNodeInstance()
{
  return fb_;
}

}  // namespace RTmotion