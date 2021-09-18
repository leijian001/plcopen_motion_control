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
 * @file axis.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include <RTmotion/axis.hpp>
#include <algorithm>
#include <RTmotion/logging.hpp>

namespace RTmotion
{
Axis::Axis()
  : axis_id_(0)
  , axis_name_("")
  , axis_pos_(0)
  , axis_vel_(0)
  , axis_acc_(0)
  , axis_jerk_(0)
  , axis_pos_cmd_(0)
  , axis_vel_cmd_(0)
  , axis_tor_cmd_(0)
  , overflow_count_(0)
  , axis_state_(mcStandstill)
  , servo_(new Servo())
  , motion_kernel_(new MotionKernel())
  , config_(new AxisConfig())
  , power_on_(false)
  , power_status_(false)
  , reset_(false)
  , enable_positive_(false)
  , enable_negative_(false)
  , axis_error_(mcErrorCodeGood)
{
  clock_gettime(CLOCK_MONOTONIC, &time_start_);
  time_prev_ = time_start_;
  node_buffer_.resize(config_->node_buffer_size_);
  for (size_t i = 0; i < node_buffer_.size(); i++)
  {
    node_buffer_[i] = new ExecutionNode(1000);
    node_buffer_[i]->taken_ = false;
  }
}

Axis::~Axis()
{
  servo_ = nullptr;
  delete motion_kernel_;
  motion_kernel_ = nullptr;
  config_ = nullptr;
}

int32_t Axis::axisId(void)
{
  return axis_id_;
}

std::string Axis::axisName()
{
  return axis_name_;
}

MC_ERROR_CODE Axis::setAxisId(int32_t id)
{
  axis_id_ = id;
  return mcErrorCodeGood;
}

MC_ERROR_CODE Axis::setAxisName(std::string name)
{
  axis_name_ = name;
  return mcErrorCodeGood;
}

void Axis::setServo(std::shared_ptr<Servo>& servo)
{
  servo_ = servo;
}

void Axis::setAxisConfig(AxisConfig* config)
{
  config_ = config;
  for (size_t i = 0; i < node_buffer_.size(); i++)
  {
    delete node_buffer_[i];
  }
  node_buffer_.resize(config_->node_buffer_size_);
  for (size_t i = 0; i < node_buffer_.size(); i++)
  {
    node_buffer_[i] = new ExecutionNode(config_->frequency_);
    node_buffer_[i]->taken_ = false;
  }
}

void Axis::runCycle()
{
  // Update time stamps
  count_ += 1;
  struct timespec time_now;
  clock_gettime(CLOCK_MONOTONIC, &time_now);
  double t = DIFF_NS(time_start_, time_now) / (double)NSEC_PER_SEC;
  double d = DIFF_NS(time_prev_, time_now) / (double)NSEC_PER_SEC;
  stamp_ = t;

  powerProcess(); // Make power operations

  if (statusHealthy()) // If axis status ok, then do motion calculation
  {
    // Motion kernel update
    motion_kernel_->runCycle(t, t - d, axis_pos_, axis_vel_cmd_, &axis_state_);

    // Sync motion kernel results to axis
    syncMotionKernelResultsToAxis(d);

    // Axis commands processing
    cmdsProcessing(1.0 / d);

    // Update motion commands to servo
    updateMotionCmdsToServo();
  }

  // Servo updates
  servo_->runCycle(1.0 / d);

  // Sync servo status to axis
  statusSync();

  time_prev_ = time_now;
  if (add_fb_)
    add_count_ += 1;
  if (add_count_ == 100)
  {
    add_fb_ = false;
    add_count_ = 0;
  }
}

double Axis::toUserPos()
{
  return axis_pos_;
}

double Axis::toUserVel()
{
  return axis_vel_;
}

double Axis::toUserPosCmd()
{
  return axis_pos_cmd_;
}

double Axis::toUserVelCmd()
{
  return axis_vel_cmd_;
}

double Axis::toUserUnit(double x)
{
  DEBUG_PRINT("Axis::toUserUnit: %f\n", x);
  return x / config_->encoder_count_per_unit_;
}

int32_t Axis::toEncoderUnit(double x)
{
  return (int32_t)fixOverFlow(x * config_->encoder_count_per_unit_);
}

void Axis::addFBToQueue(FbAxisNode* fb, MC_MOTION_MODE mode)
{
  add_fb_ = true;
  std::vector<ExecutionNode*>::iterator it = 
        std::find_if(node_buffer_.begin(), node_buffer_.end(), 
                        [](ExecutionNode* s){ return s && s->taken_ == false; });
  DEBUG_PRINT("Debug %ld\n", it - node_buffer_.begin());
  if (it != node_buffer_.end())
  {
    (*it)->reset();
    (*it)->set(fb, mode, fb->getPosition(), fb->getVelocity(),
                fb->getAcceleration(), fb->getDeceleration(),
                fb->getJerk(), fb->getBufferMode());
    (*it)->taken_ = true;

    motion_kernel_->addFBToQueue(*it, toUserPosCmd(), toUserVelCmd());
  }
}

BOOL Axis::statusHealthy()
{
  // Check axis error and power status
  if (axis_error_ || !power_status_)
  {
    motion_kernel_->setAllFBsAborted();
    DEBUG_PRINT("Axis::statusHealthy axis_error_: 0x%X\n", axis_error_);
    return false; 
  }

  return true;
}

void Axis::powerProcess()
{
  if (axis_error_) // If there is axis error
  {
    setAxisState(mcErrorStop); // Set axis state to Error Stop
    if (reset_)
    {
      bool isDone = false;
      servo_->resetError(isDone);
      if (isDone) // If servo reset successfully, i.e. no servo error, reset error code, power state and axis state
      {
        axis_error_ = mcErrorCodeGood;
        power_status_ = false;
        setAxisState(mcDisabled);
        reset_ = false;
      }
    }
    return;
  }

  // If there is no axis error, process servo power status
  bool isDone = false;
  MC_ERROR_CODE servo_res = servoErrorToAxisError(servo_->setPower(power_on_, isDone));

  // Power On
  if (power_on_ && !power_status_)
  {
    // Error happen when powering on
    if (servo_res != mcErrorCode_Servo_No_Error)
    {
      power_status_ = false;
      axis_error_ = servo_res;
      DEBUG_PRINT("axis_state_ 01: %d\n", axis_state_);
      setAxisState(mcErrorStop);
      DEBUG_PRINT("axis_state_ 02: %d\n", axis_state_);
      DEBUG_PRINT("Powering on error 0x%X\n", servo_res);
      return;
    }
    // Powered on successfully
    if (isDone)
    {
      power_status_ = true;
      axis_vel_cmd_ = axis_vel_ = 0.0;
      axis_tor_cmd_ = axis_acc_ = 0.0;
      axis_pos_cmd_ = axis_pos_ = toUserUnit(servo_->pos());
      INFO_PRINT("Powered On axis_pos_cmd_ = axis_pos_ = %f\n", toUserUnit(servo_->pos()));
      MC_ERROR_CODE res = setAxisState(mcStandstill);
      if(res != mcErrorCodeGood)
        axis_error_ = res;
      return;
    }
  }

  // When powered on
  if (power_on_ && power_status_)
  {
    // Error happen when powered on
    if (servo_res != mcErrorCode_Servo_No_Error)
    {
      power_status_ = false;
      axis_error_ = servo_res;
      DEBUG_PRINT("axis_state_ 01: %d\n", axis_state_);
      setAxisState(mcErrorStop);
      DEBUG_PRINT("axis_state_ 02: %d\n", axis_state_);
      DEBUG_PRINT("Powered on error 0x%X\n", servo_res);
      return;
    }
  }

  // Power Off
  if (!power_on_ && power_status_)
  {
    // Error happen when powering off
    if (servo_res != mcErrorCode_Servo_No_Error)
    {
      power_status_ = false;
      axis_error_ = servo_res;
      DEBUG_PRINT("axis_state_ 01: %d\n", axis_state_);
      setAxisState(mcErrorStop);
      DEBUG_PRINT("axis_state_ 02: %d\n", axis_state_);
      DEBUG_PRINT("Powering off error 0x%X\n", servo_res);
      return;
    }

    MC_ERROR_CODE res = setAxisState(mcDisabled);
    if(res != mcErrorCodeGood)
    {
      axis_error_ = res;
      return;
    }

    // Powered off successfully
    if (isDone)
    {
      power_status_ = false;
      axis_vel_cmd_ = axis_vel_ = 0.0;
      axis_tor_cmd_ = axis_acc_ = 0.0;
      axis_pos_cmd_ = axis_pos_ = toUserUnit(servo_->pos());
      DEBUG_PRINT("Powered Off axis_pos_cmd_ = axis_pos_ = %f\n", toUserUnit(servo_->pos()));
      return;
    }
  }
}

bool Axis::cmdsProcessing(double frequency)
{
  // Check motion direction and limits
  double vel_cmd = (axis_pos_cmd_ - axis_pos_) * frequency;
  double acc_cmd = (axis_vel_cmd_ - axis_vel_) * frequency;
  DEBUG_PRINT("Axis::cmdsProcessing: axis_pos_ %f, axis_vel_ %f\n", axis_pos_, axis_vel_);
  DEBUG_PRINT("Axis::cmdsProcessing: axis_pos_cmd_ %f, axis_vel_cmd_ %f\n", axis_pos_cmd_, axis_vel_cmd_);
  DEBUG_PRINT("Axis::cmdsProcessing: vel_cmd %f, acc_cmd %f\n", vel_cmd, acc_cmd);

  if(vel_cmd > 0 && !enable_positive_)
  {
    axis_error_ = mcErrorCode_Invalid_Direction_Positive;
    return false;
  } else if(vel_cmd < 0 && !enable_negative_)
  {
    axis_error_ = mcErrorCode_Invalid_Direction_Negative;
    return false;
  }

  if (config_->sw_vel_limit_ && fabs(vel_cmd) > config_->vel_limit_)
  {
    axis_error_ = mcErrorCode_Velocity_Over_Limit;
    return false;
  }

  if (config_->sw_acc_limit_ && fabs(acc_cmd) > config_->acc_limit_)
  {
    axis_error_ = mcErrorCode_Acceleration_Over_Limit;
    return false;
  }

  if(config_->sw_range_limit_ && axis_pos_cmd_ > config_->pos_positive_limit_ && vel_cmd > 0)
  {
    axis_error_ = mcErrorCode_Position_Over_Positive_Limit;
    return false;
  }

  if(config_->sw_range_limit_ && axis_pos_cmd_ < config_->pos_negative_limit_ && vel_cmd < 0)
  {
    axis_error_ = mcErrorCode_Position_Over_Negative_Limit;
    return false;
  }

  // Process home position offset

  return true;
}

void Axis::syncMotionKernelResultsToAxis(double duration)
{
  motion_kernel_->getCommands(&axis_pos_cmd_, &axis_vel_cmd_, duration);
  DEBUG_PRINT("Axis::syncMotionKernelResultsToAxis: axis_pos_cmd_: %f, axis_vel_cmd_: %f \n", axis_pos_cmd_,
              axis_vel_cmd_);
}

void Axis::updateMotionCmdsToServo()
{
  if (config_->mode_ == mcServoControlModePosition)
    servo_->setPos(toEncoderUnit(axis_pos_cmd_));
  if (config_->mode_ == mcServoControlModeVelocity)
    servo_->setVel(toEncoderUnit(axis_vel_cmd_));
  if (add_fb_ && add_count_ < 100)
    DEBUG_PRINT("T: %f, C: %u, Axis::updateMotionCmdsToServo: axis_pos_cmd_: %f, toEncoderUnit(axis_pos_cmd_): %d\n",
        stamp_, count_, axis_pos_cmd_, toEncoderUnit(axis_pos_cmd_));
}

void Axis::statusSync()
{
  // Update servo state to axis
  axis_pos_ = toUserUnit(servo_->pos() - overflow_count_ * __INT32_MAX__ * 2.0);
  axis_vel_ = toUserUnit(servo_->vel());
  if (add_fb_ && add_count_ < 100)
    DEBUG_PRINT("T: %f, C: %u, Axis::statusSync: axis_pos_: %f, axis_vel_: %f \n", stamp_, count_, axis_pos_, axis_vel_);
}

double Axis::fixOverFlow(double x)
{
  x += overflow_count_ * __INT32_MAX__ * 2.0;
  if (x >= __INT32_MAX__)
  {
    x -= __INT32_MAX__ * 2.0;
    overflow_count_ -= 1.0;
  }
  else if (x <= - __INT32_MAX__)
  {
    x += __INT32_MAX__ * 2.0;
    overflow_count_ += 1.0;
  }

  return x;
}

MC_AXIS_STATES Axis::getAxisState(void)
{
  return axis_state_;
}

MC_ERROR_CODE Axis::setAxisState(MC_AXIS_STATES set_state)
{
  switch (axis_state_)
  {
  case mcStandstill:
  case mcHoming:
  case mcDiscreteMotion:
  case mcContinuousMotion:
    switch (set_state)
    {
    case mcDisabled:
    case mcErrorStop:
      {
        axis_state_ = set_state;
        return mcErrorCodeGood;
      }
      break;
    default:
      break;
    }
    break;
  case mcStopping:
    switch (set_state)
    {
      case mcStopping:
      case mcDisabled:
      case mcErrorStop:
      case mcStandstill:
        {
          axis_state_ = set_state;
          return mcErrorCodeGood;
        }
        break;
      default:
        return mcErrorCode_InvalidState_FromStopping;
        break;
    }
    break;
  case mcErrorStop:
    switch (set_state)
    {
      case mcErrorStop:
      case mcDisabled:
      case mcStandstill:
        {
          axis_state_ = set_state;
          return mcErrorCodeGood;
        }
        break;
      default:
        return mcErrorCode_InvalidState_FromErrorStop;
        break;
    }
    break;
  case mcDisabled:
    switch (set_state)
    {
      case mcDisabled:
      case mcErrorStop:
      case mcStandstill:
        {
          axis_state_ = set_state;
          return mcErrorCodeGood;
        }
        break;
      default:
        return mcErrorCode_InvalidState_FromDisabled;
        break;
    }
    break;    
  default:
    break;
  }
  return mcErrorCodeGood;
}

void Axis::setPower(bool power_on, bool enable_positive, bool enable_negative)
{
  power_on_ = power_on;
  enable_positive_ = enable_positive;
  enable_negative_ = enable_negative;
}

void Axis::resetError(bool reset)
{
  reset_ = reset;
}

bool Axis::powerOn()
{
  return power_status_;
}

bool Axis::powerTriggered()
{
  return power_on_;
}

MC_ERROR_CODE Axis::getAxisError()
{
  return axis_error_;
}

MC_ERROR_CODE Axis::servoErrorToAxisError(MC_ServoErrorCode error_id)
{
  return static_cast<MC_ERROR_CODE>(0x60 + error_id);
}

}  // namespace RTmotion
