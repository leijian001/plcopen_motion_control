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
 * @file axis.hpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#pragma once

#include <RTmotion/global.hpp>
#include <RTmotion/servo.hpp>
#include <queue>
#include <RTmotion/motion_kernel.hpp>

namespace RTmotion
{
class FbAxisMotion;
class MotionKernel;

struct AxisConfig
{
  MC_SERVO_CONTROL_MODE mode_ = mcServoControlModePosition;
  uint64_t encoder_count_per_unit_ = 1000000;
  uint64_t node_buffer_size_ = 100;
  bool sw_vel_limit_ = false;
  double vel_limit_ = 5000.0;
  bool sw_acc_limit_ = false;
  double acc_limit_ = 5000.0;
  bool sw_range_limit_ = false;
  double pos_positive_limit_ = 5000.0;
  double pos_negative_limit_ = 5000.0;
  double frequency_ = 1000.0;
};

class Axis
{
public:
  Axis();
  virtual ~Axis();

  int32_t axisId();

  std::string axisName();

  MC_ERROR_CODE setAxisId(int32_t id);

  MC_ERROR_CODE setAxisName(std::string name);

  std::shared_ptr<Servo>& getServo()
  {
    return servo_;
  };

  virtual void setServo(std::shared_ptr<Servo>& servo);

  virtual void setAxisConfig(AxisConfig* config);

  virtual void runCycle();

  virtual double toUserPos();
  virtual double toUserVel();

  virtual double toUserPosCmd();
  virtual double toUserVelCmd();

  virtual double toUserUnit(double x);
  virtual int32_t toEncoderUnit(double x);

  virtual void addFBToQueue(FbAxisNode* fb, MC_MOTION_MODE mode);

  virtual BOOL statusHealthy();

  virtual void powerProcess();

  virtual bool cmdsProcessing(double frequency);

  virtual void syncMotionKernelResultsToAxis(double duration);

  virtual void updateMotionCmdsToServo();

  virtual void statusSync();

  double fixOverFlow(double x);

  MC_AXIS_STATES getAxisState(void);

  MC_ERROR_CODE setAxisState(MC_AXIS_STATES set_state);

  void setPower(bool power_on, bool enable_positive, bool enable_negative);

  void resetError(bool reset);

  bool powerOn();
  bool powerTriggered();

  MC_ERROR_CODE getAxisError();

  virtual MC_ERROR_CODE servoErrorToAxisError(MC_ServoErrorCode error_id);

private:
  int32_t axis_id_;
  std::string axis_name_;
  double axis_pos_;
  double axis_vel_;
  double axis_acc_;
  double axis_jerk_;
  double axis_pos_cmd_;
  double axis_vel_cmd_;
  double axis_tor_cmd_;
  double overflow_count_;
  MC_AXIS_STATES axis_state_;
  std::shared_ptr<Servo> servo_;

  MotionKernel* motion_kernel_;

  AxisConfig* config_;

  struct timespec time_start_;
  struct timespec time_prev_;

  bool power_on_;
  bool power_status_;
  bool reset_;
  bool enable_positive_;
  bool enable_negative_;

  bool add_fb_ = false;
  uint32_t count_ = 0;
  uint32_t add_count_ = 0;
  double stamp_ = 0;

  MC_ERROR_CODE axis_error_;

  std::vector<ExecutionNode*> node_buffer_;
};

typedef std::shared_ptr<Axis> AXIS_REF;

}  // namespace RTmotion
