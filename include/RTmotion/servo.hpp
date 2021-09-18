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
 * @file servo.hpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#pragma once

#include <RTmotion/global.hpp>

namespace RTmotion
{
class Servo
{
public:
  Servo();
  virtual ~Servo();

  virtual MC_ServoErrorCode setPower(bool powerStatus, bool& isDone);
  virtual MC_ServoErrorCode setPos(int32_t pos);
  virtual MC_ServoErrorCode setVel(int32_t vel);
  virtual MC_ServoErrorCode setTorque(double torque);
  virtual int32_t pos(void);
  virtual int32_t vel(void);
  virtual int32_t acc(void);
  virtual double torque(void);
  virtual bool readVal(int index, double& value);
  virtual bool writeVal(int index, double value);
  virtual MC_ServoErrorCode resetError(bool& isDone);
  virtual void runCycle(double freq);
  virtual void emergStop(void);

private:
  class ServoImpl;
  ServoImpl* mImpl_;
};
}  // namespace RTmotion
