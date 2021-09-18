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
 * @file uranus_test.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include "gtest/gtest.h"
#include <axis/Axis.hpp>
#include <Scheduler.hpp>
#include <FbSingleAxis.hpp>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <chrono>
#include <thread>

#include <RTmotion/tool/fb_debug_tool.hpp>

using namespace Uranus;
using namespace std::chrono_literals;

class FunctionBlockTest : public ::testing::Test
{
protected:
  // You can remove any or all of the following functions if their bodies would
  // be empty.

  FunctionBlockTest()
  {
    frequency_ = 1000;
    axisId_ = 1;
    sched_.setFrequency(frequency_);
    axis_ = sched_.newAxis(axisId_, new Servo());

    power_.mAxis = axis_;
    power_.mEnable = true;
    power_.mEnablePositive = true;
    power_.mEnableNegative = true;

    readPos_.mAxis = axis_;
    readPos_.mEnable = true;

    readVel_.mAxis = axis_;
    readVel_.mEnable = true;
  }

  ~FunctionBlockTest() override
  {
    sched_.release();
    axis_ = nullptr;
    power_.mAxis = nullptr;
    readPos_.mAxis = nullptr;
    readVel_.mAxis = nullptr;
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  void SetUp() override
  {
    // Code here will be called immediately after the constructor (right
    // before each test).
  }

  void TearDown() override
  {
    // Code here will be called immediately after each test (right
    // before the destructor).
  }

  void resetAxis()
  {
    sched_.release();
    axis_ = sched_.newAxis(axisId_, new Servo());
    power_.mAxis = axis_;
    readPos_.mAxis = axis_;
    readVel_.mAxis = axis_;
  }

  Scheduler sched_;
  Axis* axis_;
  int32_t axisId_;
  FbPower power_;
  double frequency_;
  FbReadActualPosition readPos_;
  FbReadActualVelocity readVel_;
  AxisProfile axis_profile_;
  FBDigitalProfile fb_profile_;
};

// Test uranus demo
TEST_F(FunctionBlockTest, DemoTest)
{
  resetAxis();
  fb_profile_.reset();
  axis_profile_.reset();

  FbMoveAbsolute moveAbs1;
  moveAbs1.mAxis = axis_;
  moveAbs1.mPosition = 500;
  moveAbs1.mVelocity = 400;
  moveAbs1.mAcceleration = 500;
  moveAbs1.mDeceleration = 500;
  moveAbs1.mJerk = 5000;

  FbMoveAbsolute moveAbs2;
  moveAbs2.mAxis = axis_;
  moveAbs2.mPosition = 1000;
  moveAbs2.mVelocity = 200;
  moveAbs2.mAcceleration = 300;
  moveAbs2.mDeceleration = 300;
  moveAbs2.mJerk = 3000;
  moveAbs2.mBufferMode = MC_BUFFERMODE_BLENDINGNEXT;

  fb_profile_.addFB("moveAbs1");
  fb_profile_.addFB("moveAbs2");
  fb_profile_.addItemName("moveAbs1.execute");
  fb_profile_.addItemName("moveAbs1.busy");
  fb_profile_.addItemName("moveAbs1.active");
  fb_profile_.addItemName("moveAbs1.done");
  fb_profile_.addItemName("moveAbs1.error");
  fb_profile_.addItemName("moveAbs2.execute");
  fb_profile_.addItemName("moveAbs2.busy");
  fb_profile_.addItemName("moveAbs2.active");
  fb_profile_.addItemName("moveAbs2.done");
  fb_profile_.addItemName("moveAbs2.error");

  bool moveAbs1AlreadyDone = false;
  double time_out = 5;
  double t = 0;
  while (t < time_out)
  {
    sched_.runCycle();

    //功能块调用
    power_.call();
    moveAbs1.call();
    moveAbs2.call();
    readPos_.call();
    readVel_.call();

    bool isPowerOn = power_.mStatus && power_.mValid;
    if (isPowerOn && !moveAbs1.mExecute)
    {  //使能成功后move1开始移动
      std::cout << "axis poweron, moveAbs1 start" << std::endl;
      moveAbs1.mExecute = isPowerOn;
    }

    if (moveAbs1.mDone && !moveAbs1AlreadyDone)
    {  //提示move1完成
      std::cout << "moveAbs1 complete, moveAbs2 start" << std::endl;
      moveAbs1AlreadyDone = true;
    }

    moveAbs2.mExecute = moveAbs1.mBusy;

    axis_profile_.addState(readPos_.mPosition, readVel_.mVelocity, t);

    fb_profile_.addState("moveAbs1.execute", moveAbs1.mExecute);
    fb_profile_.addState("moveAbs1.busy", moveAbs1.mBusy);
    fb_profile_.addState("moveAbs1.active", moveAbs1.mActive);
    fb_profile_.addState("moveAbs1.done", moveAbs1.mDone);
    fb_profile_.addState("moveAbs1.error", moveAbs1.mError);
    fb_profile_.addState("moveAbs2.execute", moveAbs2.mExecute);
    fb_profile_.addState("moveAbs2.busy", moveAbs2.mBusy);
    fb_profile_.addState("moveAbs2.active", moveAbs2.mActive);
    fb_profile_.addState("moveAbs2.done", moveAbs2.mDone);
    fb_profile_.addState("moveAbs2.error", moveAbs2.mError);
    fb_profile_.addTime(t);

    t += 1 / frequency_;
  }

  axis_profile_.plot();
  fb_profile_.plot();
}

// Test MC_Stop
TEST_F(FunctionBlockTest, MC_Stop)
{
  resetAxis();
  fb_profile_.reset();
  axis_profile_.reset();

  FbMoveVelocity moveVel;
  moveVel.mAxis = axis_;
  moveVel.mVelocity = 50;
  moveVel.mAcceleration = 10;
  moveVel.mDeceleration = 10;
  moveVel.mJerk = 5000;

  FbStop stop;
  stop.mAxis = axis_;
  stop.mDeceleration = 20;
  stop.mJerk = 5000;

  fb_profile_.addFB("moveVel");
  fb_profile_.addFB("stop");
  fb_profile_.addItemName("moveVel.execute");
  fb_profile_.addItemName("moveVel.inVel");
  fb_profile_.addItemName("moveVel.abort");
  fb_profile_.addItemName("moveVel.error");
  fb_profile_.addItemName("stop.execute");
  fb_profile_.addItemName("stop.done");

  double time_out = 25;
  double t = 0;
  while (t < time_out)
  {
    sched_.runCycle();

    //功能块调用
    power_.call();
    moveVel.call();
    stop.call();
    readPos_.call();
    readVel_.call();

    bool isPowerOn = power_.mStatus && power_.mValid;
    if (isPowerOn && !moveVel.mExecute && t < 7.5)
    {  //使能成功后move1开始移动
      std::cout << "axis poweron, moveVel start" << std::endl;
      moveVel.mExecute = isPowerOn;
    }

    if (6 < t && t < 12.5 && !stop.mExecute)
      stop.mExecute = true;

    if (12.5 < t && stop.mExecute)
      stop.mExecute = false;

    if (7.5 < t && t < 10 && moveVel.mExecute)
      moveVel.mExecute = false;

    if (10 < t && t < 15 && !moveVel.mExecute)
      moveVel.mExecute = true;

    if (15 < t && t < 20 && moveVel.mExecute)
      moveVel.mExecute = false;

    if (17.5 < t && !moveVel.mExecute)
      moveVel.mExecute = true;

    axis_profile_.addState(readPos_.mPosition, readVel_.mVelocity, t);
    fb_profile_.addState("moveVel.execute", moveVel.mExecute);
    fb_profile_.addState("moveVel.inVel", moveVel.mInVelocity);
    fb_profile_.addState("moveVel.abort", moveVel.mCommandAborted);
    fb_profile_.addState("moveVel.error", moveVel.mError);
    fb_profile_.addState("stop.execute", stop.mExecute);
    fb_profile_.addState("stop.done", stop.mDone);
    fb_profile_.addTime(t);

    t += 1 / frequency_;
  }

  axis_profile_.plot("mc_stop_axis.png");
  fb_profile_.plot("mc_stop_fb.png");
}

// Test MC_Halt
TEST_F(FunctionBlockTest, MC_Halt)
{
  resetAxis();
  fb_profile_.reset();
  axis_profile_.reset();

  FbMoveVelocity moveVel;
  moveVel.mAxis = axis_;
  moveVel.mVelocity = 50;
  moveVel.mAcceleration = 10;
  moveVel.mDeceleration = 10;
  moveVel.mJerk = 0;

  FbHalt halt;
  halt.mAxis = axis_;
  halt.mDeceleration = 5;
  halt.mJerk = 0;

  fb_profile_.addFB("moveVel");
  fb_profile_.addFB("halt");
  fb_profile_.addItemName("moveVel.execute");
  fb_profile_.addItemName("moveVel.inVel");
  fb_profile_.addItemName("moveVel.abort");
  fb_profile_.addItemName("halt.execute");
  fb_profile_.addItemName("halt.done");
  fb_profile_.addItemName("halt.abort");

  double time_out = 40;
  double t = 0;
  while (t < time_out)
  {
    sched_.runCycle();

    //功能块调用
    power_.call();
    moveVel.call();
    halt.call();
    readPos_.call();
    readVel_.call();

    bool isPowerOn = power_.mStatus && power_.mValid;
    if (isPowerOn && !moveVel.mExecute && t < 7.5)
    {  //使能成功后move1开始移动
      std::cout << "axis poweron, moveVel start" << std::endl;
      moveVel.mExecute = isPowerOn;
    }

    if (8.5 < t && t < 19 && moveVel.mExecute)
      moveVel.mExecute = false;

    if (19 < t && t < 21.5 && !moveVel.mExecute)
      moveVel.mExecute = true;

    if (21.5 < t && t < 32.5 && moveVel.mExecute)
      moveVel.mExecute = false;

    if (32.5 < t && !moveVel.mExecute)
      moveVel.mExecute = true;

    if (7 < t && t < 17.5 && !halt.mExecute)
      halt.mExecute = true;

    if (17.5 < t && t < 27.5 && halt.mExecute)
      halt.mExecute = false;

    if (27.5 < t && t < 37.5 && !halt.mExecute)
      halt.mExecute = true;

    if (37.5 < t && halt.mExecute)
      halt.mExecute = false;

    axis_profile_.addState(readPos_.mPosition, readVel_.mVelocity, t);
    fb_profile_.addState("moveVel.execute", moveVel.mExecute);
    fb_profile_.addState("moveVel.inVel", moveVel.mInVelocity);
    fb_profile_.addState("moveVel.abort", moveVel.mCommandAborted);
    fb_profile_.addState("halt.execute", halt.mExecute);
    fb_profile_.addState("halt.done", halt.mDone);
    fb_profile_.addState("halt.abort", halt.mCommandAborted);
    fb_profile_.addTime(t);

    t += 1 / frequency_;
  }

  axis_profile_.plot("mc_halt_axis.png");
  fb_profile_.plot("mc_halt_fb.png");
}

// Test MC_MoveAbsolute
TEST_F(FunctionBlockTest, MC_MoveAbsolute)
{
  resetAxis();
  fb_profile_.reset();
  axis_profile_.reset();

  FbMoveAbsolute moveAbs1;
  moveAbs1.mAxis = axis_;
  moveAbs1.mPosition = 600;
  moveAbs1.mVelocity = 300;
  moveAbs1.mAcceleration = 500;
  moveAbs1.mDeceleration = 500;
  moveAbs1.mJerk = 5000;
  moveAbs1.mDirection = MC_DIRECTION_POSITIVE;

  FbMoveAbsolute moveAbs2;
  moveAbs2.mAxis = axis_;
  moveAbs2.mPosition = 1000;
  moveAbs2.mVelocity = 200;
  moveAbs2.mAcceleration = 500;
  moveAbs2.mDeceleration = 500;
  moveAbs2.mJerk = 5000;
  moveAbs2.mDirection = MC_DIRECTION_POSITIVE;
  moveAbs2.mBufferMode = MC_BUFFERMODE_ABORTING;

  fb_profile_.addFB("moveAbs1");
  fb_profile_.addFB("moveAbs2");
  fb_profile_.addItemName("moveAbs1.go");
  fb_profile_.addItemName("moveAbs1.done");
  fb_profile_.addItemName("moveAbs1.aborted");
  fb_profile_.addItemName("moveAbs2.test");
  fb_profile_.addItemName("moveAbs2.finish");

  double time_out = 7;
  double t = 0;
  while (t < time_out)
  {
    sched_.runCycle();

    //功能块调用
    power_.call();
    moveAbs1.call();
    moveAbs2.call();
    readPos_.call();
    readVel_.call();

    bool isPowerOn = power_.mStatus && power_.mValid;
    if (isPowerOn && !moveAbs1.mExecute && t < 2)
    {  //使能成功后move1开始移动
      std::cout << "axis poweron, moveVel start" << std::endl;
      moveAbs1.mExecute = isPowerOn;
    }

    if (2 < t && moveAbs1.mExecute)
      moveAbs1.mExecute = false;

    if (1 < t && t < 6 && !moveAbs2.mExecute)
      moveAbs2.mExecute = true;

    if (6 < t && moveAbs2.mExecute)
      moveAbs2.mExecute = false;

    axis_profile_.addState(readPos_.mPosition, readVel_.mVelocity, t);
    fb_profile_.addState("moveAbs1.go", moveAbs1.mExecute);
    fb_profile_.addState("moveAbs1.done", moveAbs1.mDone);
    fb_profile_.addState("moveAbs1.aborted", moveAbs1.mCommandAborted);
    fb_profile_.addState("moveAbs2.test", moveAbs2.mExecute);
    fb_profile_.addState("moveAbs2.finish", moveAbs2.mDone);
    fb_profile_.addTime(t);

    t += 1 / frequency_;
  }

  axis_profile_.plot("mc_moveAbs_axis.png");
  fb_profile_.plot("mc_moveAbs_fb.png");
}

// Test MC_MoveRelative
TEST_F(FunctionBlockTest, MC_MoveRelative)
{
  resetAxis();
  fb_profile_.reset();
  axis_profile_.reset();

  FbMoveRelative moveRel1;
  moveRel1.mAxis = axis_;
  moveRel1.mDistance = 600;
  moveRel1.mVelocity = 300;
  moveRel1.mAcceleration = 500;
  moveRel1.mDeceleration = 500;
  moveRel1.mJerk = 5000;

  FbMoveRelative moveRel2;
  moveRel2.mAxis = axis_;
  moveRel2.mDistance = 400;
  moveRel2.mVelocity = 200;
  moveRel2.mAcceleration = 500;
  moveRel2.mDeceleration = 500;
  moveRel2.mJerk = 5000;
  moveRel2.mBufferMode = MC_BUFFERMODE_ABORTING;

  fb_profile_.addFB("moveRel1");
  fb_profile_.addFB("moveRel2");
  fb_profile_.addItemName("moveRel1.go");
  fb_profile_.addItemName("moveRel1.done");
  fb_profile_.addItemName("moveRel1.aborted");
  fb_profile_.addItemName("moveRel2.test");
  fb_profile_.addItemName("moveRel2.finish");

  double time_out = 4;
  double t = 0;
  while (t < time_out)
  {
    sched_.runCycle();

    //功能块调用
    power_.call();
    moveRel1.call();
    moveRel2.call();
    readPos_.call();
    readVel_.call();

    bool isPowerOn = power_.mStatus && power_.mValid;
    if (isPowerOn && !moveRel1.mExecute && t < 2)
    {  //使能成功后move1开始移动
      std::cout << "axis poweron, moveVel start" << std::endl;
      moveRel1.mExecute = isPowerOn;
    }

    if (2 < t && moveRel1.mExecute)
      moveRel1.mExecute = false;

    if (1 < t && t < 3.8 && !moveRel2.mExecute)
      moveRel2.mExecute = true;

    if (3.8 < t && moveRel2.mExecute)
      moveRel2.mExecute = false;

    axis_profile_.addState(readPos_.mPosition, readVel_.mVelocity, t);
    fb_profile_.addState("moveRel1.go", moveRel1.mExecute);
    fb_profile_.addState("moveRel1.done", moveRel1.mDone);
    fb_profile_.addState("moveRel1.aborted", moveRel1.mCommandAborted);
    fb_profile_.addState("moveRel2.test", moveRel2.mExecute);
    fb_profile_.addState("moveRel2.finish", moveRel2.mDone);
    fb_profile_.addTime(t);

    t += 1 / frequency_;
  }

  axis_profile_.plot("mc_moveRel_axis.png");
  fb_profile_.plot("mc_moveRel_fb.png");
}

// Test MC_MoveAdditive
TEST_F(FunctionBlockTest, MC_MoveAdditive)
{
  resetAxis();
  fb_profile_.reset();
  axis_profile_.reset();

  FbMoveAbsolute moveAbs1;
  moveAbs1.mAxis = axis_;
  moveAbs1.mPosition = 600;
  moveAbs1.mVelocity = 300;
  moveAbs1.mAcceleration = 500;
  moveAbs1.mDeceleration = 500;
  moveAbs1.mJerk = 5000;
  moveAbs1.mDirection = MC_DIRECTION_POSITIVE;

  FbMoveAdditive moveAdd2;
  moveAdd2.mAxis = axis_;
  moveAdd2.mDistance = 400;
  moveAdd2.mVelocity = 200;
  moveAdd2.mAcceleration = 500;
  moveAdd2.mDeceleration = 500;
  moveAdd2.mJerk = 5000;
  moveAdd2.mBufferMode = MC_BUFFERMODE_ABORTING;

  fb_profile_.addFB("moveAbs1");
  fb_profile_.addFB("moveAdd2");
  fb_profile_.addItemName("moveAbs1.go");
  fb_profile_.addItemName("moveAbs1.done");
  fb_profile_.addItemName("moveAbs1.aborted");
  fb_profile_.addItemName("moveAdd2.test");
  fb_profile_.addItemName("moveAdd2.finish");

  double time_out = 7;
  double t = 0;
  while (t < time_out)
  {
    sched_.runCycle();

    //功能块调用
    power_.call();
    moveAbs1.call();
    moveAdd2.call();
    readPos_.call();
    readVel_.call();

    bool isPowerOn = power_.mStatus && power_.mValid;
    if (isPowerOn && !moveAbs1.mExecute && t < 2)
    {  //使能成功后move1开始移动
      std::cout << "axis poweron, moveVel start" << std::endl;
      moveAbs1.mExecute = isPowerOn;
    }

    if (2 < t && moveAbs1.mExecute)
      moveAbs1.mExecute = false;

    if (1 < t && t < 6 && !moveAdd2.mExecute)
      moveAdd2.mExecute = true;

    if (6 < t && moveAdd2.mExecute)
      moveAdd2.mExecute = false;

    axis_profile_.addState(readPos_.mPosition, readVel_.mVelocity, t);
    fb_profile_.addState("moveAbs1.go", moveAbs1.mExecute);
    fb_profile_.addState("moveAbs1.done", moveAbs1.mDone);
    fb_profile_.addState("moveAbs1.aborted", moveAbs1.mCommandAborted);
    fb_profile_.addState("moveAdd2.test", moveAdd2.mExecute);
    fb_profile_.addState("moveAdd2.finish", moveAdd2.mDone);
    fb_profile_.addTime(t);

    t += 1 / frequency_;
  }

  axis_profile_.plot("mc_moveAdd_axis.png");
  fb_profile_.plot("mc_moveAdd_fb.png");
}

// Test MC_MoveVelocity
TEST_F(FunctionBlockTest, MC_MoveVelocity)
{
  resetAxis();
  fb_profile_.reset();
  axis_profile_.reset();

  FbMoveVelocity moveVel1;
  moveVel1.mAxis = axis_;
  moveVel1.mVelocity = 500;
  moveVel1.mAcceleration = 500;
  moveVel1.mDeceleration = 500;
  moveVel1.mJerk = 5000;
  moveVel1.mBufferMode = MC_BUFFERMODE_ABORTING;

  FbMoveVelocity moveVel2;
  moveVel2.mAxis = axis_;
  moveVel2.mVelocity = 200;
  moveVel2.mAcceleration = 500;
  moveVel2.mDeceleration = 500;
  moveVel2.mJerk = 5000;
  moveVel2.mBufferMode = MC_BUFFERMODE_ABORTING;

  fb_profile_.addFB("moveVel1");
  fb_profile_.addFB("moveVel2");
  fb_profile_.addItemName("moveVel1.go");
  fb_profile_.addItemName("moveVel1.inVel");
  fb_profile_.addItemName("moveVel1.aborted");
  fb_profile_.addItemName("moveVel1.next");
  fb_profile_.addItemName("moveVel2.test");
  fb_profile_.addItemName("moveVel2.finish");

  double time_out = 9;
  double t = 0;
  bool next = false, test = false;
  while (t < time_out)
  {
    sched_.runCycle();

    //功能块调用
    power_.call();
    moveVel1.call();
    moveVel2.call();
    readPos_.call();
    readVel_.call();

    bool isPowerOn = power_.mStatus && power_.mValid;
    if (isPowerOn && !moveVel1.mExecute && t < 5)
    {  //使能成功后move1开始移动
      std::cout << "axis poweron, moveVel start" << std::endl;
      moveVel1.mExecute = isPowerOn;
    }

    if (5 < t && t < 6 && moveVel1.mExecute)
      moveVel1.mExecute = false;

    if (6 < t && t < 8 && !moveVel1.mExecute)
      moveVel1.mExecute = true;

    if (8 < t && moveVel1.mExecute)
      moveVel1.mExecute = false;

    if (2 < t && t < 4 && !next)
      next = true;

    if (4 < t && next)
      next = false;

    if (6.5 < t && t < 8.5 && !test)
      test = true;

    if (8.5 < t && test)
      test = false;

    moveVel2.mExecute = (moveVel1.mInVelocity && next) || test;

    axis_profile_.addState(readPos_.mPosition, readVel_.mVelocity, t);
    fb_profile_.addState("moveVel1.go", moveVel1.mExecute);
    fb_profile_.addState("moveVel1.inVel", moveVel1.mInVelocity);
    fb_profile_.addState("moveVel1.aborted", moveVel1.mCommandAborted);
    fb_profile_.addState("moveVel1.next", next);
    fb_profile_.addState("moveVel2.test", test);
    fb_profile_.addState("moveVel2.finish", moveVel2.mDone);
    fb_profile_.addTime(t);

    t += 1 / frequency_;
  }

  axis_profile_.plot("mc_moveVel_axis.png");
  fb_profile_.plot("mc_moveVel_fb.png");
}

// Test Buffered
TEST_F(FunctionBlockTest, mcBuffered)
{
  resetAxis();
  fb_profile_.reset();
  axis_profile_.reset();

  FbMoveAbsolute moveAbs1;
  moveAbs1.mAxis = axis_;
  moveAbs1.mPosition = 1000;
  moveAbs1.mVelocity = 100;
  moveAbs1.mAcceleration = 100;
  moveAbs1.mDeceleration = 100;
  moveAbs1.mJerk = 5000;
  moveAbs1.mDirection = MC_DIRECTION_POSITIVE;
  moveAbs1.mBufferMode = MC_BUFFERMODE_ABORTING;

  FbMoveAbsolute moveAbs2;
  moveAbs2.mAxis = axis_;
  moveAbs2.mPosition = 2000;
  moveAbs2.mVelocity = 50;
  moveAbs2.mAcceleration = 200;
  moveAbs2.mDeceleration = 200;
  moveAbs2.mJerk = 5000;
  moveAbs2.mDirection = MC_DIRECTION_POSITIVE;
  moveAbs2.mBufferMode = MC_BUFFERMODE_BUFFERED;

  fb_profile_.addFB("moveAbs1");
  fb_profile_.addFB("moveAbs2");
  fb_profile_.addItemName("moveAbs1.start");
  fb_profile_.addItemName("moveAbs1.busy");
  fb_profile_.addItemName("moveAbs1.done");
  fb_profile_.addItemName("moveAbs1.aborted");
  fb_profile_.addItemName("moveAbs1.active");

  fb_profile_.addItemName("moveAbs2.start");
  fb_profile_.addItemName("moveAbs2.busy");
  fb_profile_.addItemName("moveAbs2.done");
  fb_profile_.addItemName("moveAbs2.aborted");
  fb_profile_.addItemName("moveAbs2.active");

  double time_out = 35;
  double t = 0;
  while (t < time_out)
  {
    sched_.runCycle();

    //功能块调用
    power_.call();
    moveAbs1.call();
    moveAbs2.call();
    readPos_.call();
    readVel_.call();

    bool isPowerOn = power_.mStatus && power_.mValid;
    if (isPowerOn && !moveAbs1.mExecute && t < 5)
    {  //使能成功后move1开始移动
      std::cout << "axis poweron, moveVel start" << std::endl;
      moveAbs1.mExecute = isPowerOn;
    }

    if (5 < t && moveAbs1.mExecute)
      moveAbs1.mExecute = false;

    if (6 < t && t < 11 && !moveAbs2.mExecute)
      moveAbs2.mExecute = true;

    if (11 < t && moveAbs2.mExecute)
      moveAbs2.mExecute = false;

    axis_profile_.addState(readPos_.mPosition, readVel_.mVelocity, t);
    fb_profile_.addState("moveAbs1.start", moveAbs1.mExecute);
    fb_profile_.addState("moveAbs1.busy", moveAbs1.mBusy);
    fb_profile_.addState("moveAbs1.done", moveAbs1.mDone);
    fb_profile_.addState("moveAbs1.aborted", moveAbs1.mCommandAborted);
    fb_profile_.addState("moveAbs1.active", moveAbs1.mActive);

    fb_profile_.addState("moveAbs2.start", moveAbs2.mExecute);
    fb_profile_.addState("moveAbs2.busy", moveAbs2.mBusy);
    fb_profile_.addState("moveAbs2.done", moveAbs2.mDone);
    fb_profile_.addState("moveAbs2.aborted", moveAbs2.mCommandAborted);
    fb_profile_.addState("moveAbs2.active", moveAbs2.mActive);
    fb_profile_.addTime(t);

    t += 1 / frequency_;
  }

  axis_profile_.plot("mc_buffered_axis.png");
  fb_profile_.plot("mc_buffered_fb.png");
}

// Test BlendingLow
TEST_F(FunctionBlockTest, mcBlendingLow)
{
  resetAxis();
  fb_profile_.reset();
  axis_profile_.reset();

  FbMoveAbsolute moveAbs1;
  moveAbs1.mAxis = axis_;
  moveAbs1.mPosition = 1000;
  moveAbs1.mVelocity = 100;
  moveAbs1.mAcceleration = 100;
  moveAbs1.mDeceleration = 100;
  moveAbs1.mJerk = 5000;
  moveAbs1.mDirection = MC_DIRECTION_POSITIVE;
  moveAbs1.mBufferMode = MC_BUFFERMODE_ABORTING;

  FbMoveAbsolute moveAbs2;
  moveAbs2.mAxis = axis_;
  moveAbs2.mPosition = 2000;
  moveAbs2.mVelocity = 50;
  moveAbs2.mAcceleration = 50;
  moveAbs2.mDeceleration = 50;
  moveAbs2.mJerk = 5000;
  moveAbs2.mDirection = MC_DIRECTION_POSITIVE;
  moveAbs2.mBufferMode = MC_BUFFERMODE_BLENDINGLOW;

  FbMoveAbsolute moveAbs3;
  moveAbs3.mAxis = axis_;
  moveAbs3.mPosition = 3000;
  moveAbs3.mVelocity = 100;
  moveAbs3.mAcceleration = 100;
  moveAbs3.mDeceleration = 100;
  moveAbs3.mJerk = 5000;
  moveAbs3.mDirection = MC_DIRECTION_POSITIVE;
  moveAbs3.mBufferMode = MC_BUFFERMODE_BLENDINGLOW;

  fb_profile_.addFB("moveAbs1");
  fb_profile_.addFB("moveAbs2");
  fb_profile_.addFB("moveAbs3");
  fb_profile_.addItemName("moveAbs1.start");
  fb_profile_.addItemName("moveAbs1.busy");
  fb_profile_.addItemName("moveAbs1.done");
  fb_profile_.addItemName("moveAbs1.abort");
  fb_profile_.addItemName("moveAbs1.active");

  fb_profile_.addItemName("moveAbs2.start");
  fb_profile_.addItemName("moveAbs2.busy");
  fb_profile_.addItemName("moveAbs2.done");
  fb_profile_.addItemName("moveAbs2.abort");
  fb_profile_.addItemName("moveAbs2.active");

  fb_profile_.addItemName("moveAbs3.start");
  fb_profile_.addItemName("moveAbs3.busy");
  fb_profile_.addItemName("moveAbs3.done");
  fb_profile_.addItemName("moveAbs3.abort");
  fb_profile_.addItemName("moveAbs3.active");

  double time_out = 55;
  double t = 0;
  while (t < time_out)
  {
    sched_.runCycle();

    //功能块调用
    power_.call();
    moveAbs1.call();
    moveAbs2.call();
    moveAbs3.call();
    readPos_.call();
    readVel_.call();

    bool isPowerOn = power_.mStatus && power_.mValid;
    if (isPowerOn && !moveAbs1.mExecute && t < 5)
    {  //使能成功后move1开始移动
      std::cout << "axis poweron, moveVel start" << std::endl;
      moveAbs1.mExecute = isPowerOn;
    }

    if (5 < t && moveAbs1.mExecute)
      moveAbs1.mExecute = false;

    if (6 < t && t < 8 && !moveAbs2.mExecute)
      moveAbs2.mExecute = true;

    if (8 < t && moveAbs2.mExecute)
      moveAbs2.mExecute = false;

    if (20 < t && t < 25 && !moveAbs3.mExecute)
      moveAbs3.mExecute = true;

    if (25 < t && moveAbs3.mExecute)
      moveAbs3.mExecute = false;

    axis_profile_.addState(readPos_.mPosition, readVel_.mVelocity, t);
    fb_profile_.addState("moveAbs1.start", moveAbs1.mExecute);
    fb_profile_.addState("moveAbs1.busy", moveAbs1.mBusy);
    fb_profile_.addState("moveAbs1.done", moveAbs1.mDone);
    fb_profile_.addState("moveAbs1.abort", moveAbs1.mCommandAborted);
    fb_profile_.addState("moveAbs1.active", moveAbs1.mActive);

    fb_profile_.addState("moveAbs2.start", moveAbs2.mExecute);
    fb_profile_.addState("moveAbs2.busy", moveAbs2.mBusy);
    fb_profile_.addState("moveAbs2.done", moveAbs2.mDone);
    fb_profile_.addState("moveAbs2.abort", moveAbs2.mCommandAborted);
    fb_profile_.addState("moveAbs2.active", moveAbs2.mActive);

    fb_profile_.addState("moveAbs3.start", moveAbs3.mExecute);
    fb_profile_.addState("moveAbs3.busy", moveAbs3.mBusy);
    fb_profile_.addState("moveAbs3.done", moveAbs3.mDone);
    fb_profile_.addState("moveAbs3.abort", moveAbs3.mCommandAborted);
    fb_profile_.addState("moveAbs3.active", moveAbs3.mActive);
    fb_profile_.addTime(t);

    t += 1 / frequency_;
  }

  axis_profile_.plot("mc_blendingLow_axis.png");
  fb_profile_.plot("mc_blendingLow_fb.png", "");
}

// Test BlendingPrevious
TEST_F(FunctionBlockTest, mcBlendingPrevious)
{
  resetAxis();
  fb_profile_.reset();
  axis_profile_.reset();

  FbMoveAbsolute moveAbs1;
  moveAbs1.mAxis = axis_;
  moveAbs1.mPosition = 1000;
  moveAbs1.mVelocity = 100;
  moveAbs1.mAcceleration = 100;
  moveAbs1.mDeceleration = 100;
  moveAbs1.mJerk = 5000;
  moveAbs1.mDirection = MC_DIRECTION_POSITIVE;
  moveAbs1.mBufferMode = MC_BUFFERMODE_ABORTING;

  FbMoveAbsolute moveAbs2;
  moveAbs2.mAxis = axis_;
  moveAbs2.mPosition = 2000;
  moveAbs2.mVelocity = 50;
  moveAbs2.mAcceleration = 200;
  moveAbs2.mDeceleration = 200;
  moveAbs2.mJerk = 5000;
  moveAbs2.mDirection = MC_DIRECTION_POSITIVE;
  moveAbs2.mBufferMode = MC_BUFFERMODE_BLENDINGPREVIOUS;

  FbMoveAbsolute moveAbs3;
  moveAbs3.mAxis = axis_;
  moveAbs3.mPosition = 3000;
  moveAbs3.mVelocity = 100;
  moveAbs3.mAcceleration = 100;
  moveAbs3.mDeceleration = 100;
  moveAbs3.mJerk = 5000;
  moveAbs3.mDirection = MC_DIRECTION_POSITIVE;
  moveAbs3.mBufferMode = MC_BUFFERMODE_BLENDINGPREVIOUS;

  fb_profile_.addFB("moveAbs1");
  fb_profile_.addFB("moveAbs2");
  fb_profile_.addFB("moveAbs3");
  fb_profile_.addItemName("moveAbs1.start");
  fb_profile_.addItemName("moveAbs1.busy");
  fb_profile_.addItemName("moveAbs1.done");
  fb_profile_.addItemName("moveAbs1.abort");
  fb_profile_.addItemName("moveAbs1.active");

  fb_profile_.addItemName("moveAbs2.start");
  fb_profile_.addItemName("moveAbs2.busy");
  fb_profile_.addItemName("moveAbs2.done");
  fb_profile_.addItemName("moveAbs2.abort");
  fb_profile_.addItemName("moveAbs2.active");

  fb_profile_.addItemName("moveAbs3.start");
  fb_profile_.addItemName("moveAbs3.busy");
  fb_profile_.addItemName("moveAbs3.done");
  fb_profile_.addItemName("moveAbs3.abort");
  fb_profile_.addItemName("moveAbs3.active");

  double time_out = 55;
  double t = 0;
  while (t < time_out)
  {
    sched_.runCycle();

    //功能块调用
    power_.call();
    moveAbs1.call();
    moveAbs2.call();
    moveAbs3.call();
    readPos_.call();
    readVel_.call();

    bool isPowerOn = power_.mStatus && power_.mValid;
    if (isPowerOn && !moveAbs1.mExecute && t < 5)
    {  //使能成功后move1开始移动
      std::cout << "axis poweron, moveVel start" << std::endl;
      moveAbs1.mExecute = isPowerOn;
    }

    if (5 < t && moveAbs1.mExecute)
      moveAbs1.mExecute = false;

    if (6 < t && t < 8 && !moveAbs2.mExecute)
      moveAbs2.mExecute = true;

    if (8 < t && moveAbs2.mExecute)
      moveAbs2.mExecute = false;

    if (20 < t && t < 25 && !moveAbs3.mExecute)
      moveAbs3.mExecute = true;

    if (25 < t && moveAbs3.mExecute)
      moveAbs3.mExecute = false;

    axis_profile_.addState(readPos_.mPosition, readVel_.mVelocity, t);
    fb_profile_.addState("moveAbs1.start", moveAbs1.mExecute);
    fb_profile_.addState("moveAbs1.busy", moveAbs1.mBusy);
    fb_profile_.addState("moveAbs1.done", moveAbs1.mDone);
    fb_profile_.addState("moveAbs1.abort", moveAbs1.mCommandAborted);
    fb_profile_.addState("moveAbs1.active", moveAbs1.mActive);

    fb_profile_.addState("moveAbs2.start", moveAbs2.mExecute);
    fb_profile_.addState("moveAbs2.busy", moveAbs2.mBusy);
    fb_profile_.addState("moveAbs2.done", moveAbs2.mDone);
    fb_profile_.addState("moveAbs2.abort", moveAbs2.mCommandAborted);
    fb_profile_.addState("moveAbs2.active", moveAbs2.mActive);

    fb_profile_.addState("moveAbs3.start", moveAbs3.mExecute);
    fb_profile_.addState("moveAbs3.busy", moveAbs3.mBusy);
    fb_profile_.addState("moveAbs3.done", moveAbs3.mDone);
    fb_profile_.addState("moveAbs3.abort", moveAbs3.mCommandAborted);
    fb_profile_.addState("moveAbs3.active", moveAbs3.mActive);
    fb_profile_.addTime(t);

    t += 1 / frequency_;
  }

  axis_profile_.plot("mc_blendingPrevious_axis.png");
  fb_profile_.plot("mc_blendingPrevious_fb.png", "");
}

// Test BlendingNext
TEST_F(FunctionBlockTest, mcBlendingNext)
{
  resetAxis();
  fb_profile_.reset();
  axis_profile_.reset();

  FbMoveAbsolute moveAbs1;
  moveAbs1.mAxis = axis_;
  moveAbs1.mPosition = 1000;
  moveAbs1.mVelocity = 100;
  moveAbs1.mAcceleration = 100;
  moveAbs1.mDeceleration = 100;
  moveAbs1.mJerk = 5000;
  moveAbs1.mDirection = MC_DIRECTION_POSITIVE;
  moveAbs1.mBufferMode = MC_BUFFERMODE_ABORTING;

  FbMoveAbsolute moveAbs2;
  moveAbs2.mAxis = axis_;
  moveAbs2.mPosition = 2000;
  moveAbs2.mVelocity = 50;
  moveAbs2.mAcceleration = 50;
  moveAbs2.mDeceleration = 50;
  moveAbs2.mJerk = 5000;
  moveAbs2.mDirection = MC_DIRECTION_POSITIVE;
  moveAbs2.mBufferMode = MC_BUFFERMODE_BLENDINGNEXT;

  FbMoveAbsolute moveAbs3;
  moveAbs3.mAxis = axis_;
  moveAbs3.mPosition = 3000;
  moveAbs3.mVelocity = 100;
  moveAbs3.mAcceleration = 100;
  moveAbs3.mDeceleration = 100;
  moveAbs3.mJerk = 5000;
  moveAbs3.mDirection = MC_DIRECTION_POSITIVE;
  moveAbs3.mBufferMode = MC_BUFFERMODE_BLENDINGNEXT;

  fb_profile_.addFB("moveAbs1");
  fb_profile_.addFB("moveAbs2");
  fb_profile_.addFB("moveAbs3");
  fb_profile_.addItemName("moveAbs1.start");
  fb_profile_.addItemName("moveAbs1.busy");
  fb_profile_.addItemName("moveAbs1.done");
  fb_profile_.addItemName("moveAbs1.abort");
  fb_profile_.addItemName("moveAbs1.active");

  fb_profile_.addItemName("moveAbs2.start");
  fb_profile_.addItemName("moveAbs2.busy");
  fb_profile_.addItemName("moveAbs2.done");
  fb_profile_.addItemName("moveAbs2.abort");
  fb_profile_.addItemName("moveAbs2.active");

  fb_profile_.addItemName("moveAbs3.start");
  fb_profile_.addItemName("moveAbs3.busy");
  fb_profile_.addItemName("moveAbs3.done");
  fb_profile_.addItemName("moveAbs3.abort");
  fb_profile_.addItemName("moveAbs3.active");

  double time_out = 55;
  double t = 0;
  while (t < time_out)
  {
    sched_.runCycle();

    //功能块调用
    power_.call();
    moveAbs1.call();
    moveAbs2.call();
    moveAbs3.call();
    readPos_.call();
    readVel_.call();

    bool isPowerOn = power_.mStatus && power_.mValid;
    if (isPowerOn && !moveAbs1.mExecute && t < 5)
    {  //使能成功后move1开始移动
      std::cout << "axis poweron, moveVel start" << std::endl;
      moveAbs1.mExecute = isPowerOn;
    }

    if (5 < t && moveAbs1.mExecute)
      moveAbs1.mExecute = false;

    if (6 < t && t < 8 && !moveAbs2.mExecute)
      moveAbs2.mExecute = true;

    if (8 < t && moveAbs2.mExecute)
      moveAbs2.mExecute = false;

    if (20 < t && t < 25 && !moveAbs3.mExecute)
      moveAbs3.mExecute = true;

    if (25 < t && moveAbs3.mExecute)
      moveAbs3.mExecute = false;

    axis_profile_.addState(readPos_.mPosition, readVel_.mVelocity, t);
    fb_profile_.addState("moveAbs1.start", moveAbs1.mExecute);
    fb_profile_.addState("moveAbs1.busy", moveAbs1.mBusy);
    fb_profile_.addState("moveAbs1.done", moveAbs1.mDone);
    fb_profile_.addState("moveAbs1.abort", moveAbs1.mCommandAborted);
    fb_profile_.addState("moveAbs1.active", moveAbs1.mActive);

    fb_profile_.addState("moveAbs2.start", moveAbs2.mExecute);
    fb_profile_.addState("moveAbs2.busy", moveAbs2.mBusy);
    fb_profile_.addState("moveAbs2.done", moveAbs2.mDone);
    fb_profile_.addState("moveAbs2.abort", moveAbs2.mCommandAborted);
    fb_profile_.addState("moveAbs2.active", moveAbs2.mActive);

    fb_profile_.addState("moveAbs3.start", moveAbs3.mExecute);
    fb_profile_.addState("moveAbs3.busy", moveAbs3.mBusy);
    fb_profile_.addState("moveAbs3.done", moveAbs3.mDone);
    fb_profile_.addState("moveAbs3.abort", moveAbs3.mCommandAborted);
    fb_profile_.addState("moveAbs3.active", moveAbs3.mActive);
    fb_profile_.addTime(t);

    t += 1 / frequency_;
  }

  axis_profile_.plot("mc_blendingNext_axis.png");
  fb_profile_.plot("mc_blendingNext_fb.png", "");
}

// Test BlendingHigh
TEST_F(FunctionBlockTest, mcBlendingHigh)
{
  resetAxis();
  fb_profile_.reset();
  axis_profile_.reset();

  FbMoveAbsolute moveAbs1;
  moveAbs1.mAxis = axis_;
  moveAbs1.mPosition = 1000;
  moveAbs1.mVelocity = 100;
  moveAbs1.mAcceleration = 100;
  moveAbs1.mDeceleration = 100;
  moveAbs1.mJerk = 5000;
  moveAbs1.mDirection = MC_DIRECTION_POSITIVE;
  moveAbs1.mBufferMode = MC_BUFFERMODE_ABORTING;

  FbMoveAbsolute moveAbs2;
  moveAbs2.mAxis = axis_;
  moveAbs2.mPosition = 2000;
  moveAbs2.mVelocity = 50;
  moveAbs2.mAcceleration = 200;
  moveAbs2.mDeceleration = 200;
  moveAbs2.mJerk = 5000;
  moveAbs2.mDirection = MC_DIRECTION_POSITIVE;
  moveAbs2.mBufferMode = MC_BUFFERMODE_BLENDINGHIGH;

  FbMoveAbsolute moveAbs3;
  moveAbs3.mAxis = axis_;
  moveAbs3.mPosition = 3000;
  moveAbs3.mVelocity = 100;
  moveAbs3.mAcceleration = 100;
  moveAbs3.mDeceleration = 100;
  moveAbs3.mJerk = 5000;
  moveAbs3.mDirection = MC_DIRECTION_POSITIVE;
  moveAbs3.mBufferMode = MC_BUFFERMODE_BLENDINGHIGH;

  fb_profile_.addFB("moveAbs1");
  fb_profile_.addFB("moveAbs2");
  fb_profile_.addFB("moveAbs3");
  fb_profile_.addItemName("moveAbs1.start");
  fb_profile_.addItemName("moveAbs1.busy");
  fb_profile_.addItemName("moveAbs1.done");
  fb_profile_.addItemName("moveAbs1.abort");
  fb_profile_.addItemName("moveAbs1.active");

  fb_profile_.addItemName("moveAbs2.start");
  fb_profile_.addItemName("moveAbs2.busy");
  fb_profile_.addItemName("moveAbs2.done");
  fb_profile_.addItemName("moveAbs2.abort");
  fb_profile_.addItemName("moveAbs2.active");

  fb_profile_.addItemName("moveAbs3.start");
  fb_profile_.addItemName("moveAbs3.busy");
  fb_profile_.addItemName("moveAbs3.done");
  fb_profile_.addItemName("moveAbs3.abort");
  fb_profile_.addItemName("moveAbs3.active");

  double time_out = 55;
  double t = 0;
  while (t < time_out)
  {
    sched_.runCycle();

    //功能块调用
    power_.call();
    moveAbs1.call();
    moveAbs2.call();
    moveAbs3.call();
    readPos_.call();
    readVel_.call();

    bool isPowerOn = power_.mStatus && power_.mValid;
    if (isPowerOn && !moveAbs1.mExecute && t < 5)
    {  //使能成功后move1开始移动
      std::cout << "axis poweron, moveVel start" << std::endl;
      moveAbs1.mExecute = isPowerOn;
    }

    if (5 < t && moveAbs1.mExecute)
      moveAbs1.mExecute = false;

    if (6 < t && t < 8 && !moveAbs2.mExecute)
      moveAbs2.mExecute = true;

    if (8 < t && moveAbs2.mExecute)
      moveAbs2.mExecute = false;

    if (20 < t && t < 25 && !moveAbs3.mExecute)
      moveAbs3.mExecute = true;

    if (25 < t && moveAbs3.mExecute)
      moveAbs3.mExecute = false;

    axis_profile_.addState(readPos_.mPosition, readVel_.mVelocity, t);
    fb_profile_.addState("moveAbs1.start", moveAbs1.mExecute);
    fb_profile_.addState("moveAbs1.busy", moveAbs1.mBusy);
    fb_profile_.addState("moveAbs1.done", moveAbs1.mDone);
    fb_profile_.addState("moveAbs1.abort", moveAbs1.mCommandAborted);
    fb_profile_.addState("moveAbs1.active", moveAbs1.mActive);

    fb_profile_.addState("moveAbs2.start", moveAbs2.mExecute);
    fb_profile_.addState("moveAbs2.busy", moveAbs2.mBusy);
    fb_profile_.addState("moveAbs2.done", moveAbs2.mDone);
    fb_profile_.addState("moveAbs2.abort", moveAbs2.mCommandAborted);
    fb_profile_.addState("moveAbs2.active", moveAbs2.mActive);

    fb_profile_.addState("moveAbs3.start", moveAbs3.mExecute);
    fb_profile_.addState("moveAbs3.busy", moveAbs3.mBusy);
    fb_profile_.addState("moveAbs3.done", moveAbs3.mDone);
    fb_profile_.addState("moveAbs3.abort", moveAbs3.mCommandAborted);
    fb_profile_.addState("moveAbs3.active", moveAbs3.mActive);
    fb_profile_.addTime(t);

    t += 1 / frequency_;
  }

  axis_profile_.plot("mc_blendingHigh_axis.png");
  fb_profile_.plot("mc_blendingHigh_fb.png", "");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}