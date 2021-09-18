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
 * @file function_block_test.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include <thread>
#include "gtest/gtest.h"
#include <RTmotion/axis.hpp>
#include <RTmotion/global.hpp>
#include <RTmotion/tool/fb_debug_tool.hpp>
#include <RTmotion/fb/fb_move_relative.hpp>
#include <RTmotion/fb/fb_move_velocity.hpp>
#include <RTmotion/fb/fb_move_absolute.hpp>
#include <RTmotion/fb/fb_move_additive.hpp>
#include <RTmotion/fb/fb_halt.hpp>
#include <RTmotion/fb/fb_stop.hpp>
#include <RTmotion/fb/fb_power.hpp>

using namespace std::chrono_literals;

class FunctionBlockTest : public ::testing::Test
{
protected:
  // You can remove any or all of the following functions if their bodies would
  // be empty.

  FunctionBlockTest()
  {
    // You can do set-up work for each test here.
  }

  ~FunctionBlockTest() override
  {
    // You can do clean-up work that doesn't throw exceptions here.
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
  AxisProfile axis_profile_;
  FBDigitalProfile fb_profile_;
};

// Test MC_MoveRelative
TEST_F(FunctionBlockTest, DemoRelative)
{
  fb_profile_.reset();
  axis_profile_.reset();

  RTmotion::AxisConfig config;
  RTmotion::AXIS_REF axis;
  axis = std::make_shared<RTmotion::Axis>();
  axis->setAxisId(1);
  axis->setAxisName("X");
  axis->setAxisConfig(&config);
  printf("Axis initialized.\n");

  std::shared_ptr<RTmotion::Servo> servo;
  servo = std::make_shared<RTmotion::Servo>();
  axis->setServo(servo);

  RTmotion::FbPower fbPower;
  fbPower.setAxis(axis);
  fbPower.setExecute(true);
  fbPower.setEnablePositive(true);
  fbPower.setEnableNegative(true);

  RTmotion::FbMoveRelative fb_move_rel;
  fb_move_rel.setAxis(axis);
  fb_move_rel.setExecute(true);
  fb_move_rel.setContinuousUpdate(false);
  fb_move_rel.setDistance(3.14);
  fb_move_rel.setVelocity(1.57);
  fb_move_rel.setAcceleration(3.14);
  fb_move_rel.setJerk(50);
  printf("Function block initialized.\n");

  auto start = std::chrono::high_resolution_clock::now();
  double timeout = 0;
  while (!fb_move_rel.isDone() && timeout < 5.0)
  {
    axis->runCycle();
    fbPower.runCycle();
    fb_move_rel.runCycle();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> d = end - start;
    timeout = d.count();
    axis_profile_.addState(axis->toUserPos(), axis->toUserVel(), timeout);
    // printf("Run time: %f, pos: %f, vel: %f\n", timeout, axis->toUserPos(),
    // axis->toUserVel());
    std::this_thread::sleep_for(1ms);
  }
  axis_profile_.plot("mc_demo_relative_axis.png");
  ASSERT_TRUE(fb_move_rel.isDone());
  ASSERT_LT(timeout, 5.0);
}

// Test MC_MoveVelocity
TEST_F(FunctionBlockTest, DemoVelocity)
{
  fb_profile_.reset();
  axis_profile_.reset();

  RTmotion::AxisConfig config;
  config.mode_ = RTmotion::mcServoControlModePosition;
  RTmotion::AXIS_REF axis;
  axis = std::make_shared<RTmotion::Axis>();
  axis->setAxisId(1);
  axis->setAxisName("X");
  axis->setAxisConfig(&config);
  printf("Axis initialized.\n");

  std::shared_ptr<RTmotion::Servo> servo;
  servo = std::make_shared<RTmotion::Servo>();
  axis->setServo(servo);

  RTmotion::FbPower fbPower;
  fbPower.setAxis(axis);
  fbPower.setExecute(true);
  fbPower.setEnablePositive(true);
  fbPower.setEnableNegative(true);

  RTmotion::FbMoveVelocity fb_move_vel;
  fb_move_vel.setAxis(axis);
  fb_move_vel.setExecute(true);
  fb_move_vel.setContinuousUpdate(false);
  fb_move_vel.setVelocity(1.57);
  fb_move_vel.setAcceleration(3.14);
  fb_move_vel.setJerk(50);
  printf("Function block initialized.\n");

  auto start = std::chrono::high_resolution_clock::now();
  double timeout = 0;
  while (!fb_move_vel.isInVelocity() && timeout < 5.0)
  {
    axis->runCycle();
    fbPower.runCycle();
    fb_move_vel.runCycle();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> d = end - start;
    timeout = d.count();
    axis_profile_.addState(axis->toUserPos(), axis->toUserVel(), timeout);
    // printf("Run time: %f, pos: %f, vel: %f\n", timeout, axis->toUserPos(),
    // axis->toUserVel());
    std::this_thread::sleep_for(1ms);
  }
  axis_profile_.plot("mc_demo_velocity_axis.png");
  ASSERT_TRUE(fb_move_vel.isInVelocity());
  ASSERT_LT(timeout, 5.0);
}

// Test MC_Stop
TEST_F(FunctionBlockTest, MC_Stop)
{
  fb_profile_.reset();
  axis_profile_.reset();

  RTmotion::AxisConfig config;
  config.mode_ = RTmotion::mcServoControlModePosition;
  RTmotion::AXIS_REF axis;
  axis = std::make_shared<RTmotion::Axis>();
  axis->setAxisId(1);
  axis->setAxisName("X");
  axis->setAxisConfig(&config);
  printf("Axis initialized.\n");

  std::shared_ptr<RTmotion::Servo> servo;
  servo = std::make_shared<RTmotion::Servo>();
  axis->setServo(servo);

  RTmotion::FbPower fbPower;
  fbPower.setAxis(axis);
  fbPower.setExecute(true);
  fbPower.setEnablePositive(true);
  fbPower.setEnableNegative(true);

  RTmotion::FbMoveVelocity moveVel;
  moveVel.setAxis(axis);
  moveVel.setVelocity(50);
  moveVel.setAcceleration(10);
  moveVel.setDeceleration(10);
  moveVel.setJerk(5000);
  moveVel.setBufferMode(RTmotion::mcAborting);

  RTmotion::FbStop stop;
  stop.setAxis(axis);
  stop.setAcceleration(20);
  stop.setDeceleration(20);
  stop.setJerk(5000);

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
  auto start = std::chrono::high_resolution_clock::now();
  while (t < time_out)
  {
    axis->runCycle();
    fbPower.runCycle();
    moveVel.runCycle();
    stop.runCycle();

    if (!moveVel.isEnabled() && t < 7.5)
    {  //使能成功后move1开始移动
      std::cout << "axis poweron, moveVel start" << std::endl;
      moveVel.setExecute(true);
    }

    if (6 < t && t < 12.5 && !stop.isEnabled())
      stop.setExecute(true);

    if (12.5 < t && stop.isEnabled())
      stop.setExecute(false);

    if (7.5 < t && t < 10 && moveVel.isEnabled())
      moveVel.setExecute(false);

    if (10 < t && t < 15 && !moveVel.isEnabled())
      moveVel.setExecute(true);

    if (15 < t && t < 20 && moveVel.isEnabled())
      moveVel.setExecute(false);

    if (17.5 < t && !moveVel.isEnabled())
      moveVel.setExecute(true);

    axis_profile_.addState(axis->toUserPos(), axis->toUserVel(), t);
    fb_profile_.addState("moveVel.execute", moveVel.isEnabled());
    fb_profile_.addState("moveVel.inVel", moveVel.isInVelocity());
    fb_profile_.addState("moveVel.abort", moveVel.isAborted());
    fb_profile_.addState("moveVel.error", moveVel.isError());
    fb_profile_.addState("stop.execute", stop.isEnabled());
    fb_profile_.addState("stop.done", stop.isDone());
    fb_profile_.addTime(t);

    // printf("Run time: %f, pos: %f, vel: %f\n", t, axis->toUserPos(),
    // axis->toUserVel());

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> d = end - start;
    t = d.count();

    std::this_thread::sleep_for(1ms);
  }

  axis_profile_.plot("mc_stop_axis.png");
  fb_profile_.plot("mc_stop_fb.png");
}

// Test MC_Halt
TEST_F(FunctionBlockTest, MC_Halt)
{
  fb_profile_.reset();
  axis_profile_.reset();

  RTmotion::AxisConfig config;
  config.mode_ = RTmotion::mcServoControlModePosition;
  RTmotion::AXIS_REF axis;
  axis = std::make_shared<RTmotion::Axis>();
  axis->setAxisId(1);
  axis->setAxisName("X");
  axis->setAxisConfig(&config);
  printf("Axis initialized.\n");

  std::shared_ptr<RTmotion::Servo> servo;
  servo = std::make_shared<RTmotion::Servo>();
  axis->setServo(servo);

  RTmotion::FbPower fbPower;
  fbPower.setAxis(axis);
  fbPower.setExecute(true);
  fbPower.setEnablePositive(true);
  fbPower.setEnableNegative(true);

  RTmotion::FbMoveVelocity moveVel;
  moveVel.setAxis(axis);
  moveVel.setVelocity(50);
  moveVel.setAcceleration(10);
  moveVel.setDeceleration(10);
  moveVel.setJerk(50);
  moveVel.setBufferMode(RTmotion::mcAborting);

  RTmotion::FbHalt halt;
  halt.setAxis(axis);
  halt.setAcceleration(5);
  halt.setJerk(50);
  halt.setBufferMode(RTmotion::mcAborting);

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
  auto start = std::chrono::high_resolution_clock::now();
  while (t < time_out)
  {
    axis->runCycle();
    fbPower.runCycle();
    moveVel.runCycle();
    halt.runCycle();

    if (!moveVel.isEnabled() && t < 7.5)
    {  //使能成功后move1开始移动
      std::cout << "axis poweron, moveVel start" << std::endl;
      moveVel.setExecute(true);
    }

    if (8.5 < t && t < 19 && moveVel.isEnabled())
      moveVel.setExecute(false);

    if (19 < t && t < 21.5 && !moveVel.isEnabled())
      moveVel.setExecute(true);

    if (21.5 < t && t < 32.5 && moveVel.isEnabled())
      moveVel.setExecute(false);

    if (32.5 < t && !moveVel.isEnabled())
      moveVel.setExecute(true);

    if (7 < t && t < 17.5 && !halt.isEnabled())
      halt.setExecute(true);

    if (17.5 < t && t < 27.5 && halt.isEnabled())
      halt.setExecute(false);

    if (27.5 < t && t < 37.5 && !halt.isEnabled())
      halt.setExecute(true);

    if (37.5 < t && halt.isEnabled())
      halt.setExecute(false);

    axis_profile_.addState(axis->toUserPos(), axis->toUserVel(), t);
    fb_profile_.addState("moveVel.execute", moveVel.isEnabled());
    fb_profile_.addState("moveVel.inVel", moveVel.isInVelocity());
    fb_profile_.addState("moveVel.abort", moveVel.isAborted());
    fb_profile_.addState("halt.execute", halt.isEnabled());
    fb_profile_.addState("halt.done", halt.isDone());
    fb_profile_.addState("halt.abort", halt.isAborted());
    fb_profile_.addTime(t);

    // printf("Run time: %f, pos: %f, vel: %f\n", t, axis->toUserPos(),
    // axis->toUserVel());

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> d = end - start;
    t = d.count();

    std::this_thread::sleep_for(1ms);
  }

  axis_profile_.plot("mc_halt_axis.png");
  fb_profile_.plot("mc_halt_fb.png");
}

// Test MC_MoveAbsolute
TEST_F(FunctionBlockTest, MC_MoveAbsolute)
{
  fb_profile_.reset();
  axis_profile_.reset();

  RTmotion::AxisConfig config;
  RTmotion::AXIS_REF axis;
  axis = std::make_shared<RTmotion::Axis>();
  axis->setAxisId(1);
  axis->setAxisName("X");
  axis->setAxisConfig(&config);
  printf("Axis initialized.\n");

  std::shared_ptr<RTmotion::Servo> servo;
  servo = std::make_shared<RTmotion::Servo>();
  axis->setServo(servo);

  RTmotion::FbPower fbPower;
  fbPower.setAxis(axis);
  fbPower.setExecute(true);
  fbPower.setEnablePositive(true);
  fbPower.setEnableNegative(true);

  RTmotion::FbMoveAbsolute moveAbs1;
  moveAbs1.setAxis(axis);
  moveAbs1.setPosition(600);
  moveAbs1.setVelocity(300);
  moveAbs1.setAcceleration(500);
  moveAbs1.setDeceleration(500);
  moveAbs1.setJerk(5000);

  RTmotion::FbMoveAbsolute moveAbs2;
  moveAbs2.setAxis(axis);
  moveAbs2.setPosition(1000);
  moveAbs2.setVelocity(200);
  moveAbs2.setAcceleration(500);
  moveAbs2.setDeceleration(500);
  moveAbs2.setJerk(5000);
  moveAbs2.setBufferMode(RTmotion::mcAborting);

  fb_profile_.addFB("moveAbs1");
  fb_profile_.addFB("moveAbs2");
  fb_profile_.addItemName("moveAbs1.go");
  fb_profile_.addItemName("moveAbs1.done");
  fb_profile_.addItemName("moveAbs1.aborted");
  fb_profile_.addItemName("moveAbs2.test");
  fb_profile_.addItemName("moveAbs2.finish");

  double time_out = 7;
  double t = 0;
  auto start = std::chrono::high_resolution_clock::now();
  while (t < time_out)
  {
    //功能块调用
    axis->runCycle();
    fbPower.runCycle();
    moveAbs1.runCycle();
    moveAbs2.runCycle();

    if (!moveAbs1.isEnabled() && t < 2)
    {  //使能成功后move1开始移动
      std::cout << "axis poweron, moveVel start" << std::endl;
      moveAbs1.setExecute(true);
    }

    if (2 < t && moveAbs1.isEnabled())
      moveAbs1.setExecute(false);

    if (1 < t && t < 6 && !moveAbs2.isEnabled())
      moveAbs2.setExecute(true);

    if (6 < t && moveAbs2.isEnabled())
      moveAbs2.setExecute(false);

    axis_profile_.addState(axis->toUserPos(), axis->toUserVel(), t);
    fb_profile_.addState("moveAbs1.go", moveAbs1.isEnabled());
    fb_profile_.addState("moveAbs1.done", moveAbs1.isDone());
    fb_profile_.addState("moveAbs1.aborted", moveAbs1.isAborted());
    fb_profile_.addState("moveAbs2.test", moveAbs2.isEnabled());
    fb_profile_.addState("moveAbs2.finish", moveAbs2.isDone());
    fb_profile_.addTime(t);

    // printf("Run time: %f, pos: %f, vel: %f\n", t, axis->toUserPos(),
    // axis->toUserVel());

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> d = end - start;
    t = d.count();

    std::this_thread::sleep_for(1ms);
  }

  axis_profile_.plot("mc_moveAbs_axis.png");
  fb_profile_.plot("mc_moveAbs_fb.png");
  ASSERT_LT(fabs(axis->toUserPos() - 1000), 2);
  ASSERT_LT(fabs(axis->toUserVel() - 0.0), 0.01);
}

// Test MC_MoveRelative
TEST_F(FunctionBlockTest, MC_MoveRelative)
{
  fb_profile_.reset();
  axis_profile_.reset();

  RTmotion::AxisConfig config;
  RTmotion::AXIS_REF axis;
  axis = std::make_shared<RTmotion::Axis>();
  axis->setAxisId(1);
  axis->setAxisName("X");
  axis->setAxisConfig(&config);
  printf("Axis initialized.\n");

  std::shared_ptr<RTmotion::Servo> servo;
  servo = std::make_shared<RTmotion::Servo>();
  axis->setServo(servo);

  RTmotion::FbPower fbPower;
  fbPower.setAxis(axis);
  fbPower.setExecute(true);
  fbPower.setEnablePositive(true);
  fbPower.setEnableNegative(true);

  RTmotion::FbMoveRelative fb_move_rel_1, fb_move_rel_2;
  fb_move_rel_1.setAxis(axis);
  fb_move_rel_1.setContinuousUpdate(false);
  fb_move_rel_1.setDistance(600);
  fb_move_rel_1.setVelocity(300);
  fb_move_rel_1.setAcceleration(500);
  fb_move_rel_1.setJerk(5000);

  fb_move_rel_2.setAxis(axis);
  fb_move_rel_2.setContinuousUpdate(false);
  fb_move_rel_2.setDistance(400);
  fb_move_rel_2.setVelocity(200);
  fb_move_rel_2.setAcceleration(500);
  fb_move_rel_2.setJerk(5000);
  fb_move_rel_2.setBufferMode(RTmotion::mcAborting);

  fb_profile_.addFB("moveRel1");
  fb_profile_.addFB("moveRel2");
  fb_profile_.addItemName("moveRel1.go");
  fb_profile_.addItemName("moveRel1.done");
  fb_profile_.addItemName("moveRel1.aborted");
  fb_profile_.addItemName("moveRel2.test");
  fb_profile_.addItemName("moveRel2.finish");

  double time_out = 4;
  double t = 0;
  auto start = std::chrono::high_resolution_clock::now();
  while (t < time_out)
  {
    axis->runCycle();
    fbPower.runCycle();
    fb_move_rel_1.runCycle();
    fb_move_rel_2.runCycle();

    if (!fb_move_rel_1.isEnabled() && t < 2)
    {
      // After enabled, move_relative_1 start to move
      fb_move_rel_1.setExecute(true);
      std::cout << "axis poweron, moveVel start" << std::endl;
    }

    if (2 < t && fb_move_rel_1.isEnabled())
      fb_move_rel_1.setExecute(false);

    if (1 < t && t < 3.8 && !fb_move_rel_2.isEnabled())
      fb_move_rel_2.setExecute(true);

    if (3.8 < t && fb_move_rel_2.isEnabled())
      fb_move_rel_2.setExecute(false);

    axis_profile_.addState(axis->toUserPos(), axis->toUserVel(), t);
    fb_profile_.addState("moveRel1.go", fb_move_rel_1.isEnabled());
    fb_profile_.addState("moveRel1.done", fb_move_rel_1.isDone());
    fb_profile_.addState("moveRel1.aborted", fb_move_rel_1.isAborted());
    fb_profile_.addState("moveRel2.test", fb_move_rel_2.isEnabled());
    fb_profile_.addState("moveRel2.finish", fb_move_rel_2.isDone());
    fb_profile_.addTime(t);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> d = end - start;
    t = d.count();

    // printf("Run time: %f, pos: %f, vel: %f\n", t, axis->toUserPos(),
    // axis->toUserVel());
    std::this_thread::sleep_for(1ms);
  }

  axis_profile_.plot("mc_moveRel_axis.png");
  fb_profile_.plot("mc_moveRel_fb.png");
  ASSERT_LT(fabs(axis->toUserPos() - 595), 50);
  ASSERT_LT(fabs(axis->toUserVel() - 0.0), 0.01);
}

// Test MC_MoveAdditive
TEST_F(FunctionBlockTest, MC_MoveAdditive)
{
  fb_profile_.reset();
  axis_profile_.reset();

  RTmotion::AxisConfig config;
  RTmotion::AXIS_REF axis;
  axis = std::make_shared<RTmotion::Axis>();
  axis->setAxisId(1);
  axis->setAxisName("X");
  axis->setAxisConfig(&config);
  printf("Axis initialized.\n");

  std::shared_ptr<RTmotion::Servo> servo;
  servo = std::make_shared<RTmotion::Servo>();
  axis->setServo(servo);

  RTmotion::FbPower fbPower;
  fbPower.setAxis(axis);
  fbPower.setExecute(true);
  fbPower.setEnablePositive(true);
  fbPower.setEnableNegative(true);

  RTmotion::FbMoveAbsolute moveAbs1;
  moveAbs1.setAxis(axis);
  moveAbs1.setPosition(600);
  moveAbs1.setVelocity(300);
  moveAbs1.setAcceleration(500);
  moveAbs1.setDeceleration(500);
  moveAbs1.setJerk(5000);
  moveAbs1.setDirection(RTmotion::mcPositiveDirection);

  RTmotion::FbMoveAdditive moveAdd2;
  moveAdd2.setAxis(axis);
  moveAdd2.setDistance(400);
  moveAdd2.setVelocity(200);
  moveAdd2.setAcceleration(500);
  moveAdd2.setDeceleration(500);
  moveAdd2.setJerk(5000);
  moveAdd2.setBufferMode(RTmotion::mcAborting);

  fb_profile_.addFB("moveAbs1");
  fb_profile_.addFB("moveAdd2");
  fb_profile_.addItemName("moveAbs1.go");
  fb_profile_.addItemName("moveAbs1.done");
  fb_profile_.addItemName("moveAbs1.aborted");
  fb_profile_.addItemName("moveAdd2.test");
  fb_profile_.addItemName("moveAdd2.finish");

  double time_out = 7;
  double t = 0;
  auto start = std::chrono::high_resolution_clock::now();
  while (t < time_out)
  {
    axis->runCycle();
    fbPower.runCycle();
    moveAbs1.runCycle();
    moveAdd2.runCycle();

    if (!moveAbs1.isEnabled() && t < 2)
    {  //使能成功后move1开始移动
      std::cout << "axis poweron, moveVel start" << std::endl;
      moveAbs1.setExecute(true);
    }

    if (2 < t && moveAbs1.isEnabled())
      moveAbs1.setExecute(false);

    if (1 < t && t < 6 && !moveAdd2.isEnabled())
      moveAdd2.setExecute(true);

    if (6 < t && moveAdd2.isEnabled())
      moveAdd2.setExecute(false);

    axis_profile_.addState(axis->toUserPos(), axis->toUserVel(), t);
    fb_profile_.addState("moveAbs1.go", moveAbs1.isEnabled());
    fb_profile_.addState("moveAbs1.done", moveAbs1.isDone());
    fb_profile_.addState("moveAbs1.aborted", moveAbs1.isAborted());
    fb_profile_.addState("moveAdd2.test", moveAdd2.isEnabled());
    fb_profile_.addState("moveAdd2.finish", moveAdd2.isDone());
    fb_profile_.addTime(t);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> d = end - start;
    t = d.count();

    // printf("Run time: %f, pos: %f, vel: %f\n", t, axis->toUserPos(),
    // axis->toUserVel());
    std::this_thread::sleep_for(1ms);
  }

  axis_profile_.plot("mc_moveAdd_axis.png");
  fb_profile_.plot("mc_moveAdd_fb.png");
  ASSERT_LT(fabs(axis->toUserPos() - 1000), 2);
  ASSERT_LT(fabs(axis->toUserVel() - 0.0), 0.01);
}

// Test MC_MoveVelocity
TEST_F(FunctionBlockTest, MC_MoveVelocity)
{
  fb_profile_.reset();
  axis_profile_.reset();

  RTmotion::AxisConfig config;
  RTmotion::AXIS_REF axis;
  axis = std::make_shared<RTmotion::Axis>();
  axis->setAxisId(1);
  axis->setAxisName("X");
  axis->setAxisConfig(&config);
  printf("Axis initialized.\n");

  std::shared_ptr<RTmotion::Servo> servo;
  servo = std::make_shared<RTmotion::Servo>();
  axis->setServo(servo);  

  RTmotion::FbPower fbPower;
  fbPower.setAxis(axis);
  fbPower.setExecute(true);
  fbPower.setEnablePositive(true);
  fbPower.setEnableNegative(true);

  RTmotion::FbMoveVelocity moveVel1;
  moveVel1.setAxis(axis);
  moveVel1.setVelocity(500);
  moveVel1.setAcceleration(500);
  moveVel1.setDeceleration(500);
  moveVel1.setJerk(5000);
  moveVel1.setBufferMode(RTmotion::mcAborting);

  RTmotion::FbMoveVelocity moveVel2;
  moveVel2.setAxis(axis);
  moveVel2.setVelocity(200);
  moveVel2.setAcceleration(500);
  moveVel2.setDeceleration(500);
  moveVel2.setJerk(5000);
  moveVel2.setBufferMode(RTmotion::mcAborting);

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
  auto start = std::chrono::high_resolution_clock::now();
  while (t < time_out)
  {
    axis->runCycle();
    fbPower.runCycle();
    moveVel1.runCycle();
    moveVel2.runCycle();

    if (!moveVel1.isEnabled() && t < 5)
    {  //使能成功后move1开始移动
      std::cout << "axis poweron, moveVel start" << std::endl;
      moveVel1.setExecute(true);
    }

    if (5 < t && t < 6 && moveVel1.isEnabled())
      moveVel1.setExecute(false);

    if (6 < t && t < 8 && !moveVel1.isEnabled())
      moveVel1.setExecute(true);

    if (8 < t && moveVel1.isEnabled())
      moveVel1.setExecute(false);

    if (2 < t && t < 4 && !next)
    {
      next = true;
      moveVel2.setExecute(next);
    }

    if (4 < t && next)
    {
      next = false;
      moveVel2.setExecute(false);
    }

    if (6.5 < t && t < 8.5 && !test)
    {
      test = true;
      moveVel2.setExecute(test);
    }

    if (8.5 < t && test)
    {
      test = false;
      moveVel2.setExecute(test);
    }

    axis_profile_.addState(axis->toUserPos(), axis->toUserVel(), t);
    fb_profile_.addState("moveVel1.go", moveVel1.isEnabled());
    fb_profile_.addState("moveVel1.inVel", moveVel1.isDone());
    fb_profile_.addState("moveVel1.aborted", moveVel1.isAborted());
    fb_profile_.addState("moveVel1.next", next);
    fb_profile_.addState("moveVel2.test", test);
    fb_profile_.addState("moveVel2.finish", moveVel2.isDone());
    fb_profile_.addTime(t);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> d = end - start;
    t = d.count();

    // printf("Run time: %f, pos: %f, vel: %f\n", t, axis->toUserPos(),
    // axis->toUserVel());
    std::this_thread::sleep_for(1ms);
  }

  axis_profile_.plot("mc_moveVel_axis.png");
  fb_profile_.plot("mc_moveVel_fb.png");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
