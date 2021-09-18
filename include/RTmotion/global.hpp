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
 * @file global.hpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#pragma once

#include <memory>
#include <stdint.h>
#include <string>
#include <time.h>

#define __EPSILON 0.01

#define NSEC_PER_SEC      (1000000000L)
#ifndef DIFF_NS
#define DIFF_NS(A, B)     (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + ((B).tv_nsec) - (A).tv_nsec)
#endif
#ifdef _WIN32
#include <intrin.h>
#else
#include <x86intrin.h>
#endif

namespace RTmotion
{
/* Servo error code */
typedef uint32_t MC_ServoErrorCode;

enum{
  Servo_No_Error                = 0,
  Servo_Fieldbus_Init_Error     = 1,
  Servo_Power_Error             = 2,
  Servo_Powering_On_Error       = 3,
  Servo_Error_When_Powered_On   = 4,
  Servo_Powering_Off_Error      = 5
};

/* Servo control mode */
typedef enum {
  mcServoControlModePosition = 0,
  mcServoControlModeVelocity = 1,
  mcServoControlModeTorque = 2,
} MC_SERVO_CONTROL_MODE;

/* MC error code */
typedef enum {
  mcErrorCodeGood                             = 0x00,   // Success
  mcErrorCode_Scurve_NotFeasible              = 0x10,   // Not feasible to compute Scurve
  mcErrorCode_Scurve_MaxVelNotReached         = 0x21,   // Maximum velocity is not reached
  mcErrorCode_Scurve_MaxAccNotReached         = 0x22,   // Maximum acceleration is not reached
  mcErrorCode_Scurve_FailToFindMaxAcc         = 0x23,   // Fail to find max velocity
  mcErrorCode_Scurve_InvalidInput             = 0x24,   // Invalid constraints for Scurve planning
  mcErrorCode_AxisStateViolation              = 0x30,   // General invalid state transition
  mcErrorCode_PowerOnOffFromErrorStop         = 0x31,   // Try to power on at ErrorStop
  mcErrorCode_InvalidState_FromStopping       = 0x32,   // Invalid state transition at Stopping
  mcErrorCode_InvalidState_FromErrorStop      = 0x33,   // Invalid state transition at ErrorStop
  mcErrorCode_InvalidState_FromDisabled       = 0x34,   // Invalid state transition at Disabled
  mcErrorCode_Motion_limit_Error              = 0x40,   // Velocity, Acceleration or Position over Limit
  mcErrorCode_Invalid_Direction_Positive      = 0x41,   // Velocity in forbiden direction
  mcErrorCode_Invalid_Direction_Negative      = 0x42,   // Velocity in forbiden direction
  mcErrorCode_Velocity_Over_Limit             = 0x43,   // Velocity over limit
  mcErrorCode_Acceleration_Over_Limit         = 0x44,   // Acceleration over limit
  mcErrorCode_Position_Over_Positive_Limit    = 0x45,   // Position over positive limit
  mcErrorCode_Position_Over_Negative_Limit    = 0x46,   // Position over negative limit
  mcErrorCode_AxisErrorStop                   = 0x50,   // Axis error stopped
  mcErrorCode_Servo_No_Error                  = 0x60,   // Servo has no error
  mcErrorCode_Servo_Fieldbus_Init_Error       = 0x61,   // Servo fieldbus initializae error
  mcErrorCode_Servo_Power_Error               = 0x62,   // Servo power error
  mcErrorCode_Servo_Powering_On_Error         = 0x63,   // Servo has error during powering on
  mcErrorCode_Servo_Error_When_Powered_On     = 0x64,   // Servo has error after powered on
  mcErrorCode_Servo_Powering_Off_Error        = 0x65    // Servo has error during powering off
} MC_ERROR_CODE;

/**
 * Axis states
 */
typedef enum {
  /**
   * The state ‘Disabled’ describes the initial state of the axis.
   * In this state the movement of the axis is not influenced by the FBs. Power
   * is off and there is no error
   * in the axis.
   * If the MC_Power FB is called with ‘Enable’=TRUE while being in ‘Disabled’,
   * the state changes to
   * ‘Standstill’. The axis feedback is operational before entering the state
   * ‘Standstill’.
   * Calling MC_Power with ‘Enable’=FALSE in any state except ‘ErrorStop’
   * transfers the axis to the
   * state ‘Disabled’, either directly or via any other state. Any on-going
   * motion commands on the axis are
   * aborted (‘CommandAborted’).
   */
  mcDisabled = 0,

  /**
   * Power is on, there is no error in the axis, and there are no motion
   * commands active on the axis.
   */
  mcStandstill = 1,
  mcHoming = 2,              /// Homing
  mcDiscreteMotion = 3,      /// DiscreteMotion
  mcContinuousMotion = 4,    /// ContinuousMotion
  mcSynchronizedMotion = 5,  /// SynchronizedMotion
  mcStopping = 6,            /// Stopping

  /**
   * ‘ErrorStop’ is valid as highest priority and applicable in case of an
   * error. The axis can have either
   * power enabled or disabled and can be changed via MC_Power. However, as long
   * as the error is pend-
   * ing the state remains ‘ErrorStop’.
   * The intention of the ‘ErrorStop’ state is that the axis goes to a stop, if
   * possible. There is no further
   * motion command accepted until a reset has been done from the ‘ErrorStop’
   * state.
   * The transition to ‘ErrorStop’ refers to errors from the axis and axis
   * control, and not from the Function
   * Block instances. These axis errors may also be reflected in the output of
   * the Function Blocks ‘FB
   * instances errors’.
   */
  mcErrorStop = 7,
} MC_AXIS_STATES;

/* Motion direction */
typedef enum {
  mcPositiveDirection = 1,
  mcShortestWay = 2,
  mcNegativeDirection = 3,
  mcCurrentDirection = 4,
} MC_DIRECTION;

/* Network error code */
typedef enum {
  mcNetworkGood = 0,
  mcNetworkConnectionBreak = 1,
} MC_NETWORK_ERROR_CODE;

/**
 * MC_BUFFER_MODE
 *
 */
typedef enum {
  /**
   * @brief Start FB immediately (default mode).
   * The next FB aborts an ongoing motion and the command
   * affects the axis immediately. The buffer is cleared.
   */
  mcAborting = 0,
  /**
   * @brief Start FB after current motion has finished.
   * The next FB affects the axis as soon as the previous movement is ‘Done’.
   * There is no blending.
   */
  mcBuffered = 1,
  /**
   * @brief The velocity is blended with the lowest velocity of both FBs.
   *
   */
  mcBlendingLow =
      2,  /// The velocity is blended with the lowest velocity of both FBs
  mcBlendingPrevious =
      3,  /// The velocity is blended with the velocity of the first FB
  mcBlendingNext =
      4,  /// The velocity is blended with velocity of the second FB
  mcBlendingHigh =
      5  /// The velocity is blended with highest velocity of both FBs
} MC_BUFFER_MODE;

typedef enum {
  mcNoMoveType   = 0,
  mcMoveAbsolute = 1,
  mcMoveRelative = 2,
  mcMoveAdditive = 3,
  mcMoveVelocity = 4,
  mcHalt = 5,
  mcStop = 6
} MC_MOTION_MODE;

}  // namespace RTmotion
