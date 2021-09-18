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
 * @file motion_kernel.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include <RTmotion/motion_kernel.hpp>
#include <RTmotion/logging.hpp>

namespace RTmotion
{
MotionKernel::MotionKernel()
{
}

MotionKernel::~MotionKernel()
{
}

void MotionKernel::runCycle(const double t, const double t_prev, const double pos, const double vel, MC_AXIS_STATES* state)
{
  if (!fb_queue_.empty())
  {
    ExecutionNode* fb_front = fb_queue_.front();
    DEBUG_PRINT("MotionKernel::runCycle fb_queue_ is not empty %p.\n", (void*)fb_front);

    // Check if the motion kernel mission is done
    if (fb_front->checkMissionDone(pos, vel))
      fb_front->onDone(state);

    // If the first FB is aborted or in error, remove it from the execution queue
    if (fb_front->isAborted() || fb_front->isError())
    {
      DEBUG_PRINT("Abort front FB %p.\n", (void*)fb_front);
      fb_front->taken_ = false;
      fb_queue_.pop();
    }

    // If the first FB is done, move it to be hold
    if (fb_front->isDone())
    {
      if (fb_hold_)
      {
        DEBUG_PRINT("Abort hold FB %p.\n", (void*)fb_hold_);
        fb_hold_->taken_ = false;
        fb_hold_ = nullptr;
      }
      fb_hold_ = fb_front;
      fb_queue_.pop();
      DEBUG_PRINT("Runtime: %f, FB is done, hold it. FB queue is empty? %d\n", t, fb_queue_.empty());
    }

    // If there is any FB in the queue, activate it
    if (!fb_queue_.empty())
    {
      ExecutionNode* fb_front = fb_queue_.front();
      DEBUG_PRINT("Front node is active? %d, is enabled? %d.\n", fb_front->isActive(), fb_front->getAxisNodeInstance()->isEnabled());
      if (!fb_front->isActive() && fb_front->getAxisNodeInstance()->isEnabled())
      {
        fb_front->onActive(t_prev, state); // Activate the first node from queue
      }

      if (fb_front->isActive()) // If the first FB is active, execute it
      {
        DEBUG_PRINT("FB queue is empty, FB mode is %d.\n", fb_front->getBufferMode());
        if (fb_front->getBufferMode() == mcAborting)
        {
          DEBUG_PRINT("FB hold is %p.\n", (void*)fb_hold_);
          if (fb_hold_) // Since the front FB from queue is ready to execute, remove the hold FB
          {
            DEBUG_PRINT("Abort hold FB %d.\n", fb_hold_->getMotionMode());
            fb_hold_->onCommandAborted();
            fb_hold_->taken_ = false;
            fb_hold_ = nullptr;
          }
        }

        fb_front->onExecution(t);
      }
    }
  }
}

void MotionKernel::addFBToQueue(ExecutionNode* node, LREAL current_pos,
                                LREAL current_vel)
{
  node->setReplan();
  DEBUG_PRINT("MotionKernel::addFBToQueue: current_pos: %f, current_vel: %f\n", current_pos, current_vel);

  if (fb_queue_.empty()) // Probably there is a hold node
  {
    node->setStartPos(current_pos);
    node->setStartVel(current_vel);
    node->setTerminateCondition(current_pos);
    DEBUG_PRINT("MotionKernel::addFBToQueue: fb_queue (size %zu) is empty\n", fb_queue_.size());
  }
  else  // If queue is not empty, it's not possible to have a hold node
  {
    ExecutionNode* fb_prev = fb_queue_.back();

    // If not aborting, then set the end velocity of the previous FB
    switch (node->getBufferMode())
    {
      case mcBuffered:
        break;
      case mcAborting:
        node->setStartPos(current_pos);
        node->setStartVel(current_vel);
        node->setTerminateCondition(fb_prev->getEndPos());
        setAllFBsAborted();
        break;
      case mcBlendingPrevious:
      {
        fb_prev->setEndVel(fb_prev->getVelocity());
        fb_prev->setReplan();
      }
      break;
      case mcBlendingNext:
      {
        fb_prev->setEndVel(node->getVelocity());
        fb_prev->setReplan();
      }
      break;
      case mcBlendingHigh:
      {
        fb_prev->setEndVel(
            std::max(fb_prev->getVelocity(), node->getVelocity()));
        fb_prev->setReplan();
      }
      break;
      case mcBlendingLow:
      {
        fb_prev->setEndVel(
            std::min(fb_prev->getVelocity(), node->getVelocity()));
        fb_prev->setReplan();
      }
      break;
      default:
        break;
    }

    if (node->getBufferMode() != mcAborting)
    {
      node->setStartPos(fb_prev->getEndPos());
      node->setStartVel(fb_prev->getEndVel());
      node->setTerminateCondition(fb_prev->getEndPos());
    }
  }

  fb_queue_.push(node);
}

std::queue<ExecutionNode*>& MotionKernel::getQueuedMotions()
{
  return fb_queue_;
}

void MotionKernel::setAllFBsAborted()
{
  while (!fb_queue_.empty())
  {
    ExecutionNode* fb = fb_queue_.front();
    fb->onCommandAborted();
    fb->taken_ = false;
    fb_queue_.pop();
  }
  if (fb_hold_)
  {
    fb_hold_->onCommandAborted();
    fb_hold_->taken_ = false;
    fb_hold_ = nullptr;
  }
}

void MotionKernel::getCommands(double* pos_cmd, double* vel_cmd,
                               double duration)
{
  if (!fb_queue_.empty())
  {
    DEBUG_PRINT("MotionKernel::getCommands: fb_queue_ (size %zu) is not empty\n", fb_queue_.size());
    fb_queue_.front()->getCommands(pos_cmd, vel_cmd, duration);
  }
  else
  {
    DEBUG_PRINT("MotionKernel::getCommands: fb_queue_ (size %zu) is empty\n", fb_queue_.size());
    if (fb_hold_)
      fb_hold_->getCommands(pos_cmd, vel_cmd, duration);
  }
}

}  // namespace RTmotion
