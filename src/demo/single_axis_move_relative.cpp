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
 * @file single_axis_move_relative.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include <RTmotion/axis.hpp>
#include <RTmotion/fb/fb_move_relative.hpp>
#include <RTmotion/global.hpp>

#include <thread>

#include <errno.h>
#include <mqueue.h>
#include <signal.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <limits.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/shm.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/time.h>
#include <time.h>
#include <getopt.h>

#define CYCLE_US 1000
#define BUFFER_SIZE 1000

static volatile int run = 1;
static pthread_t cyclic_thread;
static int64_t* execute_time;

static double running_time = 1.0;
static unsigned int running_time_t = (unsigned int)(BUFFER_SIZE * running_time);

static std::map<std::string, double> time_stamps;

void* my_thread(void* arg)
{
  RTmotion::AXIS_REF axis;
  axis = std::make_shared<RTmotion::Axis>();
  axis->setAxisId(1);
  axis->setAxisName("X");
  printf("Axis initialized.\n");

  std::shared_ptr<RTmotion::Servo> servo;
  servo = std::make_shared<RTmotion::Servo>();
  axis->setServo(servo);

  RTmotion::FbMoveRelative fb_move_abs;
  fb_move_abs.setAxis(axis);
  fb_move_abs.setContinuousUpdate(false);
  fb_move_abs.setDistance(3.14);
  fb_move_abs.setVelocity(1.57);
  fb_move_abs.setAcceleration(3.14);
  fb_move_abs.setJerk(5000);
  fb_move_abs.setTimeRecodPtr(&time_stamps);
  printf("Function block initialized.\n");

  struct timespec next_period, start_time, end_time;
  unsigned int cycle_counter = 0;

  struct sched_param param = {};
  param.sched_priority = 99;
  pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  clock_gettime(CLOCK_MONOTONIC, &next_period);

  while (run != 0)
  {
    next_period.tv_nsec += CYCLE_US * 1000;
    while (next_period.tv_nsec >= NSEC_PER_SEC)
    {
      next_period.tv_nsec -= NSEC_PER_SEC;
      next_period.tv_sec++;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);

    clock_gettime(CLOCK_MONOTONIC, &start_time);

    axis->runCycle();
    fb_move_abs.runCycle();

    clock_gettime(CLOCK_MONOTONIC, &end_time);
    if (cycle_counter < running_time_t)
    {
      execute_time[cycle_counter] = DIFF_NS(start_time, end_time);
    }
    else
      run = 0;

    if (!fb_move_abs.isEnabled())
    {
      // printf("Enable the function block\n");
      fb_move_abs.setExecute(true);
      execute_time[++cycle_counter] = 0;
    }

    if (fb_move_abs.isDone())
    {
      // printf("Function block MC_MoveRelative finished.\n");
      fb_move_abs.setExecute(false);
    }

    cycle_counter++;
  }
  return NULL;
}

void signal_handler(int sig)
{
  run = 0;
}

static void getOptions(int argc, char** argv)
{
  int index;
  static struct option longOptions[] = {
    // name		has_arg				flag	val
    { "time", required_argument, NULL, 't' },
    { "help", no_argument, NULL, 'h' },
    {}
  };
  do
  {
    index = getopt_long(argc, argv, "t:h", longOptions, NULL);
    switch (index)
    {
      case 't':
        running_time = atof(optarg);
        running_time_t = (unsigned int)(BUFFER_SIZE * running_time);
        printf("Time: Set running time to %d ms\n", running_time_t);
        break;
      case 'h':
        printf("Global options:\n");
        printf("    --time  -t  Set running time(s).\n");
        printf("    --help  -h  Show this help.\n");
        exit(0);
        break;
    }
  } while (index != -1);
}

/****************************************************************************
 * Main function
 ***************************************************************************/
int main(int argc, char* argv[])
{
  getOptions(argc, argv);
  execute_time = (int64_t*)malloc(sizeof(int64_t) * int(running_time_t));
  signal(SIGTERM, signal_handler);
  signal(SIGINT, signal_handler);
  mlockall(MCL_CURRENT | MCL_FUTURE);

  /* Create cyclic RT-thread */
  pthread_attr_t thattr;
  pthread_attr_init(&thattr);
  pthread_attr_setdetachstate(&thattr, PTHREAD_CREATE_JOINABLE);

  if (pthread_create(&cyclic_thread, &thattr, &my_thread, NULL))
  {
    fprintf(stderr, "pthread_create cyclic task failed\n");
    return 1;
  }

  while (run)
  {
    sched_yield();
  }

  pthread_join(cyclic_thread, NULL);

  FILE* fptr;

  // Write cycle running time data
  if ((fptr = fopen("data.log", "wb")) == NULL)
  {
    printf("Error! opening file: data.log");
    exit(1);  // Program exits if the file pointer returns NULL.
  }

  for (size_t i = 0; i < running_time_t; ++i)
  {
    fprintf(fptr, "%ld\n", execute_time[i]);
  }
  fclose(fptr);

  // Write time stamps data
  if ((fptr = fopen("time_stamps.log", "wb")) == NULL)
  {
    printf("Error! opening file: time_stamps.log");
    exit(1);  // Program exits if the file pointer returns NULL.
  }

  for (auto time_stamp : time_stamps)
  {
    fprintf(fptr, "%s: %f\n", time_stamp.first.c_str(), time_stamp.second);
  }
  fclose(fptr);

  printf("End of Program\n");
  return 0;
}

/****************************************************************************/
