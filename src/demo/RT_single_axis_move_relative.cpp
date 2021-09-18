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

#include <errno.h>
#include <error.h>
#include <getopt.h>
#include <mqueue.h>
#include <pthread.h>
#include <semaphore.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <thread>
#include <unistd.h>

#define NSEC_PER_SEC (1000000000L)
#define TIMESPEC2NS(T) ((uint64_t)(T).tv_sec * NSEC_PER_SEC + (T).tv_nsec)
#define DIFF_NS(A, B)                                                          \
  (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + ((B).tv_nsec) - (A).tv_nsec)
#define CYCLE_COUNTER_PERSEC(X) (NSEC_PER_SEC / 1000 / X)

static unsigned int cycle_counter = 0;
static int64_t avg_cycle_time;
static int64_t min_cycle_time;
static int64_t max_cycle_time;
static int64_t min_jitter_time;
static int64_t max_jitter_time;
static int64_t total_cycle_ns;

static pthread_t log_thread;
static sem_t* log_sem;
static pthread_t cyclic_thread;
static volatile int run = 1;
static uint32_t cycle_time = 250;

static void* rt_thread(void* cookie)
{
  struct timespec next_period;
  uint8_t servo_run = 0;

  struct sched_param param = {};
  param.sched_priority = 99;
  pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

  struct timespec startTime, endTime, lastStartTime = {};
  int64_t period_ns = 0, exec_ns = 0, period_min_ns = 1000000,
          period_max_ns = 0, exec_min_ns = 1000000, exec_max_ns = 0;
  int64_t latency_ns = 0;
  int64_t latency_min_ns = 1000000, latency_max_ns = -1000000;
  int64_t total_exec_ns = 0;
  avg_cycle_time = 0;
  min_cycle_time = 1000000;
  max_cycle_time = -1000000;
  min_jitter_time = 1000000;
  max_jitter_time = -1000000;

  /* lib init */
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
  fb_move_abs.setExecute(true);
  fb_move_abs.setContinuousUpdate(false);
  fb_move_abs.setDistance(3.14);
  fb_move_abs.setVelocity(1.57);
  fb_move_abs.setAcceleration(3.14);
  fb_move_abs.setJerk(5000);
  printf("Function block initialized.\n");

  clock_gettime(CLOCK_MONOTONIC, &next_period);
  lastStartTime = next_period;
  endTime = next_period;

  while (run)
  {
    next_period.tv_nsec += cycle_time * 1000;
    while (next_period.tv_nsec >= NSEC_PER_SEC)
    {
      next_period.tv_nsec -= NSEC_PER_SEC;
      next_period.tv_sec++;
    }

    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_period, NULL);

    clock_gettime(CLOCK_MONOTONIC, &startTime);
    latency_ns = DIFF_NS(next_period, startTime);
    period_ns = DIFF_NS(lastStartTime, startTime);
    exec_ns = DIFF_NS(lastStartTime, endTime);
    lastStartTime = startTime;

    if (latency_ns > latency_max_ns)
    {
      latency_max_ns = latency_ns;
    }
    if (latency_ns < latency_min_ns)
    {
      latency_min_ns = latency_ns;
    }
    if (period_ns > period_max_ns)
    {
      period_max_ns = period_ns;
    }
    if (period_ns < period_min_ns)
    {
      period_min_ns = period_ns;
    }
    if (exec_ns > exec_max_ns)
    {
      exec_max_ns = exec_ns;
    }
    if (exec_ns < exec_min_ns)
    {
      exec_min_ns = exec_ns;
    }
    total_exec_ns += exec_ns;

    cycle_counter++;

    /* task 1s */
    if (!(cycle_counter % CYCLE_COUNTER_PERSEC(cycle_time)))
    {
      if (min_cycle_time > exec_min_ns)
        min_cycle_time = exec_min_ns;
      if (max_cycle_time < exec_max_ns)
        max_cycle_time = exec_max_ns;
      if (min_jitter_time > latency_min_ns)
        min_jitter_time = latency_min_ns;
      if (max_jitter_time < latency_max_ns)
        max_jitter_time = latency_max_ns;

      period_max_ns = -1000000;
      period_min_ns = 1000000;
      exec_max_ns = -1000000;
      exec_min_ns = 1000000;
      latency_max_ns = -1000000;
      latency_min_ns = 1000000;
      total_cycle_ns += total_exec_ns;
      total_exec_ns = 0;

      if (servo_run == 0)
      {
        cycle_counter = 0;
        total_cycle_ns = 0;
        min_jitter_time = 1000000;
        max_jitter_time = -1000000;
        min_cycle_time = 1000000;
        max_cycle_time = -1000000;
        servo_run = 1;
      }
      sem_post(log_sem);
    }

    /* cycle process */
    axis->runCycle();
    fb_move_abs.runCycle();

    if (!fb_move_abs.isEnabled())
    {
      printf("Enable the function block\n");
      fb_move_abs.setExecute(true);
    }

    if (fb_move_abs.isDone())
    {
      printf("Function block MC_MoveRelative finished.\n");
      fb_move_abs.setExecute(false);
    }

    clock_gettime(CLOCK_MONOTONIC, &endTime);
  }

  avg_cycle_time = total_cycle_ns / cycle_counter;
  printf("*********************************************\n");
  printf("average cycle time  %10.3f\n", (float)avg_cycle_time / 1000);
  printf("cycle counter       %10d\n", cycle_counter);
  printf("cycle time          %10.3f ... %10.3f\n",
         (float)min_cycle_time / 1000, (float)max_cycle_time / 1000);
  printf("jitter time         %10.3f ... %10.3f\n",
         (float)min_jitter_time / 1000, (float)max_jitter_time / 1000);
  printf("*********************************************\n");

  return NULL;
}

void signal_handler(int sig)
{
  run = 0;
}

static void setup_sched_parameters(pthread_attr_t* attr, int prio)
{
  struct sched_param p;
  int ret;

  ret = pthread_attr_init(attr);
  if (ret)
    error(1, ret, "pthread_attr_init()");

  ret = pthread_attr_setinheritsched(attr, PTHREAD_EXPLICIT_SCHED);
  if (ret)
    error(1, ret, "pthread_attr_setinheritsched()");

  ret = pthread_attr_setschedpolicy(attr, prio ? SCHED_FIFO : SCHED_OTHER);
  if (ret)
    error(1, ret, "pthread_attr_setschedpolicy()");

  p.sched_priority = prio;
  ret = pthread_attr_setschedparam(attr, &p);
  if (ret)
    error(1, ret, "pthread_attr_setschedparam()");
}

static void* log_proc(void* cookie)
{
  while (run)
  {
    int err;
    err = sem_wait(log_sem);
    if (err < 0)
    {
      if (errno != EIDRM)
        error(1, errno, "sem_wait()");
      break;
    }

    printf("time:       %10ld\n",
           cycle_counter / CYCLE_COUNTER_PERSEC(cycle_time));
    printf("[CYCLE]   min:%10.3f    max:%10.3f\n", (float)min_cycle_time / 1000,
           (float)max_cycle_time / 1000);
    printf("[JITTER]  min:%10.3f    max:%10.3f\n",
           (float)min_jitter_time / 1000, (float)max_jitter_time / 1000);
  }

  return NULL;
}

static void getOptions(int argc, char** argv)
{
  int index;
  static struct option longOptions[] = {
    /* name         has_arg flag    val */
    { "cycle", required_argument, NULL, 't' },
    { "help", no_argument, NULL, 'h' },
    {}
  };
  do
  {
    index = getopt_long(argc, argv, "c:h", longOptions, NULL);
    switch (index)
    {
      case 'c':
        cycle_time = atoi(optarg);
        printf("cycle time: %dus\n", cycle_time);
        break;
      case 'h':
        printf("Global options:\n");
        printf("  --cycle  -c  Set cycle time for microsecond.\n");
        printf("  --help   -h  Show this help.\n");
        printf("default  cycle time:%dus\n", cycle_time);
        exit(0);
        break;
      default:
        break;
    }
  } while (index != -1);
}

int main(int argc, char* argv[])
{
  int ret;
  char sem_name[16];
  pthread_attr_t tattr;

  getOptions(argc, argv);

  signal(SIGTERM, signal_handler);
  signal(SIGINT, signal_handler);

  mlockall(MCL_CURRENT | MCL_FUTURE);

  /* create sem */
  snprintf(sem_name, sizeof(sem_name), "/logsem-%d", getpid());
  sem_unlink(sem_name); /* may fail */
  log_sem = sem_open(sem_name, O_CREAT | O_EXCL, 0666, 0);
  if (log_sem == SEM_FAILED)
    error(1, errno, "sem_open()");

  printf("starting threading\n");
  /* create log thread */
  setup_sched_parameters(&tattr, 0);
  ret = pthread_create(&log_thread, &tattr, log_proc, NULL);
  if (ret)
    error(1, ret, "pthread_create(latency)");
  pthread_attr_destroy(&tattr);

  /* create cyclic thread */
  setup_sched_parameters(&tattr, 99);
  ret = pthread_create(&cyclic_thread, &tattr, rt_thread, NULL);
  if (ret)
    error(1, ret, "pthread_create(latency)");
  pthread_attr_destroy(&tattr);

  while (run)
    sched_yield();

  usleep(10000);
  pthread_cancel(log_thread);
  pthread_join(log_thread, NULL);
  pthread_cancel(cyclic_thread);
  pthread_join(cyclic_thread, NULL);

  sem_close(log_sem);
  sem_unlink(sem_name);

  return 0;
}
