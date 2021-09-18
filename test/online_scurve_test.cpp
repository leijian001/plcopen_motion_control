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
 * @file s_curve_test.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include <RTmotion/algorithm/online_scurve_planner.hpp>
#include <thread>
#include <RTmotion/logging.hpp>
#include "gtest/gtest.h"

#ifdef PLOT
#include <RTmotion/plot/matplotlibcpp.h>
namespace plt = matplotlibcpp;
#endif

namespace traj_pro = trajectory_processing;

// The fixture for testing class Foo.
class ScurveTest : public ::testing::Test
{
protected:
  // You can remove any or all of the following functions if their bodies would
  // be empty.

  ScurveTest()
  {
    // You can do set-up work for each test here.
  }

  ~ScurveTest() override
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

#ifdef PLOT
  void TrajectoryPlot(double delta_time, double T = 10,
                      std::string file_name = "scurve.png")
  {
    double iter_t = 0;
    std::vector<double> pos, vel, acc, jerk, t;
    while (iter_t < T)
    {
      Eigen::Vector4d point = planner_.getWaypoint(iter_t);
      pos.push_back(point[POSITION_ID]);
      vel.push_back(point[SPEED_ID]);
      acc.push_back(point[ACCELERATION_ID]);
      jerk.push_back(point[JERK_ID]);
      t.push_back(iter_t);
      iter_t += delta_time;
    }

    // Set the size of output image to 1280x720 pixels
    plt::figure_size(1280, 720);
    // Plot line from given t and pos.
    plt::subplot(4, 1, 1);
    plt::title("Scurve Figure");
    plt::named_plot("Position", t, pos, "tab:blue");
    plt::ylabel("Position");
    // Plot line from given t and pos.
    plt::subplot(4, 1, 2);
    plt::named_plot("Velocity", t, vel, "tab:orange");
    plt::ylabel("Velocity");
    // Plot line from given t and pos.
    plt::subplot(4, 1, 3);
    plt::named_plot("Acceleration", t, acc, "tab:green");
    plt::ylabel("Acceleration");
    // Plot line from given t and pos.
    plt::subplot(4, 1, 4);
    plt::named_plot("Jerk", t, jerk, "tab:red");
    plt::xlabel("time");
    plt::ylabel("Jerk");
    // Save the image (file format is determined by the extension)
    plt::save(file_name);
  }
#endif

  traj_pro::ScurvePlannerOnLine planner_;
};

// Test 1
TEST_F(ScurveTest, Test1)
{
  planner_.reset();
  planner_.condition_.q0 = 0;
  planner_.condition_.q1 = 50;
  planner_.condition_.v0 = 0;
  planner_.condition_.v1 = 0;
  planner_.condition_.a0 = 0;
  planner_.condition_.a1 = 0;
  planner_.condition_.v_max = 10;
  planner_.condition_.a_max = 10;
  planner_.condition_.j_max = 30;

  planner_.plan();

#ifdef PLOT
  TrajectoryPlot(0.001, 8, "Test1.png");
  INFO_PRINT("Scurve profile: Tj2a %f, Tj2b %f, Td %f\n",
             planner_.profile_.Tj2a, planner_.profile_.Tj2b,
             planner_.profile_.Td);
  INFO_PRINT("Scurve termination: sk_e %f, vk_e %f, ak_e %f\n",
             planner_.sk_ - 50, planner_.vk_ - 0, planner_.ak_ - 0);
  ASSERT_LT(abs(planner_.sk_ - 50), 0.01);
  ASSERT_LT(abs(planner_.vk_ - 0), 0.05);
  ASSERT_LT(abs(planner_.ak_ - 0), 0.05);
#endif
}

// Test 2
TEST_F(ScurveTest, Test2)
{
  planner_.reset();
  planner_.condition_.q0 = 0;
  planner_.condition_.q1 = 5;
  planner_.condition_.v0 = 0;
  planner_.condition_.v1 = 0;
  planner_.condition_.a0 = 0;
  planner_.condition_.a1 = 0;
  planner_.condition_.v_max = 10;
  planner_.condition_.a_max = 10;
  planner_.condition_.j_max = 30;

  planner_.plan();

#ifdef PLOT
  TrajectoryPlot(0.001, 2, "Test2.png");
  INFO_PRINT("Scurve profile: Tj2a %f, Tj2b %f, Td %f\n",
             planner_.profile_.Tj2a, planner_.profile_.Tj2b,
             planner_.profile_.Td);
  INFO_PRINT("Scurve termination: sk_e %f, vk_e %f, ak_e %f\n",
             planner_.sk_ - 5, planner_.vk_ - 0, planner_.ak_ - 0);
  ASSERT_LT(abs(planner_.sk_ - 5), 0.02);
  ASSERT_LT(abs(planner_.vk_ - 0), 0.05);
  ASSERT_LT(abs(planner_.ak_ - 0), 0.05);
#endif
}

// Test 3
TEST_F(ScurveTest, Test3)
{
  planner_.reset();
  planner_.condition_.q0 = 0;
  planner_.condition_.q1 = 5;
  planner_.condition_.v0 = 5;
  planner_.condition_.v1 = 3;
  planner_.condition_.a0 = 3;
  planner_.condition_.a1 = 5;
  planner_.condition_.v_max = 10;
  planner_.condition_.a_max = 10;
  planner_.condition_.j_max = 30;

  planner_.plan();

#ifdef PLOT
  TrajectoryPlot(0.001, 1.5, "Test3.png");
  INFO_PRINT("Scurve profile: Tj2a %f, Tj2b %f, Td %f\n",
             planner_.profile_.Tj2a, planner_.profile_.Tj2b,
             planner_.profile_.Td);
  INFO_PRINT("Scurve termination: sk_e %f, vk_e %f, ak_e %f\n",
             planner_.sk_ - 5, planner_.vk_ - 3, planner_.ak_ - 5);
  ASSERT_LT(abs(planner_.sk_ - 5), 0.02);
  ASSERT_LT(abs(planner_.vk_ - 3), 0.05);
  ASSERT_LT(abs(planner_.ak_ - 5), 0.05);
#endif
}

// Test 4
TEST_F(ScurveTest, Test4)
{
  planner_.reset();
  planner_.condition_.q0 = 0;
  planner_.condition_.q1 = 5;
  planner_.condition_.v0 = 5;
  planner_.condition_.v1 = 3;
  planner_.condition_.a0 = -4;
  planner_.condition_.a1 = 2;
  planner_.condition_.v_max = 10;
  planner_.condition_.a_max = 10;
  planner_.condition_.j_max = 30;

  planner_.plan();

#ifdef PLOT
  TrajectoryPlot(0.001, 1.5, "Test4.png");
  INFO_PRINT("Scurve profile: Tj2a %f, Tj2b %f, Td %f\n",
             planner_.profile_.Tj2a, planner_.profile_.Tj2b,
             planner_.profile_.Td);
  INFO_PRINT("Scurve termination: sk_e %f, vk_e %f, ak_e %f\n",
             planner_.sk_ - 5, planner_.vk_ - 3, planner_.ak_ - 2);
  ASSERT_LT(abs(planner_.sk_ - 5), 0.01);
  ASSERT_LT(abs(planner_.vk_ - 3), 0.05);
  ASSERT_LT(abs(planner_.ak_ - 2), 4);
#endif
}

// Test 5
TEST_F(ScurveTest, Test5)
{
  planner_.reset();
  planner_.condition_.q0 = 0;
  planner_.condition_.q1 = 50;
  planner_.condition_.v0 = 15;
  planner_.condition_.v1 = 3;
  planner_.condition_.a0 = -4;
  planner_.condition_.a1 = 2;
  planner_.condition_.v_max = 10;
  planner_.condition_.a_max = 10;
  planner_.condition_.j_max = 30;

  planner_.plan();

#ifdef PLOT
  TrajectoryPlot(0.001, 6, "Test5.png");
  INFO_PRINT("Scurve profile: Tj2a %f, Tj2b %f, Td %f\n",
             planner_.profile_.Tj2a, planner_.profile_.Tj2b,
             planner_.profile_.Td);
  INFO_PRINT("Scurve termination: sk_e %f, vk_e %f, ak_e %f\n",
             planner_.sk_ - 50, planner_.vk_ - 3, planner_.ak_ - 2);
  ASSERT_LT(abs(planner_.sk_ - 50), 0.01);
  ASSERT_LT(abs(planner_.vk_ - 3), 0.05);
  ASSERT_LT(abs(planner_.ak_ - 2), 0.05);
#endif
}

// Test 6
TEST_F(ScurveTest, Test6)
{
  planner_.reset();
  planner_.condition_.q0 = 0;
  planner_.condition_.q1 = 50;
  planner_.condition_.v0 = 15;
  planner_.condition_.v1 = 20;
  planner_.condition_.a0 = -4;
  planner_.condition_.a1 = 2;
  planner_.condition_.v_max = 10;
  planner_.condition_.a_max = 10;
  planner_.condition_.j_max = 30;

  planner_.plan();

#ifdef PLOT
  TrajectoryPlot(0.001, 6, "Test6.png");
  INFO_PRINT("Scurve profile: Tj2a %f, Tj2b %f, Td %f\n",
             planner_.profile_.Tj2a, planner_.profile_.Tj2b,
             planner_.profile_.Td);
  INFO_PRINT("Scurve termination: sk_e %f, vk_e %f, ak_e %f\n",
             planner_.sk_ - 50, planner_.vk_ - 20, planner_.ak_ - 2);
  ASSERT_LT(abs(planner_.sk_ - 50), 0.025);
  ASSERT_LT(abs(planner_.vk_ - 20), 0.05);
  ASSERT_LT(abs(planner_.ak_ - 2), 0.05);
#endif
}

// Test 7
TEST_F(ScurveTest, Test7)
{
  planner_.reset();
  planner_.condition_.q0 = 0;
  planner_.condition_.q1 = 20;
  planner_.condition_.v0 = 15;
  planner_.condition_.v1 = 20;
  planner_.condition_.a0 = -4;
  planner_.condition_.a1 = 2;
  planner_.condition_.v_max = 10;
  planner_.condition_.a_max = 10;
  planner_.condition_.j_max = 30;

  planner_.plan();

#ifdef PLOT
  TrajectoryPlot(0.001, 2, "Test7.png");
  INFO_PRINT("Scurve profile: Tj2a %f, Tj2b %f, Td %f\n",
             planner_.profile_.Tj2a, planner_.profile_.Tj2b,
             planner_.profile_.Td);
  INFO_PRINT("Scurve termination: sk_e %f, vk_e %f, ak_e %f\n",
             planner_.sk_ - 20, planner_.vk_ - 20, planner_.ak_ - 2);
  ASSERT_LT(abs(planner_.sk_ - 20), 0.05);
  ASSERT_LT(abs(planner_.vk_ - 20), 0.05);
  ASSERT_LT(abs(planner_.ak_ - 2), 0.05);
#endif
}

// Test 8
TEST_F(ScurveTest, Test8)
{
  planner_.reset();
  planner_.condition_.q0 = 0;
  planner_.condition_.q1 = 3.14;
  planner_.condition_.v0 = 0;
  planner_.condition_.v1 = 0;
  planner_.condition_.a0 = 0;
  planner_.condition_.a1 = 0;
  planner_.condition_.v_max = 1.57;
  planner_.condition_.a_max = 3.14;
  planner_.condition_.j_max = 500;

  planner_.plan();

#ifdef PLOT
  TrajectoryPlot(0.001, 3, "Test8.png");
  INFO_PRINT("Scurve profile: Tj2a %f, Tj2b %f, Td %f\n",
             planner_.profile_.Tj2a, planner_.profile_.Tj2b,
             planner_.profile_.Td);
  INFO_PRINT("Scurve termination: sk_e %f, vk_e %f, ak_e %f\n",
             planner_.sk_ - 3.14, planner_.vk_ - 0, planner_.ak_ - 0);
  ASSERT_LT(abs(planner_.sk_ - 3.14), 0.02);
  ASSERT_LT(abs(planner_.vk_ - 0), 0.1);
  ASSERT_LT(abs(planner_.ak_ - 0), 1);
#endif
}

// Test 9
TEST_F(ScurveTest, Test9)
{
  planner_.reset();
  planner_.condition_.q0 = 0;
  planner_.condition_.q1 = 10;
  planner_.condition_.v0 = 1;
  planner_.condition_.v1 = 0;
  planner_.condition_.a0 = 0;
  planner_.condition_.a1 = 0;
  planner_.condition_.v_max = 10;
  planner_.condition_.a_max = 10;
  planner_.condition_.j_max = 30;

  planner_.plan();

#ifdef PLOT
  TrajectoryPlot(0.001, 3, "Test9.png");
  INFO_PRINT("Scurve profile: Tj2a %f, Tj2b %f, Td %f\n",
             planner_.profile_.Tj2a, planner_.profile_.Tj2b,
             planner_.profile_.Td);
  INFO_PRINT("Scurve termination: sk_e %f, vk_e %f, ak_e %f\n",
             planner_.sk_ - 10, planner_.vk_ - 0, planner_.ak_ - 0);
  ASSERT_LT(abs(planner_.sk_ - 10), 0.03);
  ASSERT_LT(abs(planner_.vk_ - 0), 0.1);
  ASSERT_LT(abs(planner_.ak_ - 0), 1);
#endif
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
