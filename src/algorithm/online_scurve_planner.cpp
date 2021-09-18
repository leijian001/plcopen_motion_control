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
 * @file online_scurve_planner.cpp
 *
 * Maintainer: Yu Yan <yu.yan@intel.com>
 *
 */

#include <RTmotion/algorithm/online_scurve_planner.hpp>
#include <RTmotion/logging.hpp>
#include <chrono>
#include <RTmotion/algorithm/math_utils.hpp>

#define POLYNOMIAL_STOP

using namespace RTmotion;

constexpr double EPSILON = 0.01;

namespace trajectory_processing
{
void ScurvePlannerOnLine::computeProfile(double jk, double t, double& ak,
                                         double& vk,
                                         double& sk)  // Equation (1)
{
  // Equation (1)
  ak = ak_ + (t - tk_) * (jk_ + jk) / 2.0;
  vk = vk_ + (t - tk_) * (ak_ + ak) / 2.0;
  sk = sk_ + (t - tk_) * (vk_ + vk) / 2.0;
}

void ScurvePlannerOnLine::computeStopPhaseDccProfile(double& jk, double& hk,
                                                     double& Tj2a, double& Tj2b,
                                                     double& Td)
{
  double Amin = -condition_.a_max;
  double Jmin = -condition_.j_max;
  double Jmax = condition_.j_max;
  double Ve = condition_.v1;
  double Ae = condition_.a1;

  // Equation (4)
  Tj2a = (Amin - ak_) / Jmin;
  Tj2b = (Ae - Amin) / Jmax;
  Td = (Ve - vk_) / Amin + Tj2a * (Amin - ak_) / (2 * Amin) +
       Tj2b * (Amin - Ae) / (2 * Amin);
  if (Td - (Tj2a + Tj2b) < 0)
  {
    // Equation (5)
    Tj2a = -ak_ / Jmin +
           sqrt((Jmax - Jmin) *
                (__square(ak_) * Jmax -
                 Jmin * (__square(Ae) + 2.0 * Jmax * (vk_ - Ve)))) /
               (Jmin * (Jmin - Jmax));
    Tj2b = -Ae / Jmax +
           sqrt((Jmax - Jmin) *
                (__square(ak_) * Jmax -
                 Jmin * (__square(Ae) + 2.0 * Jmax * (vk_ - Ve)))) /
               (Jmin * (Jmin - Jmax));
    Td = Tj2a + Tj2b;
  }
  // Equation (6)
  hk = ak_ * __square(Td) / 2.0 +
       (Jmin * Tj2a * (3.0 * __square(Td) - 3.0 * Td * Tj2a + __square(Tj2a)) +
        Jmax * __cube(Tj2b)) /
           6.0 +
       Td * vk_;
}

void ScurvePlannerOnLine::computeStopPhaseAccProfile(double& jk, double& hk,
                                                     double& Tj2a, double& Tj2b,
                                                     double& Td)
{
  double Amax = condition_.a_max;
  double Jmin = -condition_.j_max;
  double Jmax = condition_.j_max;
  double Ve = condition_.v1;
  double Ae = condition_.a1;

  // Equation (8)
  Tj2a = (Amax - ak_) / Jmax;
  Tj2b = (Ae - Amax) / Jmin;
  Td = (Ve - vk_) / Amax + Tj2a * (Amax - ak_) / (2.0 * Amax) +
       Tj2b * (Amax - Ae) / (2.0 * Amax);
  if (Td - (Tj2a + Tj2b) < 0)
  {
    // Equation (9)
    Tj2a = -ak_ / Jmax +
           sqrt((Jmin - Jmax) *
                (__square(ak_) * Jmin -
                 Jmax * (__square(Ae) + 2.0 * Jmin * (vk_ - Ve)))) /
               (Jmax * (Jmax - Jmin));
    Tj2b = -Ae / Jmin +
           sqrt((Jmin - Jmax) *
                (__square(ak_) * Jmin -
                 Jmax * (__square(Ae) + 2.0 * Jmin * (vk_ - Ve)))) /
               (Jmin * (Jmin - Jmax));
    Td = Tj2a + Tj2b;
  }

  // Equation (10)
  hk = ak_ * __square(Td) / 2.0 +
       (Jmax * Tj2a * (3.0 * __square(Td) - 3.0 * Td * Tj2a + __square(Tj2a)) +
        Jmin * __cube(Tj2b)) /
           6.0 +
       Td * vk_;
}

void ScurvePlannerOnLine::computeStopPhaseDcc(double t, double& jk, double& ak,
                                              double& vk, double& sk)
{
  double Td = profile_.Td;
  double Tj2a = profile_.Tj2a;
  double Tj2b = profile_.Tj2b;

  double Jmin = -condition_.j_max;
  double Jmax = condition_.j_max;
  double th = profile_.Th;

#ifdef POLYNOMIAL_STOP
  static double h, a0, a1, a2, a3, a4, a5;
  if (phase_ != JerkMin)
  {
    phase_ = JerkMin;
    s_init_ = sk_;
    v_init_ = vk_;
    a_init_ = ak_;
    t_init_ = tk_;
    h = fabs(condition_.q1 - s_init_);
    a0 = s_init_;
    a1 = v_init_;
    a2 = a_init_ / 2.0;
    a3 = (20 * h - (8 * condition_.v1 + 12 * v_init_) * Td - (3 * a_init_ - condition_.a1) * __square(Td)) / (2 * __cube(Td));
    a4 = (-30 * h + (14 * condition_.v1 + 16 * v_init_) * Td + (3 * a_init_ - 2 * condition_.a1) * __square(Td)) / (2 * __square(Td) * __square(Td));
    a5 = (12 * h - 6 * (condition_.v1 + v_init_) * Td + (condition_.a1 - a_init_) * __square(Td)) / (2 * __square(Td) * __cube(Td));
    printf("Polynomial coeffs: a0 %f, a0 %f, a0 %f, a0 %f, a0 %f, a0 %f\n", a0, a1, a2, a3, a4, a5);
  }

  if (t - th < Td)
  {
    double tt = t - t_init_;
    sk = s_init_ + a1 * tt + a2 * __square(tt) + a3 * __cube(tt) + a4 * __square(tt) * __square(tt) + a5 *  __square(tt) * __cube(tt);
    vk = v_init_ + 2 * a2 * tt + 3 * a3 * __square(tt) + 4 * a4 * __cube(tt) + 5 * a5 * __square(tt) * __square(tt);
    ak = a_init_ + 6 * a3 * tt + 12 * a4 * __square(tt) + 20 * a5 * __cube(tt);
    jk = 6 * a3 + 24 * a4 * tt + 60 * a5 * __square(tt);
  }
#else
  // Equation (7)
  if (t - th < Tj2a)
  {
    if (phase_ != JerkMin)
    {
      phase_ = JerkMin;
      s_init_ = sk_;
      v_init_ = vk_;
      a_init_ = ak_;
      t_init_ = tk_;
    }
    // double tt = t - th;
    double tt = t - t_init_;
    // (3.30e)
    jk = Jmin;
    ak = a_init_ - Jmax * tt;
    vk = v_init_ + a_init_ * tt - Jmax * __square(tt) / 2.0;
    sk = s_init_ + v_init_ * tt + a_init_ * __square(tt) / 2.0 -
         Jmax * __cube(tt) / 6.0;
  }
  if ((t - th) > Tj2a && (t - th) < (Td - Tj2b))
  {
    if (phase_ != JerkZero)
    {
      phase_ = JerkZero;
      s_init_ = sk_;
      v_init_ = vk_;
      a_init_ = ak_;
      t_init_ = tk_;
    }
    // double tt = t - th - Tj2a;
    double tt = t - t_init_;
    // (3.30f)
    jk = 0;
    ak = a_init_;
    vk = v_init_ + a_init_ * tt;
    sk = s_init_ + v_init_ * tt + a_init_ * __square(tt) / 2.0;
  }
  if ((t - th) > (Td - Tj2b) && (t - th) < Td)
  {
    if (phase_ != JerkMax)
    {
      phase_ = JerkMax;
      s_init_ = sk_;
      v_init_ = vk_;
      a_init_ = ak_;
      t_init_ = tk_;
    }
    // double tt = t - th - (Td - Tj2b);
    double tt = t - t_init_;
    // (3.30g)
    jk = Jmax;
    ak = a_init_ + Jmax * tt;
    vk = v_init_ + a_init_ * tt + Jmax * __square(tt) / 2.0;
    sk = s_init_ + v_init_ * tt + a_init_ * __square(tt) / 2.0 +
         Jmax * __cube(tt) / 6.0;
  }
#endif
  if ((t - th) >= Td)
  {
    jk = 0.0;
    ak = ak_;
    vk = vk_;
    sk = sk_;
  }
}

void ScurvePlannerOnLine::computeStopPhaseAcc(double t, double& jk, double& ak,
                                              double& vk, double& sk)
{
  double Td = profile_.Td;
  double Tj2a = profile_.Tj2a;
  double Tj2b = profile_.Tj2b;

  double Jmin = -condition_.j_max;
  double Jmax = condition_.j_max;
  double th = profile_.Th;

#ifdef POLYNOMIAL_STOP
  static double h, a0, a1, a2, a3, a4, a5;
  if (phase_ != JerkMax)
  {
    phase_ = JerkMax;
    s_init_ = sk_;
    v_init_ = vk_;
    a_init_ = ak_;
    t_init_ = tk_;
    h = fabs(condition_.q1 - s_init_);
    a0 = s_init_;
    a1 = v_init_;
    a2 = a_init_ / 2.0;
    a3 = (20 * h - (8 * condition_.v1 + 12 * v_init_) * Td - (3 * a_init_ - condition_.a1) * __square(Td)) / (2 * __cube(Td));
    a4 = (-30 * h + (14 * condition_.v1 + 16 * v_init_) * Td + (3 * a_init_ - 2 * condition_.a1) * __square(Td)) / (2 * __square(Td) * __square(Td));
    a5 = (12 * h - 6 * (condition_.v1 + v_init_) * Td + (condition_.a1 - a_init_) * __square(Td)) / (2 * __square(Td) * __cube(Td));
    printf("Polynomial coeffs: a0 %f, a0 %f, a0 %f, a0 %f, a0 %f, a0 %f\n", a0, a1, a2, a3, a4, a5);
  }

  if (t - th < Td)
  {
    double tt = t - t_init_;
    sk = s_init_ + a1 * tt + a2 * __square(tt) + a3 * __cube(tt) + a4 * __square(tt) * __square(tt) + a5 *  __square(tt) * __cube(tt);
    vk = v_init_ + 2 * a2 * tt + 3 * a3 * __square(tt) + 4 * a4 * __cube(tt) + 5 * a5 * __square(tt) * __square(tt);
    ak = a_init_ + 6 * a3 * tt + 12 * a4 * __square(tt) + 20 * a5 * __cube(tt);
    jk = 6 * a3 + 24 * a4 * tt + 60 * a5 * __square(tt);
  }
#else
  // Equation (11)
  if (t - th < Tj2a)
  {
    if (phase_ != JerkMax)
    {
      phase_ = JerkMax;
      s_init_ = sk_;
      v_init_ = vk_;
      a_init_ = ak_;
      t_init_ = tk_;
    }
    double tt = t - t_init_;
    // (3.30a)
    jk = Jmax;
    ak = a_init_ + Jmax * tt;
    vk = v_init_ + a_init_ * tt + Jmax * __square(tt) / 2.0;
    sk = s_init_ + v_init_ * tt + a_init_ * __square(tt) / 2.0 +
         Jmax * __cube(tt) / 6.0;
  }
  if ((t - th) > Tj2a && (t - th) < (Td - Tj2b))
  {
    if (phase_ != JerkZero)
    {
      phase_ = JerkZero;
      s_init_ = sk_;
      v_init_ = vk_;
      a_init_ = ak_;
      t_init_ = tk_;
    }
    double tt = t - t_init_;
    // (3.30b)
    jk = 0;
    ak = a_init_;
    vk = v_init_ + a_init_ * tt;
    sk = s_init_ + v_init_ * tt + a_init_ * __square(tt) / 2.0;
  }
  if ((t - th) > (Td - Tj2b) && sk_ < condition_.q1)
  {
    if (phase_ != JerkMin)
    {
      phase_ = JerkMin;
      s_init_ = sk_;
      v_init_ = vk_;
      a_init_ = ak_;
      t_init_ = tk_;
    }
    double tt = t - th - (Td - Tj2b);
    // (3.30c)
    jk = Jmin;
    ak = a_init_ - Jmax * tt;
    vk = v_init_ + a_init_ * tt - Jmax * __square(tt) / 2;
    sk = s_init_ + v_init_ * tt + a_init_ * __square(tt) / 2.0 -
         Jmax * __cube(tt) / 6.0;
  }
#endif
  if ((t - th) >= Td)
  {
    jk = 0.0;
    ak = ak_;
    vk = vk_;
    sk = sk_;
  }
}

void ScurvePlannerOnLine::computeFirstPhaseAcc(double t, double& jk, double& ak,
                                               double& vk,
                                               double& sk)  // v0 <= Vmax
{
  double Jmin = -condition_.j_max;
  double Jmax = condition_.j_max;
  double Amax = condition_.a_max;
  double Vmax = condition_.v_max;

  double tmp = vk_ - __square(ak_) / (2.0 * Jmin);
  // Equation (2)
  if (tmp < Vmax && ak_ < Amax)  // Accelerate increase
  {
    if (phase_ != JerkMax)
    {
      phase_ = JerkMax;
      s_init_ = sk_;
      v_init_ = vk_;
      a_init_ = ak_;
      t_init_ = tk_;
    }
    double tt = t - t_init_;
    // (3.30a)
    jk = Jmax;
    ak = a_init_ + Jmax * tt;
    vk = v_init_ + a_init_ * tt + Jmax * __square(tt) / 2.0;
    sk = s_init_ + v_init_ * tt + a_init_ * __square(tt) / 2.0 +
         Jmax * __cube(tt) / 6.0;
  }
  if (tmp < Vmax && ak_ >= Amax)  // Accelerate constant
  {
    if (phase_ != JerkZero)
    {
      phase_ = JerkZero;
      s_init_ = sk_;
      v_init_ = vk_;
      a_init_ = ak_;
      t_init_ = tk_;
    }
    double tt = t - t_init_;
    // (3.30b)
    jk = 0.0;
    ak = Amax;
    vk = v_init_ + Amax * tt;
    sk = s_init_ + v_init_ * tt + Amax * __square(tt) / 2.0;
  }
  if (tmp >= Vmax && ak_ > 0.0)  // Accelerate decrease
  {
    if (phase_ != JerkMin)
    {
      phase_ = JerkMin;
      s_init_ = sk_;
      v_init_ = vk_;
      a_init_ = ak_;
      t_init_ = tk_;
    }
    double tt = t - t_init_;
    // (3.30c)
    jk = Jmin;
    ak = a_init_ - Jmax * tt;
    vk = v_init_ + a_init_ * tt - Jmax * __square(tt) / 2;
    sk = s_init_ + v_init_ * tt + a_init_ * __square(tt) / 2.0 -
         Jmax * __cube(tt) / 6.0;
  }
  if (tmp >= Vmax && ak_ <= 0.0)  // Constant velocity
  {
    if (phase_ != NoJerk)
    {
      phase_ = NoJerk;
      s_init_ = sk_;
      v_init_ = vk_;
      a_init_ = ak_;
      t_init_ = tk_;
    }
    double tt = t - t_init_;
    // (3.30c)
    jk = 0.0;
    ak = 0.0;
    vk = v_init_;
    sk = s_init_ + v_init_ * tt;
  }
}

void ScurvePlannerOnLine::computeFirstPhaseDcc(double t, double& jk, double& ak,
                                               double& vk,
                                               double& sk)  // v0 > Vmax
{
  double Jmin = -condition_.j_max;
  double Jmax = condition_.j_max;
  double Amin = -condition_.a_max;
  double Vmax = condition_.v_max;
  // double ak = ak_;

  double tmp = vk_ - __square(ak_) / (2.0 * Jmax);
  // Equation (3)
  // double jk = 0.0;
  if (tmp > Vmax && ak_ > Amin)  // Accelerate decrease
  {
    if (phase_ != JerkMin)
    {
      phase_ = JerkMin;
      s_init_ = sk_;
      v_init_ = vk_;
      a_init_ = ak_;
      t_init_ = tk_;
    }
    double tt = t - t_init_;
    // (3.30c)
    jk = Jmin;
    ak = a_init_ - Jmax * tt;
    vk = v_init_ + a_init_ * tt - Jmax * __square(tt) / 2;
    sk = s_init_ + v_init_ * tt + a_init_ * __square(tt) / 2.0 -
         Jmax * __cube(tt) / 6.0;
  }
  if (tmp > Vmax && ak_ <= Amin)  // Accelerate constant
  {
    if (phase_ != JerkZero)
    {
      phase_ = JerkZero;
      s_init_ = sk_;
      v_init_ = vk_;
      a_init_ = ak_;
      t_init_ = tk_;
    }
    double tt = t - t_init_;
    // (3.30b)
    jk = 0.0;
    ak = Amin;
    vk = v_init_ + Amin * tt;
    sk = s_init_ + v_init_ * tt + Amin * __square(tt) / 2.0;
  }
  if (tmp <= Vmax && ak_ < 0.0)  // Accelerate increase
  {
    if (phase_ != JerkMax)
    {
      phase_ = JerkMax;
      s_init_ = sk_;
      v_init_ = vk_;
      a_init_ = ak_;
      t_init_ = tk_;
    }
    double tt = t - t_init_;
    // (3.30a)
    jk = Jmax;
    ak = a_init_ + Jmax * tt;
    vk = v_init_ + a_init_ * tt + Jmax * __square(tt) / 2.0;
    sk = s_init_ + v_init_ * tt + a_init_ * __square(tt) / 2.0 +
         Jmax * __cube(tt) / 6.0;
  }
  if (tmp <= Vmax && ak_ >= 0.0)  // Constant velocity
  {
    if (phase_ != NoJerk)
    {
      phase_ = NoJerk;
      s_init_ = sk_;
      v_init_ = vk_;
      a_init_ = ak_;
      t_init_ = tk_;
    }
    double tt = t - t_init_;
    jk = 0.0;
    ak = 0.0;
    vk = v_init_;
    sk = s_init_ + v_init_ * tt;
  }
}

Eigen::Vector4d ScurvePlannerOnLine::getWaypoint(double t)
{
  if (!triggered_)
  {
    triggered_ = true;
    sk_ = condition_.q0;
    vk_ = condition_.v0;
    ak_ = condition_.a0;
    jk_ = 0.0;
    tk_ = 0.0;
  }
  // printf("Debug t %f, q0 %f, v0 %f, q1 %f, v1 %f.\n", t, condition_.q0, condition_.v0, condition_.q1, condition_.v1);

  double jk = 0.0;
  double ak, vk, sk;
  if (mode_ == NotYetStarted)
  {
    double jk_d, hk_d, Tj2a_d, Tj2b_d, Td_d;
    double jk_a, hk_a, Tj2a_a, Tj2b_a, Td_a;

    computeStopPhaseDccProfile(jk_d, hk_d, Tj2a_d, Tj2b_d, Td_d);
    computeStopPhaseAccProfile(jk_a, hk_a, Tj2a_a, Tj2b_a, Td_a);

    hk_d = isnan(hk_d) ? 0.0 : hk_d;
    hk_a = isnan(hk_a) ? 0.0 : hk_a;
    double hk = std::fmax(hk_d, hk_a);
    if (hk >= condition_.q1 - sk_)
    {
      // printf("Debug t %f, hk_d %f, hk_a %f, hk %f, s-sk %f.\n", t, hk_d, hk_a,
            //  hk, condition_.q1 - sk_);
      if (hk == hk_d)
      {
        mode_ = Deceleration;
        profile_.Tj2a = Tj2a_d;
        profile_.Tj2b = Tj2b_d;
        profile_.Td = Td_d;
        profile_.Th = tk_;
        // printf("Deceleration stop.\n");
      }
      else
      {
        mode_ = Acceleration;
        profile_.Tj2a = Tj2a_a;
        profile_.Tj2b = Tj2b_a;
        profile_.Td = Td_a;
        profile_.Th = tk_;
        // printf("Acceleration stop.\n");
      }
    }
    else
    {
      if (condition_.v0 <= condition_.v_max)
      {
        computeFirstPhaseAcc(t, jk, ak, vk, sk);
        // printf("First Phase Aeceleration.\n");
      }
      else
      {
        computeFirstPhaseDcc(t, jk, ak, vk, sk);
        // printf("First Phase Deceleration.\n");
      }
    }
  }

  if (mode_ == Deceleration)
  {
    computeStopPhaseDcc(t, jk, ak, vk, sk);
  }
  else if (mode_ == Acceleration)
  {
    computeStopPhaseAcc(t, jk, ak, vk, sk);
  }
  // else
  // {
  //   computeProfile(jk, t, ak, vk, sk);
  // }

  Eigen::Vector4d point;
  point[POSITION_ID] = sk;
  point[SPEED_ID] = vk;
  point[ACCELERATION_ID] = ak;
  point[JERK_ID] = jk;

  sk_ = sk;
  vk_ = vk;
  ak_ = ak;
  jk_ = jk;
  tk_ = t;

  // printf("Debug t %f, sk %f, vk %f, ak %f, jk %f.\n", t, sk, vk, ak, jk);

  pointSignTransform(point);

  return point;
}

void ScurvePlannerOnLine::reset()
{
  sk_ = 0.0;
  vk_ = 0.0;
  ak_ = 0.0;
  jk_ = 0.0;
  tk_ = 0.0;

  profile_ = ScurveProfile();
  condition_ = ScurveCondition();
  triggered_ = false;
  mode_ = NotYetStarted;
}

}  // namespace trajectory_processing
