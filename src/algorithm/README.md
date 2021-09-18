# Trajectory Planning

## Single Axis Motion Planning

When driving a motor to an absolute or additve position, the motors always try to accelerate its velocity to its max value and decelerate before it reaches the target position. If the acceleration during this process is not continous or linear piece-wise, some infinite jerk spikes may happen. For this reason, this trajectory may generate efforts and stresses on the mechanical system that may result detrimental or generate undesired
vibrational effects. Therefore, the so called S-Curve trajectory planning is necessary.

![](../../image/scurve_profile.png)

As above figure shows, a complete s-curve process usually contains seven segments with inputs (q0, q1, v0, v1) and constraints (Vmax, Amax, Jmax):

- Tj1 : time-interval in which the jerk is constant (jmax or jmin) during the acceleration phase;
- Tj2 : time-interval in which the jerk is constant (jmax or jmin) during the deceleration phase;
- Ta : acceleration period;
- Tv : constant velocity period;
- Td : deceleration period;
- T : total duration of the trajectory (= Ta + Tv + Td).

The s-curve planning module in this project is based on "Trajectory Planning for Automatic Machines and Robots-Springer (2008)" by Luigi Biagiotti, Claudio Melchiorri.

Some running examples in [test/s_curve_test.cpp](../../test/s_curve_test.cpp):

- Example 3.9

  * Input: q0 = 0, q1 = 10, v0 = 1, v1 = 0
  * Constraints: Vmax = 5, Amax = 10, Jmax = 30

![](../../image/Example_3_9.png)

- Example 3.10

  * Input: q0 = 0, q1 = 10, v0 = 1, v1 = 0
  * Constraints: Vmax = 10, Amax = 10, Jmax = 30

![](../../image/Example_3_10.png)

- Example 3.11

  * Input: q0 = 0, q1 = 10, v0 = 7, v1 = 0
  * Constraints: Vmax = 10, Amax = 10, Jmax = 30

![](../../image/Example_3_11.png)

- Example 3.12

  * Input: q0 = 0, q1 = 10, v0 = 7.5, v1 = 0
  * Constraints: Vmax = 10, Amax = 10, Jmax = 30

![](../../image/Example_3_12.png)

- Example 3.13

  * Input: q0 = 0, q1 = 10, v0 = 0, v1 = 0
  * Constraints: Vmax = 10, Amax = 20, Jmax = 30

![](../../image/Example_3_13.png)
