# Gantry crane control
LQR and LQG control for gantry crane with 2 masses at different height

### Author
- Hrushikesh Budhale
## Table of Contents
   * [What is this?](#what-is-this)
   * [Requirements](#requirements)
   * [Open loop system](#open-loop-system)
   * [LQR control](#linear-quadratic-regulator)
   * [LQR control - Reference Tracking](#lqr-reference-tracking)
   * [Luenberger observer](#luenberger-observer)
   * [LQG control - Reference Tracking](#lqg-reference-tracking)
   * [Kalman filter](#kalman-filter)

## What is this?

This is a Matlab code collection of control experiments.

System to be controlled:
- A crane that moves along an one-dimensional track.
- It behaves as a frictionless cart with mass M actuated by an external force F that constitutes the input of the system.
- There are two loads suspended from cables attached to the crane.
- The loads have mass m1 and m2, and the lengths of the cables are l1 and l2 respectively


## Requirements

For running this code:
- [Matlab](https://www.mathworks.com/)

## Open Loop System
This is a open loop system with M=1000Kg, m1=100, m2=100, L1 = 20m, L2=10m.

Since the original system is non liner, At first it is linearized around the equilibrium point.

System animation:

<img src="https://github.com/HrushikeshBudhale/gantry_crane_control/blob/main/doc/open_loop.gif?raw=true" width="640" alt="Open loop pic">


This system has 6 state variables, namely: 
   1. cart position (x)
   2. cart velocity (xd)
   3. mass 1 angle (t1)
   4. mass 1 angular velocity (t1d)
   5. mass 2 angle (t2)
   6. mass 2 angular velocity (t2d),

Graph of x, t1 and t2 w.r.t. time:

<img src="https://github.com/HrushikeshBudhale/gantry_crane_control/blob/main/doc/open_loop_system.jpg?raw=true" width="640" alt="Open loop pic">


## Linear Quadratic Regulator
LQR (Linear Quadratic Regulator) controller provides the optimal feedback gain considering the relative penalization between the state error and input effort.
For controlling the system, following initial and desired state were used.
State variables | x | xd | t1 (deg) | t1d | t2 (deg) | t2d
--- | --- | --- | --- | --- | --- | ---
Initial State | 0 | 0 | 10 | 0 | -10 | 0
Desired State | 0 | 0 |  0 | 0 |  0 | 0

While Q and R were chosen experimentally.

Response after tuning the gains:

<img src="https://github.com/HrushikeshBudhale/gantry_crane_control/blob/main/doc/closed_loop_lqr.gif?raw=true" width="640" alt="Open loop pic">

## LQR Reference Tracking

After setting non zero position in desired state
State variables | x | xd | t1 (deg) | t1d | t2 (deg) | t2d
--- | --- | --- | --- | --- | --- | ---
Initial State | 0 | 0 | 10 | 0 | -10 | 0
Desired State | 10 | 0 |  0 | 0 |  0 | 0

Response after tuning the gains:

<img src="https://github.com/HrushikeshBudhale/gantry_crane_control/blob/main/doc/lqr_reference_tracking.gif?raw=true" width="640" alt="Open loop pic">


Graph of x, t1, t2 and input force w.r.t. time:   
<img src="https://github.com/HrushikeshBudhale/gantry_crane_control/blob/main/doc/closed_loop_lqr.jpg?raw=true" width="640" alt="Open loop pic">

## Luenberger Observer
In the LQR control it is assumed that complete state of the system (6 variables) are known at every time step. And the entire state is fed back to the system after multiplying with LQR gain.

But, in real systems, we can get only few states of the systems as output, hence it is required to estimate the full state of the system using another model of the same system. The state estimate of the observer system can have different initial estimate, but it should attain the actual system state. The error difference between the observer and real system is reduced by setting the Luenberger observer gain experimentally.

Below animation shows the convergence of the observer when 6 system poles are placed at,
-2 -3 -4 -5 -6 and -7.

State variables | x | xd | t1 (deg) | t1d | t2 (deg) | t2d
--- | --- | --- | --- | --- | --- | ---
Initial State | 0 | 0 | 10 | 0 | -10 | 0
Initial Estimate | 5 | 0 |  -8 | 0 |  5 | 0

In this system only 3 states (x, t1, t2) out of 6 states are observable.

Slow motion visualization of Luenberger state observer approaching the original system state

<img src="https://github.com/HrushikeshBudhale/gantry_crane_control/blob/main/doc/Luenberger_observer_slow_motion.gif?raw=true" width="640" alt="Open loop pic">


Graph of observable states x, t1, t2 w.r.t. time:

<img src="https://github.com/HrushikeshBudhale/gantry_crane_control/blob/main/doc/observer_slow_mo1.jpg?raw=true" width="640" alt="Open loop pic">

From below graph It can be seen that the observer can precisely estimate the unobservable states.

Graph of unobservable states xd, t1d, t2d w.r.t. time:

<img src="https://github.com/HrushikeshBudhale/gantry_crane_control/blob/main/doc/observer_slow_mo2.jpg?raw=true" width="640" alt="Open loop pic">


## LQG Reference Tracking
The complete state estimated by the observer can then be used as a feed back with LQR gain to control the desired position of the crane.

State variables | x | xd | t1 (deg) | t1d | t2 (deg) | t2d
--- | --- | --- | --- | --- | --- | ---
Initial State | 0 | 0 | 10 | 0 | -10 | 0
Initial Estimate | 5 | 0 |  -8 | 0 |  5 | 0
Desired State | 10 | 0 |  0 | 0 |  0 | 0

System animation:

<img src="https://github.com/HrushikeshBudhale/gantry_crane_control/blob/main/doc/lqg_reference_tracking.gif?raw=true" width="640" alt="Open loop pic">


Graph of observable states x, t1, t2 w.r.t. time:

<img src="https://github.com/HrushikeshBudhale/gantry_crane_control/blob/main/doc/obsv_ref_track1.jpg?raw=true" width="640" alt="Open loop pic">


Graph of unobservable states xd, t1d, t2d w.r.t. time:

<img src="https://github.com/HrushikeshBudhale/gantry_crane_control/blob/main/doc/obsv_ref_track2.jpg?raw=true" width="640" alt="Open loop pic">

## Kalman Filter
In the observer example, exact output of the 3 observable states of the system were visible. But in reality, systems get affected by the disturbances and the sensors are used to measure these system states. And sensors have noise in their readings. Sensors with high noise make it difficult to estimate the actual state of the system.

If this sensor noise can be approximated as zero mean gaussian noise with known variance, then Kalman filter can be designed to trade-off between the sensor measurement and prediction by the estimator.

In this experiment the filter was designed to consider lower variance in the disturbance (process noise) and higher variance in the measurement noise.

It can be seen that in few seconds after starting the estimated state converges and closely with the actual state while not getting affected by the measurement noise present in all the observable states.

State variables | x | xd | t1 (deg) | t1d | t2 (deg) | t2d
--- | --- | --- | --- | --- | --- | ---
Initial State | 1 | 0 | -10 | 0 | 10 | 0
Initial Estimate | 1.2 | 0 |  -6 | 0 |  7 | 0

System animation:

<img src="https://github.com/HrushikeshBudhale/gantry_crane_control/blob/main/doc/kalman_filter.gif?raw=true" width="640" alt="Open loop pic">


Graph of observable states x, t1, t2 w.r.t. time:

<img src="https://github.com/HrushikeshBudhale/gantry_crane_control/blob/main/doc/kalman_filter1.jpg?raw=true" width="640" alt="Open loop pic">


Graph of unobservable states xd, t1d, t2d w.r.t. time:

<img src="https://github.com/HrushikeshBudhale/gantry_crane_control/blob/main/doc/kalman_filter2.jpg?raw=true" width="640" alt="Open loop pic">

