# Module 4: Controller tuning (Gazebo Classic, effort interface)

## Method
- Controller: `joint_trajectory_controller`, effort command interface, per-joint PID gains (`controllers.yaml`).
- Test: near-instant step (0.02 s trajectory) on one joint at a time, from the home pose; response recorded from `/joint_states` (`tuning/step_test.py`).
- Metrics: rise time (10-90%), overshoot, settling time (2% band), steady-state error. Negative steady-state error means the joint settled slightly past the target.
- Step sizes: joint_1 and joint_2 0.3 rad, joint4 0.5 rad, joint3_z 0.03 m.

## Evidence that the simulation is physics-driven
Changing only the P gain of joint3_z from 200 to 50 changed the response clearly (rise 0.163 s to 0.539 s, settling 0.303 s to 4.32 s). A response that moved with the gain indicates the effort command is acted on by the physics engine rather than the joint being placed at the target. Caveat: the integral gain was unchanged between these two runs.

## Tuning runs
| Run | Rise (s) | Overshoot (%) | Settling (s) | SSE |
|---|---|---|---|---|
| joint3_z p=50 | 0.539 | 5.3 | 4.319 | -2.96e-4 m |
| joint3_z p=200 | 0.163 | 0.6 | 0.303 | -1.02e-4 m |
| joint_1 d=0.1 (initial) | 0.266 | 36.7 | 2.766 | -1.14e-4 rad |
| joint_1 d=0.3 | 0.445 | 3.1 | 0.963 | -1.18e-6 rad |
| joint_1 d=0.6 | 1.093 | 0.0 | 1.983 | 4.95e-6 rad |
| joint_2 d=0.05 (initial) | 0.265 | 9.7 | 1.524 | -2.91e-5 rad |
| joint_2 d=0.15 | 0.530 | 0.0 | 0.945 | 6.51e-7 rad |
| joint_2 d=0.3 | 1.091 | 0.0 | 2.040 | 1.20e-5 rad |
| joint4 (initial, kept) | 0.106 | 0.0 | 0.189 | -1.83e-10 rad |

Figure: `figures/tuning_comparison.png`.

## Final gains (verified after a clean restart from `controllers.yaml`)
| Joint | P | I | D | Rise (s) | Overshoot (%) | Settling (s) |
|---|---|---|---|---|---|---|
| joint_1 | 1.0 | 0 | 0.3 | 0.356 | 3.1 | 0.939 |
| joint_2 | 0.5 | 0 | 0.15 | 0.542 | 0.0 | 0.922 |
| joint3_z | 200 | 20 | 20 | 0.163 | 0.6 | 0.302 |
| joint4 | 0.01 | 0 | 0.0005 | 0.106 | 0.0 | 0.192 |

Per-joint plots: `figures/tuning_final_*.png`.

## Limitations
- Manual search over a small set of candidate gains (2-3 values per joint, one parameter at a time); the chosen values are the best of those tested, not a proven optimum.
- joint3_z I and D gains and the joint_1/joint_2 P gains were not varied; joint4 was left at its initial gains because it already met the criteria.
- One step size per joint, no payload, simulation only (Gazebo Classic, 1 kHz update rate).
- Each plot shows a short initial fast rise followed by a slower approach. Its cause was not investigated.