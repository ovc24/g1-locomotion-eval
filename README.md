# g1-locomotion-eval

Small toolkit to evaluate Unitree G1 behaviour from ROS2 bags: body inclination (IMU),
approximate mechanical power and per-joint tracking error between `/lowcmd` and `/lowstate`.
Developed and tested in simulation only (unitree_mujoco, ROS2 Humble in Docker).

## Setup (summary)

- Ubuntu 22.04 / ROS2 Humble (Docker image `osrf/ros:humble-desktop`, `--network host`)
- `unitree_ros2` (messages `unitree_hg`) and `unitree_mujoco` (C++ simulator)
- In `unitree_mujoco/simulate/config.yaml`: `robot: "g1"`, `robot_scene: "scene_29dof.xml"`,
  `idl_type: 1`, `domain_id: 1`, `interface: "lo"`, `enable_elastic_band: 1`
- Run the simulator with a clean environment (`run_sim.sh`): the ROS2 CycloneDDS library
  clashes with the Unitree one and the simulator aborts with a DDS assertion otherwise.

## Usage

    export ROS_DOMAIN_ID=1
    ros2 bag record -o bag_ankle /lowstate /lowcmd      # while an example publishes commands
    python3 analizar.py bag_ankle                       # inclination, power, falls
    python3 seguimiento.py bag_ankle 0 6 2.0            # tracking error, settling time, plot

## Metrics

| Metric | Source |
|---|---|
| Max roll / pitch, samples above 45 deg | `/lowstate` IMU |
| Mean mechanical power (sum of abs(tau_est * dq)) | `/lowstate` motors |
| RMS and max tracking error per joint | `/lowcmd` vs `/lowstate` |
| Settling time (error stays under a threshold) | `/lowcmd` vs `/lowstate` |

## Example result (g1_ankle_swing_example, simulation, first 6 s, includes start-up transient)

About 490 commands per second. RMS tracking error between 1 and 3 deg: elbows ~3.1,
ankle pitch ~2.75, shoulder pitch ~2.1, hip yaw ~1.8 (peak 8.4 deg).

## Limitations

- Simulation only. Results are not valid for the physical robot.
- Ankle AB mode is meant for the real robot and is not meaningful in this MuJoCo model.
- `tau_est` is an estimate, so power is only a rough indicator.
- Never send low-level commands to the real robot without supervision.

## Next steps

Evaluate walking with the locomotion controller, add velocity-setpoint tracking, and
compare simulated and real data.
