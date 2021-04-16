# arm_motion_controller_pkg

This package is to drive ur5e with full arm(s) motion.

**Necessary packages:**

* awindamonitor: https://github.com/Raffa87/xsense-awinda
* my_human_pkg: https://github.com/frdedynamics/my_human_pkg



## 2 arm Rviz

**IMU placement:**

*Do I really need chest IMU? can't I find a way to calibrate/originate shoulder IMU w.r.t each other?*

![Without chest](/home/gizem/catkin_ws/src/arm_motion_controller_pkg/doc/imu_replacement.png)



**Commands:**

```
roscore
rosrun awindamonitor awindamonitor
roslaunch arm_motion_controller_pkg human.launch

# For 3Dof shoulder 1 dof elbow
rosrun my_human_pkg arm_IMU_control_r.py
rosrun my_human_pkg arm_IMU_control_l.py

# For 3 dof shoulder 2 dof elbow
rosrun arm_motion_controller_pkg imu_subscriber_node.py

## Pose w.r.t chest origin (middle point of shoulders)
rosrun arm_motion_controller_pkg chest_to_wrist_tf.py

```


UR5e \base_link \tool0 TF at initial pose:
- Translation: [-0.136, 0.490, 0.687]
- Rotation: in Quaternion [-0.697, 0.005, 0.012, 0.717]
            in RPY (radian) [-1.542, 0.024, 0.010]
            in RPY (degree) [-88.374, 1.403, 0.549]

