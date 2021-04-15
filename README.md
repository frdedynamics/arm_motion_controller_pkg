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
rosrun my_human_pkg arm_IMU_control_r.py
rosrun my_human_pkg arm_IMU_control_l.py

## Pose w.r.t chest origin (middle point of shoulders)
rosrun arm_motion_controller_pkg chest_to_wrist_tf.py

```

