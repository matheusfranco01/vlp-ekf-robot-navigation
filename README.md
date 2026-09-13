cd /home/matheus/vlp-ekf-p3dx

cat > README.md <<'EOF'
# VLP-EKF Navigation for Pioneer 3-DX

ROS Noetic project for indoor localization and navigation of a Pioneer 3-DX
mobile robot using Visible Light Positioning (VLP), artificial neural network
pose estimation, wheel odometry, Extended Kalman Filter (EKF) sensor fusion,
and Pure Pursuit path tracking.

## System Overview

The project integrates:

- Visible Light Positioning simulation using Radiance.
- ANN-based pose estimation from visible-light measurements.
- Pioneer 3-DX wheel odometry.
- EKF sensor fusion using `robot_localization`.
- Pure Pursuit path tracking.
- Gazebo environments with custom luminaire models.

The main localization architecture is:

```text
Visible-light measurements
          |
          v
         ANN
          |
      /ann_pose
          |
          +------------+
                       |
Wheel odometry --------+----> EKF ----> /odometry/filtered
