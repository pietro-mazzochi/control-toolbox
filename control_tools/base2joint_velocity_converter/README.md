# base2joint_velocity_converter

## Send commands

```
ros2 topic pub /base_vel_cmd geometry_msgs/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -1
```

- linear.x: Linear velocity
- angular.z: Angular velocity