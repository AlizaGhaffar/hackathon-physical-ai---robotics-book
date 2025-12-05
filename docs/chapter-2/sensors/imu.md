# IMU Sensors

IMU (Inertial Measurement Unit) measures acceleration and rotation. Used for balance and orientation.

## Basic IMU Setup

```xml
<sensor name="imu" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <remapping>~/out:=imu</remapping>
    </ros>
  </plugin>
</sensor>
```

**Published Data**: Acceleration (m/s²), Angular velocity (rad/s), Orientation (quaternion)

**View Data**: `ros2 topic echo /imu`
