# Sensor Plugins

Gazebo plugins connect sensors to ROS 2 topics.

## Common Sensor Plugins

| Sensor | Plugin | Output Topic |
|--------|--------|--------------|
| Camera | `libgazebo_ros_camera.so` | `/camera/image_raw` |
| LiDAR | `libgazebo_ros_ray_sensor.so` | `/scan` |
| IMU | `libgazebo_ros_imu_sensor.so` | `/imu` |
| Depth Camera | `libgazebo_ros_camera.so` | `/depth/image_raw` |

## Plugin Template

```xml
<plugin name="plugin_name" filename="libgazebo_ros_SENSOR.so">
  <ros>
    <namespace>/robot</namespace>
    <remapping>~/out:=topic_name</remapping>
  </ros>
  <frame_name>sensor_link</frame_name>
</plugin>
```
