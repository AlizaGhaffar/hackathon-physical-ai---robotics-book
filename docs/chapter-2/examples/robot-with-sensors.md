# Example: Robot with Sensors

A robot with camera and LiDAR sensors.

## Key Addition: Sensor Links

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
  </visual>
  <sensor name="camera" type="camera">
    <camera>
      <horizontal_fov>1.396</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so"/>
  </sensor>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.2 0 0.1"/>
</joint>
```

## Test Sensors

```bash
ros2 topic list
ros2 topic echo /camera/image_raw
```
