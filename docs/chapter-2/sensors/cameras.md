# Camera Sensors

Cameras provide visual input for object detection and navigation.

## Basic Camera Setup

```xml
<sensor name="camera" type="camera">
  <update_rate>30</update_rate>
  <camera>
    <horizontal_fov>1.396</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <ros>
      <remapping>~/image_raw:=camera/image_raw</remapping>
    </ros>
  </plugin>
</sensor>
```

**View Images**: `ros2 run rqt_image_view rqt_image_view`
