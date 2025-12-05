# LiDAR Sensors

LiDAR (Light Detection and Ranging) uses laser beams to measure distances. Essential for autonomous navigation.

## Basic 2D LiDAR Setup

```xml
<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <min_angle>-3.14</min_angle>
        <max_angle>3.14</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.2</min>
      <max>10.0</max>
    </range>
  </ray>
  <plugin name="laser" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <remapping>~/out:=scan</remapping>
    </ros>
  </plugin>
</sensor>
```

**View in RViz**: Add LaserScan display, topic `/scan`
