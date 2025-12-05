# Example: Simple Robot

A minimal mobile robot with a box body and two wheels.

## Robot Model (simple_robot.urdf)

```xml
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.3 0.2"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.4 0.3 0.2"/>
      </geometry>
    </collision>
  </link>

  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.15 0" rpy="-1.57 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
```

## Launch in Gazebo

```bash
gazebo simple_robot.urdf
```
