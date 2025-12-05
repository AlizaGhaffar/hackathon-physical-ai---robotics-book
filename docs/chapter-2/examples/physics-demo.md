# Example: Physics Demo

Demonstrate gravity, friction, and collisions.

## Bouncing Ball

```xml
<model name="ball">
  <link name="link">
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <surface>
        <bounce>
          <restitution_coefficient>0.9</restitution_coefficient>
        </bounce>
      </surface>
    </collision>
  </link>
</model>
```

## Sliding Box on Slope

```xml
<model name="slope">
  <static>true</static>
  <link name="link">
    <collision>
      <geometry>
        <box size="2 2 0.1"/>
      </geometry>
    </collision>
    <pose>0 0 0 0 0.5 0</pose>  <!-- 30-degree tilt -->
  </link>
</model>
```

## Test: Drop objects and observe physics!
