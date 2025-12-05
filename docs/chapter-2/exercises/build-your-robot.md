# Exercise: Build Your Robot

Create a custom robot with sensors in Gazebo.

## Task

Build a mobile robot with:
1. Box chassis (0.5m x 0.3m x 0.2m)
2. Two wheels (radius 0.1m)
3. One camera sensor
4. One LiDAR sensor

## Steps

1. **Create URDF file**: Define links and joints
2. **Add sensors**: Camera on front, LiDAR on top
3. **Set physics**: Add friction to wheels
4. **Launch**: Test in Gazebo
5. **Verify**: Check topics with `ros2 topic list`

## Success Criteria

- [ ] Robot spawns in Gazebo
- [ ] Camera publishes to `/camera/image_raw`
- [ ] LiDAR publishes to `/scan`
- [ ] Robot doesn't fall through ground
- [ ] Wheels have proper friction

## Bonus Challenges

- Add an IMU sensor
- Change wheel friction and test on slopes
- Add collision detection with bumper sensor
