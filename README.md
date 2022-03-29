# project-drone

## Aim 
- Implement basic drone movement and control
- Object Tracking

## Basic drone workflow
For start we are using SITL (softare in 

1. ArduPilot
   ArduPilot is an open source autopilot system supporting many vehicle types:
   multi-copters,traditional helicopters, fixed wing aircraft, boats, submarines, rovers and more. 

2. MAVProxy 
   MAVProxy is a fully-functioning command-line, console based GCS.
   Can be extended to use GUI based ground planners like  QGroundControl.
   
3. QGroundControl
   QGroundControl is an ground planner app to configure and fly a PX4 or Ardupilot based autopilots. 
   We used a default appimage and connect our drone (SITL) to it.
   
6. Gazebo
   Gazebo is the external robotics/drone simulator.
   We connect with ROS to achieve navigation in the Gazebo world.

4. SITL
   SITL is (software in the loop) simulator allows you to run Plane, Copter or Rover without any hardware. 
   We connect through SITL through MAVproxy and 
   
5. Ros
   ROS is open source software development kit for robotics applications.
   
   
7. catkin

8. pixhawk



