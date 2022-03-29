Follow the below commands to programmatically fly the drone in a square

Run this command to create the runway and the world in the gazebo
```
roslaunch iq_sim runway.launch
```


In a new terminal, run this command to start the Arducoptor Simulator or SITL 
```
./startsitl.sh
```

Again in a New terminal, run this command MAVROS which converts the mavlink msgs from the copter to ROS commands.
```
roslaunch iq_sim apm.launch
```

Run the below command to make the Arducoptor run the square CPP file created
```
rosrun iq_gnc square
```
