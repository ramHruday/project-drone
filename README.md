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


![Shape7](RackMultipart20230224-1-nw31db_html_88ca6de25a182f15.gif)

![](RackMultipart20230224-1-nw31db_html_fb181643826149a7.jpg)

# **Internet of Drones**

## Software set-up of Internet of Drones

To emulate a drone, we use software listed below which help in

## **ArduPilot**

ArduPilot is an open-source autopilot system supporting many vehicle types: multi-copters, traditional helicopters, fixed wing aircraft, boats, submarines, rovers and more.

1. **Installing Ardupilot**

cd ~

sudo apt install git

git clone https://github.com/ArduPilot/ardupilot.git

cd ardupilot

git checkout Copter-3.6

git submodule update --init –recursive

1. **Installing dependencies**

sudo apt install python-matplotlib python-serial python-wxgtk3.0 python-wxtools python-lxml python-scipy python-opencv ccache gawk python-pip python-pexpect

## MAVproxy

MAVproxy is a fully functioning command-line, console-based GCS. Can be extended to use GUI based ground planners like QGroundControl. It's a minimalist, portable and extendable GCS for any UAV supporting the MAVLink protocol (such as one using ArduPilot).

1. **pip to install MAVproxy**

sudo pip install future pymavlink MAVProxy

1. Open **~/.bashrc**

gedit ~/.bashrc

1. **Add these lines to end of****   ****~/.bashrc**

export PATH=$PATH:$HOME/ardupilot/Tools/autotest

export PATH=/usr/lib/ccache:$PATH

1. **Reload of****   ****~/.bashrc**

. ~/.bashrc

## **SITL**

SITL is (software in the loop) simulator allows you to run Plane, Copter or Rover without any hardware. We connect through SITL through MAVproxy and run our commands.

1. **Run SITL**

cd ~/ardupilot/ArduCopter

sim\_vehicle.py -w

![](RackMultipart20230224-1-nw31db_html_79448445874694dc.png)

## **QGroundControl**

QGroundControl is a ground planner app to configure and fly a PX4 or Ardupilot based autopilots. We used a default app-image and connect our drone (SITL) to it.

1. **Install QGroundControl for Ubuntu Linux 16.04 LTS or later:**

sudo usermod -a -G dialout $USER

sudo apt-get remove modemmanager

1. **Download QGroundControl.AppImage**

wget [https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage](https://s3-us-west-2.amazonaws.com/qgroundcontrol/latest/QGroundControl.AppImage)

1. **Change permissions and run**

chmod +x ./QGroundControl.AppImage

./QGroundControl.AppImage

1. **Connect QGroundControl to SITL**

cd ~/ardupilot/ArduCopter/

sim\_vehicle.py

1. **If QGroundControl doesn't open up on double click, install below packages to resolve this**.

sudo apt install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl -y

sudo apt install libsdl2-dev

sudo apt-get install '^libxcb.\*-dev' libx11-xcb-dev libglu1-mesa-dev libxrender-dev libxi-dev libxkbcommon-dev libxkbcommon-x11-dev

![](RackMultipart20230224-1-nw31db_html_32d5bdf5dbfb6b6e.png)

![](RackMultipart20230224-1-nw31db_html_e82d14d17d32f42.png)

## **Gazebo**

Gazebo is the external robotics/drone simulator. We connect with ROS to achieve navigation in the Gazebo world. ROS is open-source software development kit for robotics applications.

1. **Install Gazebo**

sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" \> /etc/apt/sources.list.d/gazebo-stable.list'

1. **set up keys**

wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add –

1. **Reload software list:**

sudo apt update

1. **For Ubuntu [18.04]**

sudo apt install gazebo9 libgazebo9-dev

1. **For Ubuntu [20.04]**

sudo apt-get install gazebo11 libgazebo11-dev

1. **Install Gazebo plugin for APM (ArduPilot Master) :**

cd ~

git clone https://github.com/khancyr/ardupilot\_gazebo.git

cd ardupilot\_gazebo

1. **Only for Ubuntu 18.0.4**

git checkout dev

1. **Build and Install plugin**

mkdir build

cd build

cmake ..

make -j4

sudo make install

echo 'source /usr/share/gazebo/setup.sh' \>\> ~/.bashrc

1. **Now set up models for Gazebo**

echo'export GAZEBO\_MODEL\_PATH=~/ardupilot\_gazebo/models' \>\> ~/.bashrc

~/.bashrc

1. **In one Terminal (Terminal 1), run Gazebo:**

gazebo --verbose ~/ardupilot\_gazebo/worlds/iris\_arducopter\_runway.world

1. **In another Terminal (Terminal 2), run SITL:**

cd ~/ardupilot/ArduCopter/

sim\_vehicle.py -v ArduCopter -f gazebo-iris --console

![](RackMultipart20230224-1-nw31db_html_1c09ca13d566880e.png)

1. **Install ROS from this** [link](http://wiki.ros.org/melodic/Installation/Ubuntu)

**Do Desktop-full Install and follow until Step 1.7 at the end of the page**

1. **Set Up Catkin workspace**

sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools

1. **initialize the catkin workspace:**

mkdir -p ~/catkin\_ws/src

cd ~/catkin\_ws

catkin init

1. **Dependencies installation - mavros and mavlink:**

cd ~/catkin\_ws

wstool init ~/catkin\_ws/src

rosinstall\_generator --upstream mavros | tee /tmp/mavros.rosinstall

rosinstall\_generator mavlink | tee -a /tmp/mavros.rosinstall

wstool merge -t src /tmp/mavros.rosinstall

wstool update -t src

rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS\_DISTRO` -y

catkin build

1. **Edit Bashrc**

echo"source ~/catkin\_ws/devel/setup.bash" \>\> ~/.bashrc

1. **Reload and update**

source ~/.bashrc

1. **install geographiclib dependency**

sudo ~/catkin\_ws/src/mavros/mavros/scripts/install\_geographiclib\_datasets.sh

1. **Clone IQ Simulation ROS package**

cd ~/catkin\_ws/src

git clone [https://github.com/Intelligent-Quads/iq\_sim.git](https://github.com/Intelligent-Quads/iq_sim.git)

1. **run the following to tell gazebo where to look for the iq models**

echo"GAZEBO\_MODEL\_PATH=${GAZEBO\_MODEL\_PATH}:$HOME/catkin\_ws/src/iq\_sim/models"

\>\> ~/.bashrc

1. **Build**

cd ~/catkin\_ws

catkin build

1. **Reload and Update**

source ~/.bashrc

1. **Install ROS plugins for Gazebo**

sudo apt install ros-melodic-gazebo-ros ros-melodic-gazebo-plugins

1. **Launch Gazebo World**

roslaunch iq\_sim runway.launch

1. **Copy the script to run SITL**

cp ~/catkin\_ws/src/iq\_sim/scripts/startsitl.sh ~

1. **In new terminal , start SITL**

~/startsitl.sh

1. **In the same terminal, fly the Drone using below commands**

mode guided

arm throttle

takeoff 15

MAVROS is a middle man which translates the MAVlink messages into ROS messages, which are easy to use and common between different robot systems. To start mavros run

1. **Now launch MAVROS**

roslaunch iq\_sim apm.launch

![](RackMultipart20230224-1-nw31db_html_dcba64f87349920a.png)

## Drone Control using C++

We will now attempt to direct the drone into a circular path via C++ program.

This program allows you to send your drone to waypoints. It uses GNC\_APIs created by Intelligent Quads which has a bunch of high-level functions that handle the various flight operations including, take off, land, waypoint nav and all the reference frames associated with the navigation. The documentation for these GNC functions is available [here](https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/GNC_functions_documentation.md)

1. Git clone a custom ROS package with GNC API.

git clone [https://github.com/Intelligent-Quads/iq\_gnc.git](https://github.com/Intelligent-Quads/iq_gnc.git)

1. In CMakeLists.txt add the following which connects your custom C++ file

add\_executable(square src/circle.cpp)

target\_link\_libraries(circle ${catkin\_LIBRARIES})

1. Use this [link](https://github.com/ramHruday/project-drone/blob/main/circle.cpp) for Circular path and create circle.cpp

Circle cpp uses the following path algorithm to map a circle

    int increment =9;

    int radius =5;

    std::vector\<gnc\_api\_waypoint\> waypointList;

    for (size\_t i =0; i \<= increment; i++)

    {

        gnc\_api\_waypoint nextWayPoint;

        nextWayPoint.x= radius \*cos(i \*2\* M\_PI / increment);

        nextWayPoint.y= radius \*sin(i \*2\* M\_PI / increment);

        nextWayPoint.z=3;

        nextWayPoint.psi=10;

        waypointList.push\_back(nextWayPoint);

    }

Note

- gnc\_api\_waypoint is a custom interface to store coordinates (x,y,z,psi)
- where x,y,z are plane-coordinates and psi is measure of rotation around the z-axis.

_//specify control loop rate. We recommend a low frequency to not over load the_

_FCU with messages. Too many messages will cause the drone to be sluggish_

    ros::Raterate(2.0);

    int counter =0;

    while (ros::ok())

    {

        ros::spinOnce();

        rate.sleep();

        if (check\_waypoint\_reached(.3) ==1)

        {

            if (counter \<waypointList.size())

            {

                set\_destination(waypointList[counter].x, waypointList[counter].y, waypointList[counter].z, waypointList[counter].psi);

                counter++;

            }

            else

            {

                _//land after all waypoints are reached_

                land();

            }

        }

    }

The above code in circle.cpp is to constantly set communication with ROS/Ardupilot to reach a waypoint and after it reaches a waypoint move over to next waypoint.

A Vector is used to store the list of waypoints.

1. Build iq\_gnc using following commands

cd ~/catkin\_ws

catkin build

source ~/.bashrc

1. Now run below commands in new terminals respectively

roslaunch iq\_sim runway.launch

_# New Terminal_

./startsitl.sh

_# New Terminal_

roslaunch iq\_sim apm.launch

_# New Terminal_

rosrun iq\_gnc square

1. The program would only start when drone mode is set to guided.

Now set the flight mode to _ **guided** _ in the MAVproxy terminal by running

mode guided

![](RackMultipart20230224-1-nw31db_html_724ce7af0c9dc655.png)

_Figure 1 Image showing the circular path traversal in QGroundControl_

## Connecting Multiple SITL to Gazebo

1. **Create drones in runway.world by adding following lines**

\<modelname="drone1"\>

  \<pose\>10 0 0 0 0 0\</pose\>

  \<include\>

    \<uri\>model://drone2\</uri\>

  \</include\>

\</model\>

\<modelname="drone2"\>

  \<pose\>12 0 0 0 0 0\</pose\>

  \<include\>

    \<uri\>model://drone2\</uri\>

  \</include\>

\</model\>

\<modelname="drone3"\>

  \<pose\>14 0 0 0 0 0\</pose\>

  \<include\>

    \<uri\>model://drone2\</uri\>

  \</include\>

\</model\>

1. **Add the following config in** _ **ardupilot/Tools/autotest/pysim/vehicleinfo.py** _

"gazebo-drone1": {

    "waf\_target": "bin/arducopter",

    "default\_params\_filename": ["default\_params/copter.parm",

                                "default\_params/gazebo-drone1.parm"],

},

"gazebo-drone2": {

    "waf\_target": "bin/arducopter",

    "default\_params\_filename": ["default\_params/copter.parm",

                                "default\_params/gazebo-drone2.parm"],

},

"gazebo-drone3": {

    "waf\_target": "bin/arducopter",

    "default\_params\_filename": ["default\_params/copter.parm",

                                "default\_params/gazebo-drone3.parm"],

},

1. **create the following files**

- default\_params/gazebo-drone1.parm
- default\_params/gazebo-drone2.parm
- default\_params/gazebo-drone3.parm

In Each parm file add this corresponding to the drone

# Iris is X frame

FRAME\_CLASS 1

FRAME\_TYPE  1

# IRLOCK FEATURE

RC8\_OPTION 39

PLND\_ENABLED    1

PLND\_TYPE       3

# SONAR FOR IRLOCK

SIM\_SONAR\_SCALE 10

RNGFND1\_TYPE 1

RNGFND1\_SCALING 10

RNGFND1\_PIN 0

RNGFND1\_MAX\_CM 5000

SYSID\_THISMAV 1

- default\_params/gazebo-drone1.parm should contain SYSID\_THISMAV 1
- default\_params/gazebo-drone2.parm should contain SYSID\_THISMAV 2
- default\_params/gazebo-drone3.parm should contain SYSID\_THISMAV 3

**Notes from this week:**

1. **Focus on battery and speed data connection to ROS and C++ program.**
2. **Since the connection 3 drones is still a work-in-progress, should be attempting to work on multiple drones after completely integrating a single drone.**
3. **On multiple drone system, should get a clear picture on IP connections.**
4. **How does default connection of drone's work and where can we change the default config.**

## Reading Drone Battery level

Reading battery level can be done by subscribing to drone _ **mavros/battery** _ and initiate and store a global variable for the battery state of drone.

The BatteryState of drone is an object which contains the following

![](RackMultipart20230224-1-nw31db_html_4e02fb91b7f4dc71.png)

For the current scenario we make use of percentage and capacity to determine the total battery usage for a trip. The code for connecting to the endpoint is initiated as a subscriber to a gnc\_node is given below

![](RackMultipart20230224-1-nw31db_html_9d933c8127f715f5.png)

![](RackMultipart20230224-1-nw31db_html_e8767c3c58f20502.png)

The state\_b is custom call-back function which updates the global variable for the battery level whenever the subscriber emits a msg.

![](RackMultipart20230224-1-nw31db_html_ac11a8d9c35ce7f4.png)

The above screenshot is after running the drone for circular radius of r= 5metres.

From the above data, a distance of (r + 2πr) = 36.4metres was covered. Starting battery level was 100% and trip ended with 87% of battery. The altitude was 3metres.

## Reading Drone Velocity

Reading battery level can be done by subscribing _ **local\_position/velocity** _ endpoint would give us a call back to store the Twist object which had Angular and Velocity information.

The Twist is an object which contains the following

geometry\_msgs/Vector3 linear

geometry\_msgs/Vector3 angular

structgeometry\_msgs{

float64 x

float64 y

float64 z

}

The velocity was set randomly before each waypoint in the circular path. The velocity was printed out before starting out to the next waypoint.

The code for setting a velocity is given below

![](RackMultipart20230224-1-nw31db_html_42f19f1d4a1542b.png)

The velocity information during flight is logged and highlighted in the screenshot below. ![](RackMultipart20230224-1-nw31db_html_9578978d7d879e9c.png)

## Adding Models to Gazebo

Used git to get a bunch of open-source gazebo models from the Open-Source Robotics Foundation (OSRF)

1. git clone [https://github.com/osrf/gazebo\_models.git](https://github.com/osrf/gazebo_models.git)
2. echo 'export GAZEBO\_MODEL\_PATH=~/gazebo\_ws/gazebo\_models:${GAZEBO\_MODEL\_PATH}' \>\> ~/.bashrc
3. source ~/.bashrc

![](RackMultipart20230224-1-nw31db_html_9ce67e9503398577.png)

1. Create a file in directory~/catkin\_ws/src/iq\_sim/worlds/ called hills.world

\<?xml version="1.0" ?\>

\<sdfversion="1.6"\>

  \<worldname="default"\>

    \<physicstype="ode"\>

      \<ode\>

        \<solver\>

          \<type\>quick\</type\>

          \<iters\>100\</iters\>

          \<sor\>1.0\</sor\>

        \</solver\>

        \<constraints\>

          \<cfm\>0.0\</cfm\>

          \<erp\>0.9\</erp\>

          \<contact\_max\_correcting\_vel\>0.1\</contact\_max\_correcting\_vel\>

          \<contact\_surface\_layer\>0.0\</contact\_surface\_layer\>

        \</constraints\>

      \</ode\>

      \<real\_time\_update\_rate\>-1\</real\_time\_update\_rate\>

      _\<!-- \<max\_step\_size\>0.0020\</max\_step\_size\> --\>_

    \</physics\>

    \<include\>

      \<uri\>model://sun\</uri\>

    \</include\>

    \<include\>

      \<uri\>model://ground\_plane\</uri\>

    \</include\>

    \<modelname="iris"\>

      \<include\>

        \<uri\>model://iris\_with\_standoffs\_demo\</uri\>

      \</include\>

      _\<!-- add new camera --\>_

      \<linkname='camera'\>

        \<pose\>0 -0.01 0.070 .8 0 1.57\</pose\>

        \<inertial\>

          \<pose\>0 0 0 0 0 0\</pose\>

          \<mass\>0.1\</mass\>

          \<inertia\>

            \<ixx\>0.001\</ixx\>

            \<ixy\>0\</ixy\>

            \<ixz\>0\</ixz\>

            \<iyy\>0.001\</iyy\>

            \<iyz\>0\</iyz\>

            \<izz\>0.001\</izz\>

          \</inertia\>

        \</inertial\>

        \<visualname='visual'\>

          \<pose\>0 0 0 0 0 0\</pose\>

          \<geometry\>

            \<cylinder\>

              \<radius\>0.025\</radius\>

              \<length\>0.025\</length\>

            \</cylinder\>

          \</geometry\>

           \<material\>

            \<script\>

              \<uri\>file://media/materials/scripts/gazebo.material\</uri\>

              \<name\>Gazebo/Grey\</name\>

            \</script\>

          \</material\>

        \</visual\>

        \<sensorname="camera"type="camera"\>

          \<pose\>0 0 0 -1.57 -1.57 0\</pose\>

          \<camera\>

            \<horizontal\_fov\>1.0472\</horizontal\_fov\>

            \<image\>

              \<width\>640\</width\>

              \<height\>480\</height\>

            \</image\>

            \<clip\>

              \<near\>0.05\</near\>

              \<far\>1000\</far\>

            \</clip\>

          \</camera\>

          \<always\_on\>1\</always\_on\>

          \<update\_rate\>10\</update\_rate\>

          \<visualize\>true\</visualize\>

         _\<!--  \<plugin name="irlock" filename="libArduCopterIRLockPlugin.so"\>_

              _\<fiducial\>irlock\_beacon\_01\</fiducial\>_

          _\</plugin\> --\>_

          \<pluginname="camera\_controller"filename="libgazebo\_ros\_camera.so"\>

          \<alwaysOn\>true\</alwaysOn\>

          \<updateRate\>0.0\</updateRate\>

          \<cameraName\>webcam\</cameraName\>

          \<imageTopicName\>image\_raw\</imageTopicName\>

          \<cameraInfoTopicName\>camera\_info\</cameraInfoTopicName\>

          \<frameName\>camera\_link\</frameName\>

          \<hackBaseline\>0.07\</hackBaseline\>

          \<distortionK1\>0.0\</distortionK1\>

          \<distortionK2\>0.0\</distortionK2\>

          \<distortionK3\>0.0\</distortionK3\>

          \<distortionT1\>0.0\</distortionT1\>

          \<distortionT2\>0.0\</distortionT2\>

      \</plugin\>

        \</sensor\>

      \</link\>

      _\<!-- attach camera --\>_

      \<jointtype="revolute"name="base\_camera\_joint"\>

        \<pose\>0 0 0.0 0 0 0\</pose\>

        \<parent\>iris::iris\_demo::gimbal\_small\_2d::tilt\_link\</parent\>

        \<child\>camera\</child\>

        \<axis\>

          \<limit\>

            \<lower\>0\</lower\>

            \<upper\>0\</upper\>

          \</limit\>

          \<xyz\>0 0 1\</xyz\>

          \<use\_parent\_model\_frame\>true\</use\_parent\_model\_frame\>

        \</axis\>

      \</joint\>

    \</model\>

  \</world\>

\</sdf\>

Create New ~/catkin\_ws/src/iq\_sim/launch called hills.launch file with below code

\<launch\>

  _\<!-- We resume the logic in empty\_world.launch, changing only the name of the world to be launched --\>_

  \<includefile="$(find gazebo\_ros)/launch/empty\_world.launch"\>

    \<argname="world\_name"value="$(find iq\_sim)/worlds/hills.world"/\>

    _\<!-- more default parameters can be changed here --\>_

  \</include\>

\</launch\>

1. Launch the new world with the command roslaunch iq\_sim hills.launch
2. Install Cuda using sudo apt install nvidia-cuda-toolkit
3. Clone the darknet repo into our catkin\_ws

cd ~/catkin\_ws/src

git clone --recursive https://github.com/leggedrobotics/darknet\_ros.git

1. Run commandcatkin build -DCMAKE\_BUILD\_TYPE=Release
2. If the above return "packages failed", then run this command to get gcc path in your env using_ **type -a gcc** _
3. catkin build -DCMAKE\_BUILD\_TYPE=Release -DCMAKE\_C\_COMPILER=\<\<the path returned in previous command\>\>
4. Edit from /webcam/image\_raw to /webcam/image\_raw in ros.yaml under darknet\_ros/darknet\_ros/config
5. Run roslaunch iq\_sim hills.launch
6. In another terminal run ./startsitlh

In another terminal run roslaunch darknet\_ros darknet\_ros.launch

**Notes from this week:**

1. **Focus on Trip report generation in C++.**
2. **Explore Virtual machines offered by amazon to speed up the yolo image recognition.**

## Trip report generation

Since a trip is a group of coordinates/waypoints a drone visited, we consider following data points at each waypoint.

1. Speed between each waypoint.
2. Battery before and after waypoint
3. Distance travelled
4. Total number of commands drone has received.

The trip generation is modular function which after trip prints out a html table of the whole trip as shown below.

![](RackMultipart20230224-1-nw31db_html_a55add6b97e15c07.png)

## YOLO subscription

The drone camera is setup and the object detection is carried out by [YOLO](https://pjreddie.com/darknet/yolo/). You only look once (YOLO) is a state-of-the-art, real-time object detection system.

We connect the rostopic _ **/webcam/image\_raw** _ which streams camera images from the drone in the gazebo world. Before connecting the camera raw stream, install the following for YOLO.

Do this step if your machine has GPU

sudo apt install nvidia-cuda-toolkit

Clone the darknet repo into our catkin\_ws

cd ~/catkin\_ws/src

git clone --recursive https://github.com/leggedrobotics/darknet\_ros.git

Build Darknet

catkin build -DCMAKE\_BUILD\_TYPE=Release -DCMAKE\_C\_COMPILER=/usr/bin/gcc-6

If the above steps throw a error, Find out the c\_compiler in your machine by the below command

gcc -c

And retry the build command with the compiler version

catkin build -DCMAKE\_BUILD\_TYPE=Release -DCMAKE\_C\_COMPILER=_ **\<\<your-compiler\>\>** _

The above steps are for Ubuntu 18.0.4

If your machine is Ubuntu 20.04, clone darknet using the below commands

cd ~/catkin\_ws/src

git clone https://github.com/kunaltyagi/darknet\_ros.git

git checkout opencv4

git submodule update --init –recursive

Build catkin using the below command

catkin build -DCMAKE\_BUILD\_TYPE=Release

In the file ros.yaml specifies ros parameters. You can find this file under darknet\_ros/darknet\_ros/config. You will need to change the image topic from _ **/camera/rgb/image\_raw** _ to

/webcam/image\_raw

In the file _ **darknet\_ros.launch** _ under _ **darknet\_ros/darknet\_ros/launch,** _ change the following l

\<arg name="network\_param\_file" default="$(find darknet\_ros)/config/_ **yolov2-tiny** _.yaml"/\>

- yolov1: Not recommended. this model is old
- yolov2: more accurate, and faster.
- yolov3: about as fast as v2, but more accurate. Yolo v3 has a high GPU ram requirement to train and run. If your graphics card does not have enough ram, use yolo v2
- tiny-yolo: Very fast yolo model. Would recommend for application where speed is most important. Works very well on Nvidia Jetson

We have successfully connected the camera stream to YOLO.

Now we run the Gazebo simulation and see how our drone passes down images to YOLO where the object detection takes place.

## Object detection and hover

Now for each command open new terminal and run the following commands

Launch Yolo detection ros topic

roslaunch darknet\_ros darknet\_ros.launch

Now launch the Gazebo simulation and insert a person as shown in "Adding models to Gazebo" section

roslaunch iq\_sim hills.launch

And start the SITL simulation by the below command

./startsitl.sh

Now launch the MavLink to enable communication between ROS and Ardupilot (SITL)

roslaunch iq\_sim apm.launch

Now for object detection replace the below code in square cpp and build it

#include\<waypoint\_functions.hpp\>

#include\<vector\>

#include\<ctime\>

#include\<fstream\>

#include\<iostream\>

#include\<string\>

#include\<map\>

#include\<unistd.h\>

#include\<darknet\_ros\_msgs/BoundingBoxes.h\>

float avg\_image\_p\_time =0;

float counter =1;

std::map\<std::string, int[1]\> FrameMap;

bool found;

voiddetection\_cb(const darknet\_ros\_msgs::BoundingBoxes::ConstPtr&_msg_)

{

    for (int i =0; i \<msg-\>bounding\_boxes.size(); i++)

    {

        if (msg-\>bounding\_boxes[i].Class=="person")

        {

            ROS\_INFO("Person found. calculating average time");

            found =true;

        }

    };

}

intsearch\_mode(std::string_lost\_item_, int_increment_, int_speed_, int_limit_)

{

    int counter =0;

    ros::Rate rate(2);

    while (ros::ok() &&!found)

    {

        ros::spinOnce();

        rate.sleep();

        if (check\_waypoint\_reached(.3) ==1)

        {

            std::vector\<double\> b =get\_current\_posing();

            if (b[1] == limit)

            {

                break;

            }

            set\_speed(speed);

            for (size\_t i =-1; i \<=1; i++)

            {

                if (check\_waypoint\_reached(.3) ==1)

                {

                    set\_destination(0, b[1], 2, i \*180);

                }

            }

            set\_destination(0, counter + increment, 3, 0);

            counter++;

        }

    }

    return0;

}

intmain(int_argc_, char\*\*_argv_)

{

    _// initialize ros_

    ros::init(argc, argv, "gnc\_node");

    ros::NodeHandle n;

    ros::Subscriber sub =n.subscribe("/darknet\_ros/bounding\_boxes", 1, detection\_cb);

    _// initialize control publisher/subscribers_

    init\_publisher\_subscriber(n);

    _// wait for FCU connection_

    wait4connect();

    _// wait for used to switch to mode GUIDED_

    wait4start();

    _// create local reference frame_

    initialize\_local\_frame();

    _// request takeoff_

    takeoff(3);

    search\_mode("person", 5, 5, 50);

    if (found)

    {

        wait4Land();

    }

}

Now to build the code, follow the instructions

Note: After any modifications to this code, we need to build them again using the below commands

cd catkin\_ws

catkin build

source ~/.bash.rc

Now in a new terminal run

rosrun iq\_gnc square

This will run the above program saved in square.cpp. Additionally, a refined code was created which also listens for commands published over the time. This is in the [battery\_metrics.cpp at project-drone (github.com)](https://github.com/ramHruday/project-drone/blob/main/src/battery_metrics.cpp)

Now open Gazebo and you would observe that the drone checks continually for the person and stops for land command once it detects a person.

![](RackMultipart20230224-1-nw31db_html_f576aa0f17e9fb89.png)

## Listening to commands

Here we design a command listener which analyses all the commands throughout the ros nodes.

It is an executable program running inside your application. You will write many nodes and put them into packages. Nodes are combined into a graph and communicate with each other using ROS topics, services, actions, etc.

Now to analyse a trip battery usage, we consider all nodes and the commands published to them. Then segregate the commands and parameters to find the corelation between commands and battery for the drone.

Code for the listening to the commands here at [commands\_listner.cpp · rh/project-drone (github.com)](https://github.com/ramHruday/project-drone/blob/main/include/commands_listner.cpp)

This code logs counters for all the node published on the ROS node. A sample of the [commands counter](https://github.com/ramHruday/project-drone/blob/main/commands_counter_report.html) is given below printed by a utility function. This utility function prints out the details of the commands in a html table.

![](RackMultipart20230224-1-nw31db_html_b61bbe1ca4233910.png)

## Image battery relation

To calculate the battery usage for image transmissions, two missions/test-cases were created.

1. Drone **with** camera sensor.
2. Drone **without** camera sensor

The drone in gazebo was modified accordingly in hills.Launch file found the catkin workspace. Also, the drone can be directly modified on gazebo.

Both the test cases were run for 15 metres, 1.5 min at same speed. The case with camera also has the detection script which runs detection scripts. The gazebo world has "Person" a gazebo object which we are detecting.

![](RackMultipart20230224-1-nw31db_html_b435e506895a72c.png)

The scripts are designed to print the resulting data to a html file. Below are the results.

_Fig 1 Without camera sensor_

_Fig 2 With camera sensor_

![](RackMultipart20230224-1-nw31db_html_ec7224946d3005ff.png) ![](RackMultipart20230224-1-nw31db_html_b56ca1606122f14e.png)

## Observations & Conclusions

1. A successful and complete drone simulation in Gazebo through C++ and ROS scripts has been setup.
2. Scripts created for drone movement like circular, square and straight-line path have been created.
3. Battery simulations seems to be a function of time rather than function of different tasks and its parameters. For example, a drone with speed 10m/s should consume more battery than a drone with 5m/s.
4. Battery usage for image transmission also seems to be function of time rather than the dependence on data transmission which is almost near 10~16images/sec.
5. The current study has been able to give data on details of commands, report generation and software setup.
6. This setup could be further extended to create a different samples of mission types in an actual drone, for example with or without different sensors and record the battery usage.
7. Has an scope of using machine learning algorithms to understand the pattern of battery usage from the command map generated.
8. All the code and documentation are present in a public [GitHub](https://github.com/ramHruday/project-drone) repository.

