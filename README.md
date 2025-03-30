[//]: # (Image References)

[image1]: ./assets/starter-package.png "Starter package"
[image2]: ./assets/starter-package-1.png "Starter package"
[image3]: ./assets/links.png "Links"
[image4]: ./assets/shoulder.png "Shoulder"
[image5]: ./assets/joint-states.png "Joint states"
[image6]: ./assets/elbow.png "Elbow"
[image7]: ./assets/wrist.png "Wrist"
[image8]: ./assets/gripper.png "Gripper"


[image16]: ./assets/joint-states-1.png "Joint states"

# Week-9-10-Simple-arm

## This is how far we will get by the end of this lesson: 
  <a href="https://youtu.be/NkOX4zX9XbQ"><img width="600" src="./assets/youtube-navigation.png"></a>  

# Table of Contents
1. [Introduction](#introduction)  
1.1. [Download ROS package](#download-ros-package)  
1.2. [Test the starter package](#test-the-starter-package)  
2. [Building the robotic arm](#building-the-robotic-arm)  
2.1. [Shoulder](#shoulder)   
2.2. [Elbow](#elbow)  
2.3. [Wrist](#wrist)  
2.4. [Gripper](#gripper)  
2.5. [Joint state publishing](#joint-state-publishing) 
3. [ROS Controller](#ros-controller)  
3.1. [Joint trajectory control](#joint-trajectory-control)   
4. [3D model](#3d-model)  
5. [Grabbing objects](#grabbing-objects)  
5.1. [Using friction](#using-friction)  
5.2. [Using detachable joints](#using-detachable-joints)  
6. [Detecting collision](#detecting-collision)  
7. [Adding an end effector](#adding-an-end-effector)  
8. [Simulating cameras](#simulating-cameras)  
8.1. [Gripper camera](#gripper-camera)  
8.2. [Table camera](#table-camera)  
8.3. [RGBD camera](#rgbd-camera)  
9. [Moving the robot with a ROS node](#moving-the-robot-with-a-ros-node)  
9.1. [Forward kinematics](#forward-kinematics)  
9.2. [Inverse kinematics](#inverse-kinematics)  
10. [MoveIt 2](#moveit-2)  
10.1. [Changing the controller](#changing-the-controller)  
10.2. [Setup assistant](#setup-assistant)  
10.3. [Debugging](#debugging)  
11. [Fake 6 axis](#fake-six-axis)  
11.1. [Setting up moveit](#setting-up-moveit)  

# Introduction

In this lesson we'll lear how to build up a 4 axis robotic arm and make it move with our own ROS2 nodes, with our custom inverse kinematics and finally using MoveIt 2!

## Download ROS package

To download the starter package, clone the following git repo with the `starter-branch` (using the `-b branch` flag) into your colcon workspace:
```bash
git clone -b starter-branch https://github.com/MOGI-ROS/Week-9-10-Simple-arm
```

Let's take a look what's inside the `bme_ros2_simple_arm` package with the `tree` command!


```bash
.
├── CMakeLists.txt
├── package.xml
├── config
│   ├── amcl_localization.yaml
│   ├── ekf.yaml
│   ├── gz_bridge.yaml
│   ├── navigation.yaml
│   ├── slam_toolbox_localization.yaml
│   ├── slam_toolbox_mapping.yaml
│   └── waypoints.yaml
├── launch
│   ├── check_urdf.launch.py
│   ├── spawn_robot.launch.py
│   └── world.launch.py
├── maps
│   ├── my_map.pgm
│   ├── my_map.yaml
│   ├── serialized.data
│   └── serialized.posegraph
├── meshes
│   ├── lidar.dae
│   ├── mogi_bot.dae
│   └── wheel.dae
├── rviz
│   ├── localization.rviz
│   ├── mapping.rviz
│   ├── navigation.rviz
│   ├── rviz.rviz
│   └── urdf.rviz
├── urdf
│   ├── materials.xacro
│   ├── mogi_bot.gazebo
│   └── mogi_bot.urdf
└── worlds
    ├── empty.sdf
    └── home.sdf
```

Let's see what will we do with the existing files and folders:
- `config`: As we saw previously, we usually store parameters and large configuration files for ROS packages which aren't comfortable to handle from the launchfiles directly. In this lesson we will use more configuration files from this folder.
- `launch`: Default launch files are already part of the starting package, we can test the package with `spawn_robot.launch.py`.
- `maps`: Offline map files for the Gazebo world
- `meshes`: this folder contains the 3D models in `dae` format (collada mesh) that we use for our robot's body, wheels and lidar sensor.
- `rviz`: Pre-configured RViz2 layouts
- `urdf`: The URDF models of our robot, we'll extend the `mogi_bot.urdf` and `gazebo` files through this lesson
- `worlds`: default Gazebo worlds that we'll use in the simulations.

We have another package `bme_ros2_navigation_py` for our python scripts:
```bash
.
├── bme_ros2_navigation_py
│   ├── __init__.py
│   ├── send_initialpose.py
│   └── slam_toolbox_load_map.py
├── package.xml
├── resource
│   └── bme_ros2_navigation_py
├── setup.cfg
└── setup.py
```

## Test the starter package

After we downloaded the `starter-branch` from GitHub, let's rebuild the workspace and source the `install/setup.bash` file to make sure ROS and its tools are aware about the new package.

Let's test the package with the usual launch file:
```bash
ros2 launch bme_ros2_navigation spawn_robot.launch.py
```

![alt text][image1]

# Building the robotic arm

The base of the robotic arm is already in the URDF file, but the colors in RViz and Gazebo doesn't match. Before we move forward let's fix this, include the materials in the URDF file:

```xml
  <!-- STEP 3 - RViz colors -->
  <xacro:include filename="$(find bme_ros2_simple_arm)/urdf/materials.xacro" />
```
![alt text][image2]

Now we can proceed to adding the links of the robotic arm! We'll add the following links:

![alt text][image3]

## Shoulder

The shoulder of our robot is actually not 1 but 2 links, one for the pan and the other for lift. Let's add it to the URDF file:

```xml
  <!-- STEP 4 - Shoulder -->
  <joint name="shoulder_pan_joint" type="revolute">
    <limit lower="-3.14" upper="3.14" effort="330.0" velocity="3.14"/>
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <axis xyz="0 0 1"/>
    <origin xyz="0.0 0.0 0.05" rpy="0.0 0.0 0.0"/>
    <dynamics damping="0.0" friction="0.0"/>
  </joint>

  <!-- Shoulder link -->
  <link name="shoulder_link">
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <inertia ixx="0.0014" ixy="0.0" ixz="0.0"
               iyy="0.0014" iyz="0.0"
               izz="0.0025"
      />
    </inertial>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    </collision>
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="orange"/>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    </visual>
  </link>

  <!-- Shoulder lift joint -->
  <joint name="shoulder_lift_joint" type="revolute">
    <limit lower="-1.5708" upper="1.5708" effort="330.0" velocity="3.14"/>
    <parent link="shoulder_link"/>
    <child link="upper_arm_link"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0.0 0.0 0.025" rpy="0.0 0.0 0.0"/>
    <dynamics damping="0.0" friction="0.0"/>
  </joint>

  <!-- Upper arm link -->
  <link name="upper_arm_link">
    <inertial>
      <mass value="0.3"/>
      <origin xyz="0.0 0.0 0.1" rpy="0.0 0.0 0.0"/>
      <inertia ixx="0.0012" ixy="0.0" ixz="0.0"
               iyy="0.0012" iyz="0.0"
               izz="0.0004"
      />
    </inertial>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
      <origin xyz="0.0 0.0 0.1" rpy="0.0 0.0 0.0"/>
    </collision>
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.2"/>
      </geometry>
      <material name="orange"/>
      <origin xyz="0.0 0.0 0.1" rpy="0.0 0.0 0.0"/>
    </visual>
  </link>
```

Rebuild the workspace and try it:
```bash
ros2 launch bme_ros2_navigation spawn_robot.launch.py
```

![alt text][image4]

Joint state publisher opens a small GUI where we can adjust the angles of the two new shoulder joints, but this has clearly no impact on the simulation. We should always make sure that the `spawn_robot` launch file sends the joint states from the simulation in Gazebo and not from somewhere else - like the small GUI in this case. We'll handle this a bit later.

![alt text][image5]

## Elbow

Let's add the elbow that is the connecting joint between upper arm and forearm.

```xml
  <!-- STEP 5 - Elbow -->
  <joint name="elbow_joint" type="revolute">
    <limit lower="-2.3562" upper="2.3562" effort="150.0" velocity="3.14"/>
    <parent link="upper_arm_link"/>
    <child link="forearm_link"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0.0 0.0 0.2" rpy="0.0 0.0 0.0"/>
    <dynamics damping="0.0" friction="0.0"/>
  </joint>

  <!-- Forearm link -->
  <link name="forearm_link">
    <inertial>
      <mass value="0.2"/>
      <origin xyz="0.0 0.0 0.125" rpy="0.0 0.0 0.0"/>
      <inertia ixx="0.0011" ixy="0.0" ixz="0.0"
               iyy="0.0011" iyz="0.0"
               izz="0.0004"
      />
    </inertial>
    <collision>
      <geometry>
        <cylinder radius="0.025" length="0.25"/>
      </geometry>
      <origin xyz="0.0 0.0 0.125" rpy="0.0 0.0 0.0"/>
    </collision>
    <visual>
      <geometry>
        <cylinder radius="0.025" length="0.25"/>
      </geometry>
      <material name="orange"/>
      <origin xyz="0.0 0.0 0.125" rpy="0.0 0.0 0.0"/>
    </visual>
  </link>
```

Rebuild the workspace and try it:
```bash
ros2 launch bme_ros2_navigation spawn_robot.launch.py
```

![alt text][image6]

## Wrist

Now add the wrist of the robotic arm:

```xml
  <!-- STEP 6 - Wrist -->
  <joint name="wrist_joint" type="revolute">
    <limit lower="-2.3562" upper="2.3562" effort="54.0" velocity="3.14"/>
    <parent link="forearm_link"/>
    <child link="wrist_link"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0.0 0.0 0.25" rpy="0.0 0.0 0.0"/>
    <dynamics damping="0.0" friction="0.0"/>
  </joint>

  <!-- Wrist link -->
  <link name="wrist_link">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0.0 0.0 0.05" rpy="0.0 0.0 0.0"/>
      <inertia ixx="0.00009" ixy="0.0" ixz="0.0"
               iyy="0.00009" iyz="0.0"
               izz="0.00002"
      />
    </inertial>
    <collision>
      <geometry>
        <cylinder radius="0.02" length="0.1"/>
      </geometry>
      <origin xyz="0.0 0.0 0.05" rpy="0.0 0.0 0.0"/>
    </collision>
    <visual>
      <geometry>
        <cylinder radius="0.02" length="0.1"/>
      </geometry>
      <material name="orange"/>
      <origin xyz="0.0 0.0 0.05" rpy="0.0 0.0 0.0"/>
    </visual>
  </link>
```

Rebuild the workspace and try it:
```bash
ros2 launch bme_ros2_navigation spawn_robot.launch.py
```

![alt text][image7]


## Gripper

And finally add a gripper. The gripper conists of a base and two fingers. The fingers are connected with prismatic joints to the base. Also we improve the friction parameters of the fingers in the simulation:

```xml
  <!-- STEP 7 - Gripper -->
  <joint name="gripper_base_joint" type="fixed">
    <parent link="wrist_link"/>
    <child link="gripper_base"/>
    <origin xyz="0.0 0 0.105" rpy="0.0 0 0"/> 
  </joint>

  <!-- Gripper base link -->
  <link name="gripper_base">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <inertia ixx="0.00009" ixy="0.0" ixz="0.0"
               iyy="0.00009" iyz="0.0"
               izz="0.00002"
      />
    </inertial>
    <collision>
      <geometry>
        <box size=".05 .1 .01"/>
      </geometry>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    </collision>
    <visual>
      <geometry>
        <box size=".05 .1 .01"/>
      </geometry>
      <material name="grey"/>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    </visual>
  </link>

  <!-- Left finger joint -->
  <joint name="left_finger_joint" type="prismatic">
    <limit lower="0" upper="0.04" effort="100.0" velocity="4.0"/>
    <parent link="gripper_base"/>
    <child link="left_finger"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0.0 0.01 0.045" />
  </joint>

  <!-- Left finger link -->
  <link name="left_finger">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <inertia ixx="0.00009" ixy="0.0" ixz="0.0"
               iyy="0.00009" iyz="0.0"
               izz="0.00002"
      />
    </inertial>
    <collision>
      <geometry>
        <box size=".04 .01 .08"/>
      </geometry>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    </collision>
    <visual>
      <geometry>
        <box size=".04 .01 .08"/>
      </geometry>
      <material name="blue"/>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    </visual>
  </link>

  <gazebo reference="left_finger">
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <mu1>15</mu1>
    <mu2>15</mu2>
    <fdir1>1 0 0</fdir1>
    <maxVel>1.0</maxVel>
    <minDepth>0.002</minDepth>
  </gazebo>

  <!-- Right finger joint -->
  <joint name="right_finger_joint" type="prismatic">
    <limit lower="0" upper="0.04" effort="100.0" velocity="4.0"/>
    <parent link="gripper_base"/>
    <child link="right_finger"/>
    <axis xyz="0 -1 0"/>
    <origin xyz="0.0 -0.01 0.045" />
  </joint>

  <!-- Right finger link -->
  <link name="right_finger">
    <inertial>
      <mass value="0.1"/>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
      <inertia ixx="0.00009" ixy="0.0" ixz="0.0"
               iyy="0.00009" iyz="0.0"
               izz="0.00002"
      />
    </inertial>
    <collision>
      <geometry>
        <box size=".04 .01 .08"/>
      </geometry>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    </collision>
    <visual>
      <geometry>
        <box size=".04 .01 .08"/>
      </geometry>
      <material name="blue"/>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    </visual>
  </link>

  <gazebo reference="right_finger">
    <kp>1000000.0</kp>
    <kd>100.0</kd>
    <mu1>15</mu1>
    <mu2>15</mu2>
    <fdir1>1 0 0</fdir1>
    <maxVel>1.0</maxVel>
    <minDepth>0.002</minDepth>
  </gazebo>
```

Rebuild the workspace and try it:
```bash
ros2 launch bme_ros2_navigation spawn_robot.launch.py
```

![alt text][image8]



## Joint state publishing

As we noticed earlier, we can move the joint angles of the robotic arm in RViz using the `joint_state_publisher_gui` but this has no impact on the simulation. To move the arm inb the simulation first we have to turn off the `joint_state_publisher_gui` in the `spawn_robot.launch.py`, it means the small GUI won't open anymore using this launch file.

The next step is to forward `joint_states` from Gazebo to ROS, this we can set up in the config file of `gz_bridge` as we did previously with the mobile robots too:

```yaml
- ros_topic_name: "joint_states"
  gz_topic_name: "joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: "GZ_TO_ROS"
```

And the last step is to add a `mogi_arm.gazebo` file in the urdf folder with a joint state publisher plugin:
```xml
<robot>
  <gazebo>
    <plugin
        filename="gz-sim-joint-state-publisher-system"
        name="gz::sim::systems::JointStatePublisher">
        <topic>joint_states</topic> <!--from <ros><remapping> -->
        <joint_name>shoulder_pan_joint</joint_name>
        <joint_name>shoulder_lift_joint</joint_name>
        <joint_name>elbow_joint</joint_name>
        <joint_name>wrist_joint</joint_name>
        <joint_name>left_finger_joint</joint_name>
        <joint_name>right_finger_joint</joint_name>
    </plugin>
  </gazebo>
</robot>
```

And of course, include it in the beginning of our URDF file:

```xml
  <!-- STEP 8 - Gazebo plugin -->
  <xacro:include filename="$(find bme_ros2_simple_arm)/urdf/mogi_arm.gazebo" />
```

Rebuild the workspace and try it:
```bash
ros2 launch bme_ros2_navigation spawn_robot.launch.py
```

![alt text][image6]

# ROS Controller


After ROS control joint state gz bridge can be removed

- ros_topic_name: "joint_states"
  gz_topic_name: "joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: "GZ_TO_ROS"



  ros2 param set /move_group use_sim_time true