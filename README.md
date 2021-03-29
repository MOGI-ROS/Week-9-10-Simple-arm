[//]: # (Image References)

[image1]: ./assets/gazebo1.png "Gazebo"
[image2]: ./assets/base_link_1.png "base_link"
[image3]: ./assets/base_link_2.png "base_link"
[image4]: ./assets/shoulder_1.png "shoulder"
[image5]: ./assets/shoulder_2.png "shoulder"
[image6]: ./assets/elbow_1.png "elbow"
[image7]: ./assets/elbow_2.png "elbow"
[image8]: ./assets/wrist_1.png "wrist"
[image9]: ./assets/wrist_2.png "wrist"
[image10]: ./assets/links.png "links"
[image11]: ./assets/gripper_1.png "gripper"
[image12]: ./assets/gripper_2.png "gripper"

# 9. - 10. hét - robotkarok

# Hova fogunk eljutni?
<a href="https://youtu.be/XveJMrCh9vw"><img height="400" src="./assets/youtube1.png"></a> 

# Kezdőcsomag

A kiindulási csomag tartalmazza a Gazebo világot a launchfájlokat és az RViz konfigurációját, minden mást mi fogunk felépíteni közösen!

A kezdőprojekt letöltése:
```console
git clone -b starter-branch https://github.com/MOGI-ROS/Week-9-10-Simple-arm.git
```

A kezdőprojekt tartalma a következő:
```console
david@DavidsLenovoX1:~/bme_catkin_ws/src/Week-9-10-Simple-arm/bme_ros_simple_arm$ tree
.
├── CMakeLists.txt
├── config
│   ├── costmap_common_params.yaml
│   ├── dwa_local_planner_params.yaml
```

# Gazebo világ

A fejezetben egy Gazebo világot fogunk használni, ami két asztalból és néhány megfogható testből áll:
![alt text][image1]

A szimulációt el tudjuk indítani a következő launch fájllal:
```console
roslaunch bme_ros_simple_arm world.launch
```

Vegyük észre, hogy van egy apró változás a `world.launch` fájlban, ugyanis a Gazebo szimulációnk megállítva indul!

```xml
<arg name="paused" value="true"/>
```

# Robot kar építése URDF-fel

![alt text][image10]

## base_link

Hozzuk létre a robotunk első linkjét, ami a robotkar alapja lesz. A robotkar felépítése során hengerekkel fogunk dolgozni, és ennek megfelelően megpróbálunk reális értékeket választani a tömeg és a tehetetlenségi nyomaték mátrixának. Ehhez használhatjuk a mellékelt `inertia_calculator.xlsx` segítségképpen.

Hozzuk létre a `mogi_arm.xacro` fájlt az urdf mappában:

```xml
<?xml version="1.0"?>

<robot name="mogi_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- RViz colors -->
  <xacro:include filename="$(find bme_ros_simple_arm)/urdf/materials.xacro" />

  <!-- Global reference link -->
  <link name="world"/>

  <joint name="fixed_base" type="fixed">
    <parent link="world"/>
    <child link="base_link"/>
  </joint>

  <!-- Arm base link -->
  <link name="base_link">
    <inertial>
      <mass value="2"/>
      <origin xyz="0.0 0.0 0.0"/>
      <inertia ixx="0.0117" ixy="0.0" ixz="0.0" 
               iyy="0.0117" iyz="0.0"
               izz="0.0225"
      />
    </inertial>
    <collision>
      <geometry>
        <cylinder radius="0.15" length="0.05"/>
      </geometry>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    </collision>
    <visual>
      <geometry>
        <cylinder radius="0.15" length="0.05"/>
      </geometry>
      <material name="grey"/>
      <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
    </visual>
  </link>

  <gazebo reference="base_link">
    <material>Gazebo/Grey</material>
  </gazebo>

</robot>
```

A `base_link`-ünk egy egyszerű lapos korong, de ennek ellenére megbizonyosodhatunk arról, hogy minden rendben van-e vele, ha megvizsgáljuk a `check_urdf.launch` segítségével:
![alt text][image2]

Illetve megnézhetjük a `spawn_robot.launch` fájlunkat is működés közben:
![alt text][image3]

## shoulder linkek

A robot válla két jointból fog állni, az egyik a `base_link` körüli forgatást, a másik a "felkar" mozgatását csinálja. Adjuk hozzá a két új linket és jointot az urdf fájlunkhoz:

```xml
  <!-- Shoulder pan joint -->
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

  <gazebo reference="shoulder_link">
    <material>Gazebo/Orange</material>
  </gazebo>

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

  <gazebo reference="upper_arm_link">
    <material>Gazebo/Orange</material>
  </gazebo>
```

A `check_urdf.launch` segítségével meg tudjuk mozgatni a robotkarunk vállának joint-jait:
![alt text][image4]

És a Gazebo szimulációban is elhelyezhetjük a `spawn_robot.launch` fájllal:
![alt text][image5]

Láthatjuk, hogy a Gazebo szimulációban nem tudjuk olyan egyszerűen mozgatni még a csuklókat, ehhez motorokat kell majd szimulálnunk a robotkar csuklóiban, de ezt csak azután tesszük meg, hogy elkészültünk a teljes karral!

## könyök

```xml
  <!-- Elbow joint -->
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

  <gazebo reference="forearm_link">
    <material>Gazebo/Orange</material>
  </gazebo>
```

![alt text][image6]

![alt text][image7]

## csukló

```xml
  <!-- Wrist joint -->
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

  <gazebo reference="wrist_link">
    <material>Gazebo/Orange</material>
  </gazebo>
```

![alt text][image8]

![alt text][image9]

## gripper

Először adjuk hozzá a gripperünk alapját:

```xml
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

  <gazebo reference="gripper_base">
    <material>Gazebo/Grey</material>
  </gazebo>
```

Majd ezek után a jobb és bal ujjakat:
```xml
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
    <material>Gazebo/Blue</material>
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
    <material>Gazebo/Blue</material>
  </gazebo>
```

Vegyük észre, hogy az ujjaknak megváltoztatjuk a mechanikai tulajdonságait, többek között a surlódását, ami ahhoz kell, hogy fogni tudjunk majd a gripperrel.

![alt text][image11]

![alt text][image12]

# Transmission és Controller

Ahhoz, hogy a szimulált robotkarunkat meg tudjuk mozdítani végre kell hajtanunk néhány lépést.

Először is adjuk hozzá a `transmission.xacro` és a `mogi_arm.gazebo` fájlokat az urdf-ünkhöz:
```xml
  <!-- Transmisions -->
  <xacro:include filename="$(find bme_ros_simple_arm)/urdf/transmission.xacro" />
  <!-- Gazebo plugin -->
  <xacro:include filename="$(find bme_ros_simple_arm)/urdf/mogi_arm.gazebo" />
```

Ha ezután elindítjuk a `spawn_robot.launch` fájlt, a következő üzeneteket látjuk majd a konzolban:
```console
[ INFO] [1617044019.576815200]: Loading gazebo_ros_control plugin
[ INFO] [1617044019.577572300]: Starting gazebo_ros_control plugin in namespace: /
[ INFO] [1617044019.584272700]: gazebo_ros_control plugin is waiting for model URDF in parameter [robot_description] on the ROS param server.
[ERROR] [1617044019.730203800]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/shoulder_pan_joint
[ERROR] [1617044019.732750200]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/shoulder_lift_joint
[ERROR] [1617044019.735765000]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/elbow_joint
[ERROR] [1617044019.738999900]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/wrist_joint
[ERROR] [1617044019.742558100]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/left_finger_joint
[ERROR] [1617044019.748125500]: No p gain specified for pid.  Namespace: /gazebo_ros_control/pid_gains/right_finger_joint
[ INFO] [1617044019.772403000]: Loaded gazebo_ros_control.
```

Ez azt jelenti, hogy a szimulált hardware interfészeink működnek, a ROS control csomagja megtalálta őket, de ezzel még nincs vége a feladatnak, hiszen továbbra sem tudjuk mozgatni a kart. A mozgatáshoz az `rqt_joint_trajectory_controller`-ét fogjuk használni, amit így tudunk elindítani:

```console
rosrun rqt_joint_trajectory_controller rqt_joint_trajectory_controller
```

Azonban ez még nem látja a controller-t. Ehhez fel kell paramétereznünk a `controller_manager`-t, amit a `spawn_robot.launch` módosításával tudunk megtenni:

```xml
  <!-- Send joint values-->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="use_gui" value="false"/>
  </node>
```

Az eredeti `joint state publisher` node-ot cseréljük ki a következőre:

```xml
  <!-- Joint_state_controller -->
  <rosparam file="$(find bme_ros_simple_arm)/controller/joint_state_controller.yaml" command="load"/>
  <node name="joint_state_controller_spawner" pkg="controller_manager" type="controller_manager" args="spawn joint_state_controller" respawn="false" output="screen"/>

  <!-- Start this controller -->
  <rosparam file="$(find bme_ros_simple_arm)/controller/arm_controller.yaml" command="load"/>
  <node name="arm_controller_spawner" pkg="controller_manager" type="controller_manager" args="spawn arm_controller" respawn="false" output="screen"/>
```

Ha most indítjuk el a `spawn_robot.launch` fájlt, akkor láthatjuk, hogy megmaradt továbbra is a figyelmeztetés:
```console
No p gain specified for pid.
```

Másrészt viszont működik a controllerünk, és tudjuk mozgatni a robotkart a fizikai szimulációban!

# 3D modell

# Megfogás

## Fizikai szimulációval

### Inerciaszámítás

### PID tuning

## Tárgy rögzítésével

### Attach/detach

### Collsion érzékelés

# Szimulált kamerák

# Robotkar mozgatása ROS node-dal

# Inverz kinematika