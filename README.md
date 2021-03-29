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



## 3D modell

# Robot betöltése Gazebo-ba

# Transmission és Controller

# Jointok mozgatása

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