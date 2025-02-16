# Week-9-10-Simple-arm
Simulation of a simple robotic arm using ROS2 Jazzy, Gazebo Harmonic and MoveIt






After ROS control joint state gz bridge can be removed

- ros_topic_name: "joint_states"
  gz_topic_name: "joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: "GZ_TO_ROS"