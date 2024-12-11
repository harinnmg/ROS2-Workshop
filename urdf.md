#URDF for RVIZ
```
<?xml version="1.0"?>
<robot name="myfirst">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </visual>
  </link>
</robot>
```
Launch file for RVIZ
```
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
import xml.etree.ElementTree as ElementTree


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('my_robot')

 

    # Define the path to the URDF file
    urdf_file_name = 'my_robot.urdf'
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file_name)
    with open(urdf_path, 'rb') as f:
     xml_data = f.read()
    xml_parsed = ElementTree.fromstring(xml_data)
    return LaunchDescription([
        
     
        Node(
             package='joint_state_publisher',
             executable='joint_state_publisher',
             name='joint_state_publisher',
             ),
 
        # Publish the URDF to the robot_description topic
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_path]
        ),
        Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        ),
        # Spawn the robot entity
 
    ])

if __name__ == '__main__':
    generate_launch_description()


```
<?xml version="1.0"?>
<robot name="multipleshapes">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Right Leg -->
  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <material name="green">
        <color rgba="0.2 0.8 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- Joint between base_link and right_leg -->
  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>
</robot>
```
launch file
```
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('hello1')

    # Define the path to the world file
   
    # Define the path to the URDF file
    urdf_file_name = 'check.urdf'
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file_name)

    return LaunchDescription([
        # Start Gazebo server with the world file
        DeclareLaunchArgument(name='use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')]),
            
        ),
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        # Start Gazebo client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')]),
        ),

        Node(
             package='joint_state_publisher',
             executable='joint_state_publisher',
             name='joint_state_publisher',
             ),
 
        # Publish the URDF to the robot_description topic
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            arguments=[urdf_path]
        ),
        Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
       
        ),
        # Spawn the robot entity
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=['-entity', 'test2', '-topic', 'robot_description', "-x", "0.0", "-y", "0.0", "-z", "0.0"],
            output="screen"
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
```
