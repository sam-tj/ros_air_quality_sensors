
ros2 pkg create custom_message --build-type ament_cmake --dependencies rclcpp std_msgs
cd custom_message
mkdir -p msg
cd msg/
touch Sensdata.msg

paste following in Sensdata.msg
string time
int16 counter
int16 co2
int16 temperature
int16 r0value
int16 lpg
int16 co
int16 smoke

open CMakeList.txt 

paste below find packages
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

paste before ament_package

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Sensdata.msg"
  DEPENDENCIES builtin_interfaces
)


open package.xml

paste after depend tag

<build_depend>builtin_interfaces</build_depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>builtin_interfaces</exec_depend>
<exec_depend>rosidl_default_runtime</exec_depend>

<member_of_group>rosidl_interface_packages</member_of_group>

colcon build

ros2 interface list | grep Sensdata
ros2 interface show custom_message/msg/Sensdata


//ref https://www.theconstructsim.com/ros2-tutorials-7-how-to-create-a-ros2-custom-message-new/