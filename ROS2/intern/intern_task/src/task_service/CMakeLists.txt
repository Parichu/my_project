cmake_minimum_required(VERSION 3.8)
project(task_service)
find_package(rosidl_default_runtime REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Custom interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Feedbackservo.action"
  "action/TurnMotor.action"
  "srv/CustomTwist.srv"
  "action/ControlServo.action"
  "srv/MultiplyTwoInts.srv"
  "action/ClassRoom.action"
  "action/DeliveryItems.action"
  "srv/DeliveryRequest.srv"
  "action/CountUntil.action"
  "msg/Num.msg"
  "msg/Sphere.msg"
  "srv/Calculate.srv"
  DEPENDENCIES geometry_msgs
)

# C++ Executables
# add_executable(pub src/pub.cpp)
# ament_target_dependencies(pub rclcpp task_service)

# add_executable(sub src/sub.cpp)
# ament_target_dependencies(sub rclcpp task_service)

# add_executable(svr src/svr.cpp)
# ament_target_dependencies(svr rclcpp task_service)

# add_executable(cli src/cli.cpp)
# ament_target_dependencies(cli rclcpp task_service)

# install(TARGETS
#   pub
#   sub
#   svr
#   cli
#   DESTINATION lib/${PROJECT_NAME}
# )

# Python scripts
install(PROGRAMS
  src/turn_motor_action.py
  src/servo_control.py
  src/pir_lis.py
  src/multi_cli_add.py
  src/multi_srv.py
  src/clean.py
  src/Room.py
  src/delivery_client.py
  src/delivery_server.py
  src/cli_count_until.py
  src/count_until.py
  src/pub.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  src/sub.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pysub
)


ament_package()
