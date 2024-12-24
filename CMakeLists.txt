cmake_minimum_required(VERSION 3.5)
project(miivii_bynav_driver)

# 找到ROS 2包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(swri_serial_util REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 包含头文件
include_directories(include)
include_directories(${swri_serial_util_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})


file(GLOB parsers_srcs "src/parsers/*.cpp")
file(GLOB MSG_FILES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "msgs/*.msg")

#添加自定的msg
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  DEPENDENCIES builtin_interfaces std_msgs)
ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 创建包
ament_package()

# 定义可执行文件
add_executable(miivii_bynav_node 
                  "src/bynav_node.cpp"
                  "src/bynav_control.cpp"
                  "src/bynav_connection.cpp"
                  ${parsers_srcs}
              )

#必须设置,否则找不到自定义的msg
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# 链接ROS 2依赖项   
ament_target_dependencies(miivii_bynav_node rclcpp std_msgs sensor_msgs swri_serial_util Boost tf2_geometry_msgs)
target_link_libraries(miivii_bynav_node ${typesupport_target})

# 安装可执行文件
install(TARGETS miivii_bynav_node DESTINATION lib/${PROJECT_NAME})
