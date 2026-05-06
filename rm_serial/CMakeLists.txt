cmake_minimum_required(VERSION 3.8)
project(rm_serial)

find_package(rclcpp REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(include)

add_executable(rm_serial  src/uart_transporter.cpp
        src/talker.cpp
)

add_executable(talker src/talker.cpp)

ament_target_dependencies(rm_serial
        rclcpp
        rm_interfaces
)

ament_target_dependencies(talker
        rclcpp
        rm_interfaces
        geometry_msgs
        std_msgs
)

target_link_libraries(rm_serial
    Eigen3::Eigen
        serial
)

target_link_libraries(talker
        serial
)

install(TARGETS rm_serial
        talker
        DESTINATION lib/${PROJECT_NAME}
)
ament_package()
