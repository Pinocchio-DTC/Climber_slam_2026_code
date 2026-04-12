cmake_minimum_required(VERSION 3.8)
project(cod_serial)

find_package(rclcpp REQUIRED)
find_package(rm_interfaces REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


include_directories(include)

add_executable(cod_serial  src/uart_transporter.cpp
        src/cod_serial.cpp
)

add_executable(talker src/talker.cpp)

ament_target_dependencies(cod_serial
        rclcpp
        rm_interfaces
)

ament_target_dependencies(talker
        rclcpp
        rm_interfaces
        geometry_msgs
)

target_link_libraries(cod_serial
    Eigen3::Eigen
)

target_link_libraries(talker
        serial
)

install(TARGETS cod_serial
        talker
        DESTINATION lib/${PROJECT_NAME}
)
ament_package()
