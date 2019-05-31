# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(vizzy_serial_interfaces)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    message_generation
    nodelet
    pluginlib
    roscpp
    roslint
    sensor_msgs
    tf
    vizzy_msgs
    )

catkin_package(
  CATKIN_DEPENDS
    message_runtime
    nodelet
    sensor_msgs
    tf
    serial
  INCLUDE_DIRS
    include
)

include_directories(include ${CMAKE_INSTALL_PREFIX}/include ${catkin_INCLUDE_DIRS} )


## Declare a cpp executable



add_library(vizzy_battery_monitor_interface src/VoltageCurrentSerialInterface.cpp)
target_link_libraries(vizzy_battery_monitor_interface 
${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
  ${CMAKE_INSTALL_PREFIX}/lib/libserial.so
)
#add_executable(battery_monitor_example src/VoltageCurrentSerialInterface.cpp)

add_executable(kokam_battery_state_service src/battery_state_service.cpp)

target_link_libraries(kokam_battery_state_service 
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
  vizzy_battery_monitor_interface
  ${CMAKE_INSTALL_PREFIX}/lib/libserial.so
)

add_executable(segway_battery_state_service src/segway_battery_state_service.cpp)

target_link_libraries(segway_battery_state_service 
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
  vizzy_battery_monitor_interface
  ${CMAKE_INSTALL_PREFIX}/lib/libserial.so
)



add_executable(battery_charging_state_service src/battery_charging_state_service.cpp)

target_link_libraries(battery_charging_state_service 
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
  vizzy_battery_monitor_interface
  ${CMAKE_INSTALL_PREFIX}/lib/libserial.so
)

add_executable(kokam_power_publisher src/kokam_power_publisher.cpp)

target_link_libraries(kokam_power_publisher 
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
  vizzy_battery_monitor_interface
  ${CMAKE_INSTALL_PREFIX}/lib/libserial.so
)

# Install the plugin and the plugin description
install (DIRECTORY include/vizzy_sensors/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install runtime launch and configuration files
foreach(dir config launch)
  install(DIRECTORY ${dir}/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

# unit tests are enabled selectively
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
endif()
