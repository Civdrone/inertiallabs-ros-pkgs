cmake_minimum_required(VERSION 3.5)
project(inertiallabs_ins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(civros_msgs REQUIRED)
find_package(civros_utils REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(inertiallabs_msgs REQUIRED)
find_package(connectivity_client2 REQUIRED)
find_package(OpenCV)
find_package(rclcpp REQUIRED)

# find_package(system_lib)

include_directories(
  include
  ../inertiallabs_sdk/
  ${civros_msgs_INCLUDE_DIRS} 
  ${civros_utils_INCLUDE_DIRS} 
  ${geometry_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${inertiallabs_msgs_INCLUDE_DIRS}
  ${connectivity_client2_INCLUDE_DIRS}
  ${Opencv_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  )


set(DEPRECATION_FLAG "-Wsizeof-array-argument")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra ${DEPRECATION_FLAG}")

if(WIN32)
	set(IL_PLATFORM "windows")
else()
	set(IL_PLATFORM "linux")
endif()

add_executable(il_ins src/il_ins.cpp
                      ../inertiallabs_sdk/ILDriver.cpp
                      ../inertiallabs_sdk/UDDParser.cpp
                      ../inertiallabs_sdk/platforms/${IL_PLATFORM}/SerialPort.cpp
                      ../inertiallabs_sdk/platforms/${IL_PLATFORM}/NetClient.cpp
)

target_link_libraries(il_ins 
                      ${civros_msgs_LIBRARIES}
                      ${connectivity_client2_LIBRARIES}
                      ${diagnostic_msgs_LIBRARIES}
                      ${rclcpp_LIBRARIES}
                      ${inertiallabs_msgs_LIBRARIES}
)

if(WIN32)
	target_link_libraries(il_ins ${catkin_LIBRARIES} )
else()
	target_link_libraries(il_ins ${catkin_LIBRARIES} pthread )
endif()
target_link_libraries(il_ins ${connectivity_client2_LIBRARIES})

install(TARGETS il_ins
    DESTINATION lib/${PROJECT_NAME})
