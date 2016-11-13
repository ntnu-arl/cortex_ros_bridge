cmake_minimum_required(VERSION 2.8.3)
project(cortex_bridge)

# Add support for C++ 11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin and any catkin packages
find_package ( catkin REQUIRED COMPONENTS 
								roscpp 
								message_generation 
								geometry_msgs 
								tf
)

## Add message files
add_message_files ( FILES
  									Marker.msg
										Markers.msg
)

## Add service files
add_service_files ( FILES
										cortexSetOrigin.srv
)

## Generate added messages and services
generate_messages(DEPENDENCIES geometry_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Add include dirs
include_directories(include ${catkin_INCLUDE_DIRS})

## Build publisher
add_executable(cortex_bridge src/cortex_bridge.cpp src/cortex.cpp src/cortex_socket.cpp 								src/cortex_unpack.cpp src/m3x3.cpp src/CortexHandler.cpp src/MarkerHandler.cpp src/TransformHandler.cpp)
target_link_libraries(cortex_bridge ${catkin_LIBRARIES})
add_dependencies(cortex_bridge cortex_bridge_generate_messages_cpp)