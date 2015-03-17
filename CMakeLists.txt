cmake_minimum_required(VERSION 2.8.4)
project(staubli_jointtrajectory)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(SDF sdformat)
endif()

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
    ${SDFormat_INCLUDE_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/msgs
    )

link_directories(
    ${GAZEBO_LIBRARY_DIRS}
    ${SDFormat_LIBRARY_DIRS}
    ${CMAKE_CURRENT_BINARY_DIR}/msgs
    )
add_subdirectory(msgs)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_library (StaubliKinematicModelPlugin SHARED StaubliKinematicModelPlugin.cc)
target_link_libraries(StaubliKinematicModelPlugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(StaubliKinematicModelPlugin staubli_joint_states_msgs)
