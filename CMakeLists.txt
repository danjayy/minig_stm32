cmake_minimum_required(VERSION 3.0.2)
project(stm32)

## Compile as C++14
add_compile_options(-std=c++14 -Wall -Wextra)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cola2_msgs
  cola2_lib_ros
)

## System dependencies
find_package(COLA2_LIB REQUIRED)

## Declare catkin package
catkin_package(
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs cola2_msgs cola2_lib_ros
  DEPENDS COLA2_LIB
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
  )

## Include directories
include_directories(
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${COLA2_LIB_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/stm32.cpp

)

## Link libraries
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${COLA2_LIB_LIBRARIES}
)

## Declare a C++ executable
add_executable(stm32_node  scripts/stm32_node.cpp)

target_link_libraries(stm32_node ${PROJECT_NAME}_lib)


## Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
