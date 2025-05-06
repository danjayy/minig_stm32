cmake_minimum_required(VERSION 3.0.2)
project(stm32)

## Compile as C++14
add_compile_options(-std=c++14 -Wall -Wextra)

## System dependencies
find_package(COLA2_LIB REQUIRED)

## List catkin macros and libraries
list(APPEND CATKIN_SOURCES
    roscpp
    sensor_msgs
    std_msgs
    cola2_msgs
    cola2_lib_ros
  )

find_package(catkin REQUIRED COMPONENTS ${CATKIN_SOURCES} message_generation actionlib genmsg)

# add_message_files(
#   FILES
#   PlannerParam.msg
# )

add_service_files(
  FILES
  Intensitylightcamfrequency.srv
)

generate_messages(
 DEPENDENCIES
 std_msgs
)

## Declare catkin package
catkin_package(
  LIBRARIES ${LIBRARY_NAME}
  CATKIN_DEPENDS ${CATKIN_SOURCES}
  DEPENDS COLA2_LIB
  INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
)

list(APPEND INCLUDES
  ${CMAKE_CURRENT_LIST_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${COLA2_LIB_INCLUDE_DIRS}
  )

## Include directories
include_directories(SYSTEM ${INCLUDES})

list(APPEND LIBRARIES
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${COLA2_LIB_LIBRARIES}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/stm32.cpp
)

add_dependencies(stm32 ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(stm32 ${catkin_LIBRARIES} ${LIBRARIES})

add_executable(stm32_node scripts/stm32_node.cpp)
target_link_libraries(stm32_node stm32)

# ## Link libraries
# target_link_libraries(${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${COLA2_LIB_LIBRARIES}
# )

# ## Declare a C++ executable
# add_executable(stm32_node  scripts/stm32_node.cpp)
# target_link_libraries(stm32_node ${PROJECT_NAME}_lib)


## Install targets
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# ## Install header files
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# ## Install launch files
# install(DIRECTORY launch/
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
# )
