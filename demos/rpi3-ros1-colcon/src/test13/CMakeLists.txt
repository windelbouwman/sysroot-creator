cmake_minimum_required(VERSION 3.0.2)
project(test13)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/talker.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS ${PROJECT_NAME}_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
