cmake_minimum_required(VERSION 3.5.0)
project(tesseract_collision_benchmark VERSION 0.1.0 LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning_interface
  random_numbers
  tesseract_rosutils)

find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_urdf REQUIRED)
find_package(tesseract_environment REQUIRED)
find_package(tesseract_collision_physx REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS
    moveit_core
    moveit_ros_planning_interface
    random_numbers
    tesseract_rosutils
  DEPENDS
    EIGEN3
    console_bridge
    tesseract_geometry
    tesseract_common
    tesseract_collision
    tesseract_urdf
    tesseract_environment
    tesseract_collision_physx
)

add_executable(${PROJECT_NAME} src/tesseract_collision_benchmark.cpp src/utils.cpp)
target_link_libraries(${PROJECT_NAME}
  tesseract::tesseract_urdf
  tesseract::tesseract_environment_core
  tesseract::tesseract_environment_ofkt
  tesseract::tesseract_collision_bullet
  tesseract::tesseract_collision_fcl
  tesseract::tesseract_collision_physx
  ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
 )
