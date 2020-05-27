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

add_executable(${PROJECT_NAME} src/tesseract_collision_benchmark.cpp src/utils.cpp)
target_link_libraries(${PROJECT_NAME}
  tesseract::tesseract_urdf
  tesseract::tesseract_environment_kdl
  tesseract::tesseract_collision_bullet
  tesseract::tesseract_collision_fcl
  tesseract::tesseract_collision_physx
  ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(tesseract_collision_benchmark SYSTEM PUBLIC ${catkin_INCLUDE_DIRS})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
 )
