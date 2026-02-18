cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

find_package(moveit_core REQUIRED)
find_package(random_numbers REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(tesseract_geometry REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(tesseract_collision REQUIRED)
find_package(tesseract_collision_coal REQUIRED)
find_package(tesseract_urdf REQUIRED)
find_package(tesseract_environment REQUIRED)
# find_package(tesseract_collision_physx REQUIRED)

add_executable(${PROJECT_NAME} src/tesseract_collision_benchmark.cpp)
target_link_libraries(${PROJECT_NAME}
  tesseract::tesseract_urdf
  tesseract::tesseract_environment
  tesseract::tesseract_collision_bullet
  tesseract::tesseract_collision_fcl
  tesseract::tesseract_collision_coal_coal
  # tesseract::tesseract_collision_physx
  moveit_core::moveit_collision_detection
  moveit_core::moveit_collision_detection_fcl
  moveit_core::moveit_collision_detection_bullet
  moveit_core::moveit_planning_scene
  moveit_core::moveit_robot_model
  moveit_core::moveit_test_utils
  ${random_numbers_LIBRARIES}
)
target_include_directories(${PROJECT_NAME} PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME} PRIVATE SYSTEM
    ${random_numbers_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_only src/tesseract_collision_benchmark_only.cpp)
target_link_libraries(${PROJECT_NAME}_only
  tesseract::tesseract_urdf
  tesseract::tesseract_environment
  tesseract::tesseract_collision_bullet
  tesseract::tesseract_collision_fcl
  tesseract::tesseract_collision_coal_coal
  # tesseract::tesseract_collision_physx
  ${random_numbers_LIBRARIES}
)
target_include_directories(${PROJECT_NAME}_only PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_only PRIVATE SYSTEM
    ${random_numbers_INCLUDE_DIRS})

configure_package(
  NAMESPACE tesseract
  TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_only)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
 )

foreach(dir data)
  install(DIRECTORY ${dir}/ DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

install(PROGRAMS scripts/plot_benchmark_csv.py
  DESTINATION share/${PROJECT_NAME}/scripts)
