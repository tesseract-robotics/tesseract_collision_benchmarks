cmake_minimum_required(VERSION 3.15.0)

# Extract package name and version
find_package(ros_industrial_cmake_boilerplate REQUIRED)
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES C CXX)

find_package(moveit_core REQUIRED)
find_package(random_numbers REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(tesseract REQUIRED COMPONENTS common geometry urdf collision collision_bullet collision_fcl environment)
find_package(tesseract_collision_coal REQUIRED)
# find_package(tesseract_collision_physx REQUIRED)

add_executable(${PROJECT_NAME} src/tesseract_collision_benchmark.cpp)
target_link_libraries(
  ${PROJECT_NAME}
  tesseract::urdf
  tesseract::environment
  tesseract::collision_bullet
  tesseract::collision_fcl
  tesseract::tesseract_collision_coal_coal
  # tesseract::tesseract_collision_physx
  moveit_core::moveit_collision_detection
  moveit_core::moveit_collision_detection_fcl
  moveit_core::moveit_collision_detection_bullet
  moveit_core::moveit_planning_scene
  moveit_core::moveit_robot_model
  moveit_core::moveit_test_utils
  random_numbers::random_numbers)
target_include_directories(${PROJECT_NAME} PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                   "$<INSTALL_INTERFACE:include>")

add_executable(${PROJECT_NAME}_only src/tesseract_collision_benchmark_only.cpp)
target_link_libraries(
  ${PROJECT_NAME}_only
  tesseract::urdf
  tesseract::environment
  tesseract::collision_bullet
  tesseract::collision_fcl
  tesseract::tesseract_collision_coal_coal
  # tesseract::tesseract_collision_physx
  random_numbers::random_numbers)
target_include_directories(${PROJECT_NAME}_only PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                                                        "$<INSTALL_INTERFACE:include>")

configure_package(NAMESPACE tesseract TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_only)

# Gate on TESSERACT_ENABLE_TESTING, as sibling packages do. BUILD_TESTING is ON here only as a side
# effect of a dependency calling include(CTest).
if(TESSERACT_ENABLE_TESTING)
  enable_testing()
  add_test(NAME cli_contract COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/test/cli_contract.bash
                                     $<TARGET_FILE:${PROJECT_NAME}_only>)
  # Several cases run a full benchmark, and the trial count is compiled in.
  set_tests_properties(cli_contract PROPERTIES TIMEOUT 900)
endif()

# Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE)

foreach(dir data)
  install(DIRECTORY ${dir}/ DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

install(PROGRAMS scripts/plot_benchmark_csv.py DESTINATION share/${PROJECT_NAME}/scripts)
