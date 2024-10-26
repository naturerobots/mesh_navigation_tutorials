cmake_minimum_required(VERSION 3.8)
project(mesh_navigation_tutorials_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


# set env var such that gazebo simulation finds this packages' resources
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

# install
install(DIRECTORY config launch models worlds meshes urdf
  DESTINATION share/${PROJECT_NAME}
)



# Ground Truth Localization Node
add_library(ground_truth_localization SHARED
 src/ground_truth_localization.cpp)

target_include_directories(ground_truth_localization
 PRIVATE
   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include>)

target_link_libraries(ground_truth_localization
  Eigen3::Eigen
)

ament_target_dependencies(ground_truth_localization 
  rclcpp
  rclcpp_components
  geometry_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
)

rclcpp_components_register_node(ground_truth_localization PLUGIN "mesh_navigation_tutorials_sim::GroundTruthLocalizationNode" EXECUTABLE ground_truth_localization_node)

install(TARGETS ground_truth_localization 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



# test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
