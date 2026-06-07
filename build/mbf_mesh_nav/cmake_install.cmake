# Install script for directory: /home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/mesh_navigation/mbf_mesh_nav

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/mbf_mesh_nav")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/mesh_navigation/mbf_mesh_nav/include/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmbf_mesh_server.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmbf_mesh_server.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmbf_mesh_server.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav/libmbf_mesh_server.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmbf_mesh_server.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmbf_mesh_server.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmbf_mesh_server.so"
         OLD_RPATH "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/mbf_msgs/lib:/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/mbf_abstract_nav/lib:/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/mbf_utility/lib:/opt/ros/jazzy/lib:/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/mesh_map/lib:/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/mesh_msgs/lib:/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/lvr2/lib:/usr/lib/x86_64-linux-gnu/openmpi/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libmbf_mesh_server.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mbf_mesh_nav/mbf_mesh_nav" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mbf_mesh_nav/mbf_mesh_nav")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mbf_mesh_nav/mbf_mesh_nav"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/mbf_mesh_nav" TYPE EXECUTABLE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav/mbf_mesh_nav")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mbf_mesh_nav/mbf_mesh_nav" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mbf_mesh_nav/mbf_mesh_nav")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mbf_mesh_nav/mbf_mesh_nav"
         OLD_RPATH "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/mbf_abstract_nav/lib:/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/mbf_utility/lib:/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/mbf_msgs/lib:/opt/ros/jazzy/lib:/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/mesh_map/lib:/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/mesh_msgs/lib:/usr/lib/x86_64-linux-gnu/openmpi/lib:/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav:/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/lvr2/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/mbf_mesh_nav/mbf_mesh_nav")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav/CMakeFiles/mbf_mesh_nav.dir/install-cxx-module-bmi-Release.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_mesh_nav/environment" TYPE FILE FILES "/opt/ros/jazzy/lib/python3.12/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_mesh_nav/environment" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav/ament_cmake_environment_hooks/library_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/mbf_mesh_nav")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/mbf_mesh_nav")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_mesh_nav/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_mesh_nav/environment" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_mesh_nav/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_mesh_nav/environment" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_mesh_nav" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_mesh_nav" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_mesh_nav" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_mesh_nav" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_mesh_nav" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav/ament_cmake_index/share/ament_index/resource_index/packages/mbf_mesh_nav")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_mesh_nav/cmake" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_mesh_nav/cmake" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_mesh_nav/cmake" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_mesh_nav/cmake" TYPE FILE FILES
    "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav/ament_cmake_core/mbf_mesh_navConfig.cmake"
    "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav/ament_cmake_core/mbf_mesh_navConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/mbf_mesh_nav" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/mesh_navigation/mbf_mesh_nav/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/mbf_mesh_nav/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
