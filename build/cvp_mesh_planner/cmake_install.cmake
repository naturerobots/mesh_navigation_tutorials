# Install script for directory: /home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/mesh_navigation/cvp_mesh_planner

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/cvp_mesh_planner")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/mesh_navigation/cvp_mesh_planner/cvp_mesh_planner.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/mesh_navigation/cvp_mesh_planner/include")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcvp_mesh_planner.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcvp_mesh_planner.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcvp_mesh_planner.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/libcvp_mesh_planner.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcvp_mesh_planner.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcvp_mesh_planner.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcvp_mesh_planner.so"
         OLD_RPATH "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/mbf_msgs/lib:/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/mbf_utility/lib:/opt/ros/jazzy/lib:/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/mesh_map/lib:/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/mesh_msgs/lib:/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/lvr2/lib:/usr/lib/x86_64-linux-gnu/openmpi/lib:/usr/lib/x86_64-linux-gnu/hdf5/serial:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libcvp_mesh_planner.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner/environment" TYPE FILE FILES "/opt/ros/jazzy/lib/python3.12/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner/environment" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/ament_cmake_environment_hooks/library_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/cvp_mesh_planner")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/cvp_mesh_planner")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner/environment" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner/environment" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/ament_cmake_index/share/ament_index/resource_index/packages/cvp_mesh_planner")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/mbf_mesh_core__pluginlib__plugin" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/ament_cmake_index/share/ament_index/resource_index/mbf_mesh_core__pluginlib__plugin/cvp_mesh_planner")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner/cmake/export_cvp_mesh_plannerExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner/cmake/export_cvp_mesh_plannerExport.cmake"
         "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/CMakeFiles/Export/1042e0ffc41f688c1d02edd8b72d19de/export_cvp_mesh_plannerExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner/cmake/export_cvp_mesh_plannerExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner/cmake/export_cvp_mesh_plannerExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner/cmake" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/CMakeFiles/Export/1042e0ffc41f688c1d02edd8b72d19de/export_cvp_mesh_plannerExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner/cmake" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/CMakeFiles/Export/1042e0ffc41f688c1d02edd8b72d19de/export_cvp_mesh_plannerExport-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner/cmake" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner/cmake" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/ament_cmake_export_libraries/ament_cmake_export_libraries-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner/cmake" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner/cmake" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner/cmake" TYPE FILE FILES
    "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/ament_cmake_core/cvp_mesh_plannerConfig.cmake"
    "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/ament_cmake_core/cvp_mesh_plannerConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cvp_mesh_planner" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/mesh_navigation/cvp_mesh_planner/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/cvp_mesh_planner/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
