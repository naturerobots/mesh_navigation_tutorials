# Install script for directory: /home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/move_base_flex/rviz_mbf_plugins

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/rviz_mbf_plugins")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/move_base_flex/rviz_mbf_plugins/plugins_description.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librviz_mbf_plugins.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librviz_mbf_plugins.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librviz_mbf_plugins.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/librviz_mbf_plugins.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librviz_mbf_plugins.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librviz_mbf_plugins.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librviz_mbf_plugins.so"
         OLD_RPATH "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/mbf_msgs/lib:/opt/ros/jazzy/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/home/haris/miniconda3/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/librviz_mbf_plugins.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/move_base_flex/rviz_mbf_plugins/include/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/move_base_flex/rviz_mbf_plugins/plugins_description.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins/environment" TYPE FILE FILES "/opt/ros/jazzy/lib/python3.12/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins/environment" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/ament_cmake_environment_hooks/library_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/rviz_mbf_plugins")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/rviz_mbf_plugins")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins/environment" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins/environment" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/ament_cmake_index/share/ament_index/resource_index/packages/rviz_mbf_plugins")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/rviz_common__pluginlib__plugin" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/ament_cmake_index/share/ament_index/resource_index/rviz_common__pluginlib__plugin/rviz_mbf_plugins")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins/cmake/export_rviz_mbf_pluginsExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins/cmake/export_rviz_mbf_pluginsExport.cmake"
         "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/CMakeFiles/Export/c252f45672a5fd6bcaf222e526a64e66/export_rviz_mbf_pluginsExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins/cmake/export_rviz_mbf_pluginsExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins/cmake/export_rviz_mbf_pluginsExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins/cmake" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/CMakeFiles/Export/c252f45672a5fd6bcaf222e526a64e66/export_rviz_mbf_pluginsExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^()$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins/cmake" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/CMakeFiles/Export/c252f45672a5fd6bcaf222e526a64e66/export_rviz_mbf_pluginsExport-noconfig.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins/cmake" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/ament_cmake_export_include_directories/ament_cmake_export_include_directories-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins/cmake" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins/cmake" TYPE FILE FILES
    "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/ament_cmake_core/rviz_mbf_pluginsConfig.cmake"
    "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/ament_cmake_core/rviz_mbf_pluginsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/rviz_mbf_plugins" TYPE FILE FILES "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/move_base_flex/rviz_mbf_plugins/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/build/rviz_mbf_plugins/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
