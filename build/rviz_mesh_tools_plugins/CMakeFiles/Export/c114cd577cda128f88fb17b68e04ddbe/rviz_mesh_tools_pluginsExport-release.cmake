#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rviz_mesh_tools_plugins::rviz_mesh_tools_plugins" for configuration "Release"
set_property(TARGET rviz_mesh_tools_plugins::rviz_mesh_tools_plugins APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rviz_mesh_tools_plugins::rviz_mesh_tools_plugins PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librviz_mesh_tools_plugins.so"
  IMPORTED_SONAME_RELEASE "librviz_mesh_tools_plugins.so"
  )

list(APPEND _cmake_import_check_targets rviz_mesh_tools_plugins::rviz_mesh_tools_plugins )
list(APPEND _cmake_import_check_files_for_rviz_mesh_tools_plugins::rviz_mesh_tools_plugins "${_IMPORT_PREFIX}/lib/librviz_mesh_tools_plugins.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
