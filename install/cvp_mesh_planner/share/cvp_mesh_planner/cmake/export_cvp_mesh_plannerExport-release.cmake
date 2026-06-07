#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "cvp_mesh_planner::cvp_mesh_planner" for configuration "Release"
set_property(TARGET cvp_mesh_planner::cvp_mesh_planner APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(cvp_mesh_planner::cvp_mesh_planner PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcvp_mesh_planner.so"
  IMPORTED_SONAME_RELEASE "libcvp_mesh_planner.so"
  )

list(APPEND _cmake_import_check_targets cvp_mesh_planner::cvp_mesh_planner )
list(APPEND _cmake_import_check_files_for_cvp_mesh_planner::cvp_mesh_planner "${_IMPORT_PREFIX}/lib/libcvp_mesh_planner.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
