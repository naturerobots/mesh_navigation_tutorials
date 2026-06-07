#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "mesh_msgs_conversions::mesh_msgs_conversions" for configuration "Release"
set_property(TARGET mesh_msgs_conversions::mesh_msgs_conversions APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(mesh_msgs_conversions::mesh_msgs_conversions PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libmesh_msgs_conversions.a"
  )

list(APPEND _cmake_import_check_targets mesh_msgs_conversions::mesh_msgs_conversions )
list(APPEND _cmake_import_check_files_for_mesh_msgs_conversions::mesh_msgs_conversions "${_IMPORT_PREFIX}/lib/libmesh_msgs_conversions.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
