#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "mesh_controller::mesh_controller" for configuration "Release"
set_property(TARGET mesh_controller::mesh_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(mesh_controller::mesh_controller PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libmesh_controller.so"
  IMPORTED_SONAME_RELEASE "libmesh_controller.so"
  )

list(APPEND _cmake_import_check_targets mesh_controller::mesh_controller )
list(APPEND _cmake_import_check_files_for_mesh_controller::mesh_controller "${_IMPORT_PREFIX}/lib/libmesh_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
