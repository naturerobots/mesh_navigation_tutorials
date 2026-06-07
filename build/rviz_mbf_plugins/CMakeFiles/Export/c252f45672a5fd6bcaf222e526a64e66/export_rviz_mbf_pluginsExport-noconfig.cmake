#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rviz_mbf_plugins::rviz_mbf_plugins" for configuration ""
set_property(TARGET rviz_mbf_plugins::rviz_mbf_plugins APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rviz_mbf_plugins::rviz_mbf_plugins PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librviz_mbf_plugins.so"
  IMPORTED_SONAME_NOCONFIG "librviz_mbf_plugins.so"
  )

list(APPEND _cmake_import_check_targets rviz_mbf_plugins::rviz_mbf_plugins )
list(APPEND _cmake_import_check_files_for_rviz_mbf_plugins::rviz_mbf_plugins "${_IMPORT_PREFIX}/lib/librviz_mbf_plugins.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
