#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "lvr2::lvr2rply_static" for configuration "Release"
set_property(TARGET lvr2::lvr2rply_static APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(lvr2::lvr2rply_static PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblvr2rply_static.a"
  )

list(APPEND _cmake_import_check_targets lvr2::lvr2rply_static )
list(APPEND _cmake_import_check_files_for_lvr2::lvr2rply_static "${_IMPORT_PREFIX}/lib/liblvr2rply_static.a" )

# Import target "lvr2::lvr2rply" for configuration "Release"
set_property(TARGET lvr2::lvr2rply APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(lvr2::lvr2rply PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblvr2rply.so.25.2.3"
  IMPORTED_SONAME_RELEASE "liblvr2rply.so.25"
  )

list(APPEND _cmake_import_check_targets lvr2::lvr2rply )
list(APPEND _cmake_import_check_files_for_lvr2::lvr2rply "${_IMPORT_PREFIX}/lib/liblvr2rply.so.25.2.3" )

# Import target "lvr2::lvr2las_static" for configuration "Release"
set_property(TARGET lvr2::lvr2las_static APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(lvr2::lvr2las_static PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblvr2las_static.a"
  )

list(APPEND _cmake_import_check_targets lvr2::lvr2las_static )
list(APPEND _cmake_import_check_files_for_lvr2::lvr2las_static "${_IMPORT_PREFIX}/lib/liblvr2las_static.a" )

# Import target "lvr2::lvr2las" for configuration "Release"
set_property(TARGET lvr2::lvr2las APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(lvr2::lvr2las PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblvr2las.so.25.2.3"
  IMPORTED_SONAME_RELEASE "liblvr2las.so.25"
  )

list(APPEND _cmake_import_check_targets lvr2::lvr2las )
list(APPEND _cmake_import_check_files_for_lvr2::lvr2las "${_IMPORT_PREFIX}/lib/liblvr2las.so.25.2.3" )

# Import target "lvr2::lvr2_static" for configuration "Release"
set_property(TARGET lvr2::lvr2_static APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(lvr2::lvr2_static PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblvr2_static.a"
  )

list(APPEND _cmake_import_check_targets lvr2::lvr2_static )
list(APPEND _cmake_import_check_files_for_lvr2::lvr2_static "${_IMPORT_PREFIX}/lib/liblvr2_static.a" )

# Import target "lvr2::lvr2" for configuration "Release"
set_property(TARGET lvr2::lvr2 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(lvr2::lvr2 PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/liblvr2.so.25.2.3"
  IMPORTED_SONAME_RELEASE "liblvr2.so.25"
  )

list(APPEND _cmake_import_check_targets lvr2::lvr2 )
list(APPEND _cmake_import_check_files_for_lvr2::lvr2 "${_IMPORT_PREFIX}/lib/liblvr2.so.25.2.3" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
