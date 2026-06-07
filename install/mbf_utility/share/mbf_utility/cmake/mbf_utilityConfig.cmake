# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mbf_utility_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mbf_utility_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mbf_utility_FOUND FALSE)
  elseif(NOT mbf_utility_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mbf_utility_FOUND FALSE)
  endif()
  return()
endif()
set(_mbf_utility_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mbf_utility_FIND_QUIETLY)
  message(STATUS "Found mbf_utility: 1.0.5 (${mbf_utility_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mbf_utility' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT mbf_utility_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mbf_utility_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${mbf_utility_DIR}/${_extra}")
endforeach()
