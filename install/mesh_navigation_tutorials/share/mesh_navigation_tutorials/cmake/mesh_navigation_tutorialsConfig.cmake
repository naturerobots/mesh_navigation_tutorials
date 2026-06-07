# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_mesh_navigation_tutorials_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED mesh_navigation_tutorials_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(mesh_navigation_tutorials_FOUND FALSE)
  elseif(NOT mesh_navigation_tutorials_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(mesh_navigation_tutorials_FOUND FALSE)
  endif()
  return()
endif()
set(_mesh_navigation_tutorials_CONFIG_INCLUDED TRUE)

# output package information
if(NOT mesh_navigation_tutorials_FIND_QUIETLY)
  message(STATUS "Found mesh_navigation_tutorials: 3.0.0 (${mesh_navigation_tutorials_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'mesh_navigation_tutorials' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT mesh_navigation_tutorials_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(mesh_navigation_tutorials_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${mesh_navigation_tutorials_DIR}/${_extra}")
endforeach()
