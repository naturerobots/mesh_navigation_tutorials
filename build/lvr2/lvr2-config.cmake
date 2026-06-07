# -----------------------------------------------------------------
# lvr's cmake configuration
#
# provided options:
# - LVR2_USE_STATIC_LIBS(=OFF) to link the static libraries of lvr
#
# provided interface variables:
# - LVR2_INCLUDE_DIRS
# - LVR2_DEFINITIONS
# - LVR2_LIBRARIES
#
# Do not forget to add_defintions(${LVR2_DEFINITIONS}) as they
# describe the build configuration of liblvr.
#
# -----------------------------------------------------------------

####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was lvr2-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################
include(${CMAKE_CURRENT_LIST_DIR}/lvr2-config-version.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/lvr2-targets.cmake)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/Modules")

# The following block exists only for backwards compatibility 
# and marked as obsolete via the migration guide 25.1.0 -> 25.2.0

#########################
## OBSOLETE START

set(LVR2_INCLUDE_DIRS /home/haris/SharedDrive/Harris/Desktop/ros2_workspaces/naturerobots/mesh_navigation_tutorials/install/lvr2/include)
set(LVR2_DEFINITIONS -DLVR2_USE_EMBREE;-DLVR2_EMBREE_VERSION=4;-DLVR2_USE_OPENCL;-DLVR2_USE_OPENCL_NEW_API;-DLVR2_USE_OPEN_MP)

option(LVR2_USE_STATIC_LIBS OFF)
if(LVR2_USE_STATIC_LIBS)
  set(LVR2_LIBRARY lvr2::lvr2_static)
  set(LVR2_LIBRARIES ${LVR2_LIBRARY})
else()
  set(LVR2_LIBRARY lvr2::lvr2)
  set(LVR2_LIBRARIES ${LVR2_LIBRARY})
endif()

## OBSOLETE END
#########################

include(CMakeFindDependencyMacro)

find_dependency(TBB)
find_dependency(MPI)
find_dependency(TIFF)
find_dependency(GDAL)
find_dependency(OpenCV)

find_dependency(FLANN)
find_dependency(LZ4)
find_dependency(GSL)
find_dependency(Eigen3)
find_dependency(yaml-cpp)
# find_dependency(Boost)
find_package(Boost COMPONENTS program_options;filesystem;thread;serialization;timer;iostreams;date_time;mpi REQUIRED QUIET)
# find_dependency(HDF5)
find_package(HDF5 REQUIRED COMPONENTS C CXX HL)
# find_dependency(OpenGL)
find_package(OpenGL COMPONENTS OpenGL)
find_dependency(GLUT)

#####################
##  OPTIONAL DEPS  ##
#####################
if()
  find_dependency(RDB)
endif()

# VTK
if()
    find_dependency(VTK)
endif()

# pcl
if()
  find_dependency(PCL)
endif()

# cgal
if()
  find_dependency(CGAL)
endif()

# libfreenect
if()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBFREENECT REQUIRED libfreenect)
  list(APPEND LVR2_INCLUDE_DIRS ${LIBFREENECT_INCLUDE_DIRS})
endif()

# Embree
if(1)
  set_property(GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS ON)
  find_package(embree 4 REQUIRED)
endif()

set(lvr2_FOUND TRUE)
