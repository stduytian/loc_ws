# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_point2d_loc_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED point2d_loc_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(point2d_loc_FOUND FALSE)
  elseif(NOT point2d_loc_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(point2d_loc_FOUND FALSE)
  endif()
  return()
endif()
set(_point2d_loc_CONFIG_INCLUDED TRUE)

# output package information
if(NOT point2d_loc_FIND_QUIETLY)
  message(STATUS "Found point2d_loc: 0.0.0 (${point2d_loc_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'point2d_loc' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${point2d_loc_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(point2d_loc_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${point2d_loc_DIR}/${_extra}")
endforeach()
