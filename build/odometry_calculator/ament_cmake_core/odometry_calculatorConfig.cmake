# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_odometry_calculator_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED odometry_calculator_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(odometry_calculator_FOUND FALSE)
  elseif(NOT odometry_calculator_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(odometry_calculator_FOUND FALSE)
  endif()
  return()
endif()
set(_odometry_calculator_CONFIG_INCLUDED TRUE)

# output package information
if(NOT odometry_calculator_FIND_QUIETLY)
  message(STATUS "Found odometry_calculator: 0.0.0 (${odometry_calculator_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'odometry_calculator' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${odometry_calculator_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(odometry_calculator_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${odometry_calculator_DIR}/${_extra}")
endforeach()
