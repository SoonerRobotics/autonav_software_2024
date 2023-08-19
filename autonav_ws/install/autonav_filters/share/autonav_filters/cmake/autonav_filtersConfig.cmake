# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_autonav_filters_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED autonav_filters_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(autonav_filters_FOUND FALSE)
  elseif(NOT autonav_filters_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(autonav_filters_FOUND FALSE)
  endif()
  return()
endif()
set(_autonav_filters_CONFIG_INCLUDED TRUE)

# output package information
if(NOT autonav_filters_FIND_QUIETLY)
  message(STATUS "Found autonav_filters: 0.0.0 (${autonav_filters_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'autonav_filters' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${autonav_filters_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(autonav_filters_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${autonav_filters_DIR}/${_extra}")
endforeach()
