# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_nexus_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED nexus_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(nexus_FOUND FALSE)
  elseif(NOT nexus_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(nexus_FOUND FALSE)
  endif()
  return()
endif()
set(_nexus_CONFIG_INCLUDED TRUE)

# output package information
if(NOT nexus_FIND_QUIETLY)
  message(STATUS "Found nexus: 0.0.0 (${nexus_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'nexus' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${nexus_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(nexus_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${nexus_DIR}/${_extra}")
endforeach()
