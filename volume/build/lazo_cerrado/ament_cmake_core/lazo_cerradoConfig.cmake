# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_lazo_cerrado_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED lazo_cerrado_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(lazo_cerrado_FOUND FALSE)
  elseif(NOT lazo_cerrado_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(lazo_cerrado_FOUND FALSE)
  endif()
  return()
endif()
set(_lazo_cerrado_CONFIG_INCLUDED TRUE)

# output package information
if(NOT lazo_cerrado_FIND_QUIETLY)
  message(STATUS "Found lazo_cerrado: 0.0.0 (${lazo_cerrado_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'lazo_cerrado' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${lazo_cerrado_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(lazo_cerrado_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake")
foreach(_extra ${_extras})
  include("${lazo_cerrado_DIR}/${_extra}")
endforeach()
