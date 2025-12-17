# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_modelo_diferencial_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED modelo_diferencial_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(modelo_diferencial_FOUND FALSE)
  elseif(NOT modelo_diferencial_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(modelo_diferencial_FOUND FALSE)
  endif()
  return()
endif()
set(_modelo_diferencial_CONFIG_INCLUDED TRUE)

# output package information
if(NOT modelo_diferencial_FIND_QUIETLY)
  message(STATUS "Found modelo_diferencial: 0.1.0 (${modelo_diferencial_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'modelo_diferencial' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${modelo_diferencial_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(modelo_diferencial_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${modelo_diferencial_DIR}/${_extra}")
endforeach()
