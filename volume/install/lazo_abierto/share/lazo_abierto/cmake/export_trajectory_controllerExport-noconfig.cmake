#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "lazo_abierto::trajectory_controller" for configuration ""
set_property(TARGET lazo_abierto::trajectory_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(lazo_abierto::trajectory_controller PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/lazo_abierto/libtrajectory_controller.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS lazo_abierto::trajectory_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_lazo_abierto::trajectory_controller "${_IMPORT_PREFIX}/lib/lazo_abierto/libtrajectory_controller.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
