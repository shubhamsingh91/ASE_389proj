#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "towr::towr" for configuration "Debug"
set_property(TARGET towr::towr APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(towr::towr PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libtowr.so"
  IMPORTED_SONAME_DEBUG "libtowr.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS towr::towr )
list(APPEND _IMPORT_CHECK_FILES_FOR_towr::towr "${_IMPORT_PREFIX}/lib/libtowr.so" )

# Import target "towr::towr-example" for configuration "Debug"
set_property(TARGET towr::towr-example APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(towr::towr-example PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/towr/towr-example"
  )

list(APPEND _IMPORT_CHECK_TARGETS towr::towr-example )
list(APPEND _IMPORT_CHECK_FILES_FOR_towr::towr-example "${_IMPORT_PREFIX}/lib/towr/towr-example" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
