# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_Miau_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED Miau_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(Miau_FOUND FALSE)
  elseif(NOT Miau_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(Miau_FOUND FALSE)
  endif()
  return()
endif()
set(_Miau_CONFIG_INCLUDED TRUE)

# output package information
if(NOT Miau_FIND_QUIETLY)
  message(STATUS "Found Miau: 0.0.0 (${Miau_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'Miau' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT Miau_DEPRECATED_QUIET)
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(Miau_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${Miau_DIR}/${_extra}")
endforeach()
