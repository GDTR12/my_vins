########## MACROS ###########################################################################
#############################################################################################

# Requires CMake > 3.15
if(${CMAKE_VERSION} VERSION_LESS "3.15")
    message(FATAL_ERROR "The 'CMakeDeps' generator only works with CMake >= 3.15")
endif()

if(gklib_FIND_QUIETLY)
    set(gklib_MESSAGE_MODE VERBOSE)
else()
    set(gklib_MESSAGE_MODE STATUS)
endif()

include(${CMAKE_CURRENT_LIST_DIR}/cmakedeps_macros.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/gklibTargets.cmake)
include(CMakeFindDependencyMacro)

check_build_type_defined()

foreach(_DEPENDENCY ${gklib_FIND_DEPENDENCY_NAMES} )
    # Check that we have not already called a find_package with the transitive dependency
    if(NOT ${_DEPENDENCY}_FOUND)
        find_dependency(${_DEPENDENCY} REQUIRED ${${_DEPENDENCY}_FIND_MODE})
    endif()
endforeach()

set(gklib_VERSION_STRING "5.1.1")
set(gklib_INCLUDE_DIRS ${gklib_INCLUDE_DIRS_DEBUG} )
set(gklib_INCLUDE_DIR ${gklib_INCLUDE_DIRS_DEBUG} )
set(gklib_LIBRARIES ${gklib_LIBRARIES_DEBUG} )
set(gklib_DEFINITIONS ${gklib_DEFINITIONS_DEBUG} )


# Only the last installed configuration BUILD_MODULES are included to avoid the collision
foreach(_BUILD_MODULE ${gklib_BUILD_MODULES_PATHS_DEBUG} )
    message(${gklib_MESSAGE_MODE} "Conan: Including build module from '${_BUILD_MODULE}'")
    include(${_BUILD_MODULE})
endforeach()


