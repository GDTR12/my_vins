########## MACROS ###########################################################################
#############################################################################################

# Requires CMake > 3.15
if(${CMAKE_VERSION} VERSION_LESS "3.15")
    message(FATAL_ERROR "The 'CMakeDeps' generator only works with CMake >= 3.15")
endif()

if(hwloc_FIND_QUIETLY)
    set(hwloc_MESSAGE_MODE VERBOSE)
else()
    set(hwloc_MESSAGE_MODE STATUS)
endif()

include(${CMAKE_CURRENT_LIST_DIR}/cmakedeps_macros.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/hwlocTargets.cmake)
include(CMakeFindDependencyMacro)

check_build_type_defined()

foreach(_DEPENDENCY ${hwloc_FIND_DEPENDENCY_NAMES} )
    # Check that we have not already called a find_package with the transitive dependency
    if(NOT ${_DEPENDENCY}_FOUND)
        find_dependency(${_DEPENDENCY} REQUIRED ${${_DEPENDENCY}_FIND_MODE})
    endif()
endforeach()

set(hwloc_VERSION_STRING "2.9.3")
set(hwloc_INCLUDE_DIRS ${hwloc_INCLUDE_DIRS_DEBUG} )
set(hwloc_INCLUDE_DIR ${hwloc_INCLUDE_DIRS_DEBUG} )
set(hwloc_LIBRARIES ${hwloc_LIBRARIES_DEBUG} )
set(hwloc_DEFINITIONS ${hwloc_DEFINITIONS_DEBUG} )


# Only the last installed configuration BUILD_MODULES are included to avoid the collision
foreach(_BUILD_MODULE ${hwloc_BUILD_MODULES_PATHS_DEBUG} )
    message(${hwloc_MESSAGE_MODE} "Conan: Including build module from '${_BUILD_MODULE}'")
    include(${_BUILD_MODULE})
endforeach()


