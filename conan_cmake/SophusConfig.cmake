########## MACROS ###########################################################################
#############################################################################################

# Requires CMake > 3.15
if(${CMAKE_VERSION} VERSION_LESS "3.15")
    message(FATAL_ERROR "The 'CMakeDeps' generator only works with CMake >= 3.15")
endif()

if(Sophus_FIND_QUIETLY)
    set(Sophus_MESSAGE_MODE VERBOSE)
else()
    set(Sophus_MESSAGE_MODE STATUS)
endif()

include(${CMAKE_CURRENT_LIST_DIR}/cmakedeps_macros.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/SophusTargets.cmake)
include(CMakeFindDependencyMacro)

check_build_type_defined()

foreach(_DEPENDENCY ${sophus_FIND_DEPENDENCY_NAMES} )
    # Check that we have not already called a find_package with the transitive dependency
    if(NOT ${_DEPENDENCY}_FOUND)
        find_dependency(${_DEPENDENCY} REQUIRED ${${_DEPENDENCY}_FIND_MODE})
    endif()
endforeach()

set(Sophus_VERSION_STRING "1.22.10")
set(Sophus_INCLUDE_DIRS ${sophus_INCLUDE_DIRS_DEBUG} )
set(Sophus_INCLUDE_DIR ${sophus_INCLUDE_DIRS_DEBUG} )
set(Sophus_LIBRARIES ${sophus_LIBRARIES_DEBUG} )
set(Sophus_DEFINITIONS ${sophus_DEFINITIONS_DEBUG} )


# Only the last installed configuration BUILD_MODULES are included to avoid the collision
foreach(_BUILD_MODULE ${sophus_BUILD_MODULES_PATHS_DEBUG} )
    message(${Sophus_MESSAGE_MODE} "Conan: Including build module from '${_BUILD_MODULE}'")
    include(${_BUILD_MODULE})
endforeach()


