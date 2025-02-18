########## MACROS ###########################################################################
#############################################################################################

# Requires CMake > 3.15
if(${CMAKE_VERSION} VERSION_LESS "3.15")
    message(FATAL_ERROR "The 'CMakeDeps' generator only works with CMake >= 3.15")
endif()

if(GTSAM_FIND_QUIETLY)
    set(GTSAM_MESSAGE_MODE VERBOSE)
else()
    set(GTSAM_MESSAGE_MODE STATUS)
endif()

include(${CMAKE_CURRENT_LIST_DIR}/cmakedeps_macros.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/GTSAMTargets.cmake)
include(CMakeFindDependencyMacro)

check_build_type_defined()

foreach(_DEPENDENCY ${gtsam_FIND_DEPENDENCY_NAMES} )
    # Check that we have not already called a find_package with the transitive dependency
    if(NOT ${_DEPENDENCY}_FOUND)
        find_dependency(${_DEPENDENCY} REQUIRED ${${_DEPENDENCY}_FIND_MODE})
    endif()
endforeach()

set(GTSAM_VERSION_STRING "4.2")
set(GTSAM_INCLUDE_DIRS ${gtsam_INCLUDE_DIRS_DEBUG} )
set(GTSAM_INCLUDE_DIR ${gtsam_INCLUDE_DIRS_DEBUG} )
set(GTSAM_LIBRARIES ${gtsam_LIBRARIES_DEBUG} )
set(GTSAM_DEFINITIONS ${gtsam_DEFINITIONS_DEBUG} )


# Only the last installed configuration BUILD_MODULES are included to avoid the collision
foreach(_BUILD_MODULE ${gtsam_BUILD_MODULES_PATHS_DEBUG} )
    message(${GTSAM_MESSAGE_MODE} "Conan: Including build module from '${_BUILD_MODULE}'")
    include(${_BUILD_MODULE})
endforeach()


