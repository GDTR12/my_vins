########## MACROS ###########################################################################
#############################################################################################

# Requires CMake > 3.15
if(${CMAKE_VERSION} VERSION_LESS "3.15")
    message(FATAL_ERROR "The 'CMakeDeps' generator only works with CMake >= 3.15")
endif()

if(Ceres_FIND_QUIETLY)
    set(Ceres_MESSAGE_MODE VERBOSE)
else()
    set(Ceres_MESSAGE_MODE STATUS)
endif()

include(${CMAKE_CURRENT_LIST_DIR}/cmakedeps_macros.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/CeresTargets.cmake)
include(CMakeFindDependencyMacro)

check_build_type_defined()

foreach(_DEPENDENCY ${ceres-solver_FIND_DEPENDENCY_NAMES} )
    # Check that we have not already called a find_package with the transitive dependency
    if(NOT ${_DEPENDENCY}_FOUND)
        find_dependency(${_DEPENDENCY} REQUIRED ${${_DEPENDENCY}_FIND_MODE})
    endif()
endforeach()

set(Ceres_VERSION_STRING "2.2.0")
set(Ceres_INCLUDE_DIRS ${ceres-solver_INCLUDE_DIRS_DEBUG} )
set(Ceres_INCLUDE_DIR ${ceres-solver_INCLUDE_DIRS_DEBUG} )
set(Ceres_LIBRARIES ${ceres-solver_LIBRARIES_DEBUG} )
set(Ceres_DEFINITIONS ${ceres-solver_DEFINITIONS_DEBUG} )


# Only the last installed configuration BUILD_MODULES are included to avoid the collision
foreach(_BUILD_MODULE ${ceres-solver_BUILD_MODULES_PATHS_DEBUG} )
    message(${Ceres_MESSAGE_MODE} "Conan: Including build module from '${_BUILD_MODULE}'")
    include(${_BUILD_MODULE})
endforeach()


