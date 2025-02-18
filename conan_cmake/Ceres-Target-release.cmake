# Avoid multiple calls to find_package to append duplicated properties to the targets
include_guard()########### VARIABLES #######################################################################
#############################################################################################
set(ceres-solver_FRAMEWORKS_FOUND_RELEASE "") # Will be filled later
conan_find_apple_frameworks(ceres-solver_FRAMEWORKS_FOUND_RELEASE "${ceres-solver_FRAMEWORKS_RELEASE}" "${ceres-solver_FRAMEWORK_DIRS_RELEASE}")

set(ceres-solver_LIBRARIES_TARGETS "") # Will be filled later


######## Create an interface target to contain all the dependencies (frameworks, system and conan deps)
if(NOT TARGET ceres-solver_DEPS_TARGET)
    add_library(ceres-solver_DEPS_TARGET INTERFACE IMPORTED)
endif()

set_property(TARGET ceres-solver_DEPS_TARGET
             APPEND PROPERTY INTERFACE_LINK_LIBRARIES
             $<$<CONFIG:Release>:${ceres-solver_FRAMEWORKS_FOUND_RELEASE}>
             $<$<CONFIG:Release>:${ceres-solver_SYSTEM_LIBS_RELEASE}>
             $<$<CONFIG:Release>:Eigen3::Eigen>)

####### Find the libraries declared in cpp_info.libs, create an IMPORTED target for each one and link the
####### ceres-solver_DEPS_TARGET to all of them
conan_package_library_targets("${ceres-solver_LIBS_RELEASE}"    # libraries
                              "${ceres-solver_LIB_DIRS_RELEASE}" # package_libdir
                              "${ceres-solver_BIN_DIRS_RELEASE}" # package_bindir
                              "${ceres-solver_LIBRARY_TYPE_RELEASE}"
                              "${ceres-solver_IS_HOST_WINDOWS_RELEASE}"
                              ceres-solver_DEPS_TARGET
                              ceres-solver_LIBRARIES_TARGETS  # out_libraries_targets
                              "_RELEASE"
                              "ceres-solver"    # package_name
                              "${ceres-solver_NO_SONAME_MODE_RELEASE}")  # soname

# FIXME: What is the result of this for multi-config? All configs adding themselves to path?
set(CMAKE_MODULE_PATH ${ceres-solver_BUILD_DIRS_RELEASE} ${CMAKE_MODULE_PATH})

########## COMPONENTS TARGET PROPERTIES Release ########################################

    ########## COMPONENT Ceres::ceres #############

        set(ceres-solver_Ceres_ceres_FRAMEWORKS_FOUND_RELEASE "")
        conan_find_apple_frameworks(ceres-solver_Ceres_ceres_FRAMEWORKS_FOUND_RELEASE "${ceres-solver_Ceres_ceres_FRAMEWORKS_RELEASE}" "${ceres-solver_Ceres_ceres_FRAMEWORK_DIRS_RELEASE}")

        set(ceres-solver_Ceres_ceres_LIBRARIES_TARGETS "")

        ######## Create an interface target to contain all the dependencies (frameworks, system and conan deps)
        if(NOT TARGET ceres-solver_Ceres_ceres_DEPS_TARGET)
            add_library(ceres-solver_Ceres_ceres_DEPS_TARGET INTERFACE IMPORTED)
        endif()

        set_property(TARGET ceres-solver_Ceres_ceres_DEPS_TARGET
                     APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                     $<$<CONFIG:Release>:${ceres-solver_Ceres_ceres_FRAMEWORKS_FOUND_RELEASE}>
                     $<$<CONFIG:Release>:${ceres-solver_Ceres_ceres_SYSTEM_LIBS_RELEASE}>
                     $<$<CONFIG:Release>:${ceres-solver_Ceres_ceres_DEPENDENCIES_RELEASE}>
                     )

        ####### Find the libraries declared in cpp_info.component["xxx"].libs,
        ####### create an IMPORTED target for each one and link the 'ceres-solver_Ceres_ceres_DEPS_TARGET' to all of them
        conan_package_library_targets("${ceres-solver_Ceres_ceres_LIBS_RELEASE}"
                              "${ceres-solver_Ceres_ceres_LIB_DIRS_RELEASE}"
                              "${ceres-solver_Ceres_ceres_BIN_DIRS_RELEASE}" # package_bindir
                              "${ceres-solver_Ceres_ceres_LIBRARY_TYPE_RELEASE}"
                              "${ceres-solver_Ceres_ceres_IS_HOST_WINDOWS_RELEASE}"
                              ceres-solver_Ceres_ceres_DEPS_TARGET
                              ceres-solver_Ceres_ceres_LIBRARIES_TARGETS
                              "_RELEASE"
                              "ceres-solver_Ceres_ceres"
                              "${ceres-solver_Ceres_ceres_NO_SONAME_MODE_RELEASE}")


        ########## TARGET PROPERTIES #####################################
        set_property(TARGET Ceres::ceres
                     APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                     $<$<CONFIG:Release>:${ceres-solver_Ceres_ceres_OBJECTS_RELEASE}>
                     $<$<CONFIG:Release>:${ceres-solver_Ceres_ceres_LIBRARIES_TARGETS}>
                     )

        if("${ceres-solver_Ceres_ceres_LIBS_RELEASE}" STREQUAL "")
            # If the component is not declaring any "cpp_info.components['foo'].libs" the system, frameworks etc are not
            # linked to the imported targets and we need to do it to the global target
            set_property(TARGET Ceres::ceres
                         APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                         ceres-solver_Ceres_ceres_DEPS_TARGET)
        endif()

        set_property(TARGET Ceres::ceres APPEND PROPERTY INTERFACE_LINK_OPTIONS
                     $<$<CONFIG:Release>:${ceres-solver_Ceres_ceres_LINKER_FLAGS_RELEASE}>)
        set_property(TARGET Ceres::ceres APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                     $<$<CONFIG:Release>:${ceres-solver_Ceres_ceres_INCLUDE_DIRS_RELEASE}>)
        set_property(TARGET Ceres::ceres APPEND PROPERTY INTERFACE_LINK_DIRECTORIES
                     $<$<CONFIG:Release>:${ceres-solver_Ceres_ceres_LIB_DIRS_RELEASE}>)
        set_property(TARGET Ceres::ceres APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS
                     $<$<CONFIG:Release>:${ceres-solver_Ceres_ceres_COMPILE_DEFINITIONS_RELEASE}>)
        set_property(TARGET Ceres::ceres APPEND PROPERTY INTERFACE_COMPILE_OPTIONS
                     $<$<CONFIG:Release>:${ceres-solver_Ceres_ceres_COMPILE_OPTIONS_RELEASE}>)

    ########## AGGREGATED GLOBAL TARGET WITH THE COMPONENTS #####################
    set_property(TARGET Ceres::ceres APPEND PROPERTY INTERFACE_LINK_LIBRARIES Ceres::ceres)

########## For the modules (FindXXX)
set(ceres-solver_LIBRARIES_RELEASE Ceres::ceres)
