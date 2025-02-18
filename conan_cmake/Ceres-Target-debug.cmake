# Avoid multiple calls to find_package to append duplicated properties to the targets
include_guard()########### VARIABLES #######################################################################
#############################################################################################
set(ceres-solver_FRAMEWORKS_FOUND_DEBUG "") # Will be filled later
conan_find_apple_frameworks(ceres-solver_FRAMEWORKS_FOUND_DEBUG "${ceres-solver_FRAMEWORKS_DEBUG}" "${ceres-solver_FRAMEWORK_DIRS_DEBUG}")

set(ceres-solver_LIBRARIES_TARGETS "") # Will be filled later


######## Create an interface target to contain all the dependencies (frameworks, system and conan deps)
if(NOT TARGET ceres-solver_DEPS_TARGET)
    add_library(ceres-solver_DEPS_TARGET INTERFACE IMPORTED)
endif()

set_property(TARGET ceres-solver_DEPS_TARGET
             APPEND PROPERTY INTERFACE_LINK_LIBRARIES
             $<$<CONFIG:Debug>:${ceres-solver_FRAMEWORKS_FOUND_DEBUG}>
             $<$<CONFIG:Debug>:${ceres-solver_SYSTEM_LIBS_DEBUG}>
             $<$<CONFIG:Debug>:Eigen3::Eigen>)

####### Find the libraries declared in cpp_info.libs, create an IMPORTED target for each one and link the
####### ceres-solver_DEPS_TARGET to all of them
conan_package_library_targets("${ceres-solver_LIBS_DEBUG}"    # libraries
                              "${ceres-solver_LIB_DIRS_DEBUG}" # package_libdir
                              "${ceres-solver_BIN_DIRS_DEBUG}" # package_bindir
                              "${ceres-solver_LIBRARY_TYPE_DEBUG}"
                              "${ceres-solver_IS_HOST_WINDOWS_DEBUG}"
                              ceres-solver_DEPS_TARGET
                              ceres-solver_LIBRARIES_TARGETS  # out_libraries_targets
                              "_DEBUG"
                              "ceres-solver"    # package_name
                              "${ceres-solver_NO_SONAME_MODE_DEBUG}")  # soname

# FIXME: What is the result of this for multi-config? All configs adding themselves to path?
set(CMAKE_MODULE_PATH ${ceres-solver_BUILD_DIRS_DEBUG} ${CMAKE_MODULE_PATH})

########## COMPONENTS TARGET PROPERTIES Debug ########################################

    ########## COMPONENT Ceres::ceres #############

        set(ceres-solver_Ceres_ceres_FRAMEWORKS_FOUND_DEBUG "")
        conan_find_apple_frameworks(ceres-solver_Ceres_ceres_FRAMEWORKS_FOUND_DEBUG "${ceres-solver_Ceres_ceres_FRAMEWORKS_DEBUG}" "${ceres-solver_Ceres_ceres_FRAMEWORK_DIRS_DEBUG}")

        set(ceres-solver_Ceres_ceres_LIBRARIES_TARGETS "")

        ######## Create an interface target to contain all the dependencies (frameworks, system and conan deps)
        if(NOT TARGET ceres-solver_Ceres_ceres_DEPS_TARGET)
            add_library(ceres-solver_Ceres_ceres_DEPS_TARGET INTERFACE IMPORTED)
        endif()

        set_property(TARGET ceres-solver_Ceres_ceres_DEPS_TARGET
                     APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                     $<$<CONFIG:Debug>:${ceres-solver_Ceres_ceres_FRAMEWORKS_FOUND_DEBUG}>
                     $<$<CONFIG:Debug>:${ceres-solver_Ceres_ceres_SYSTEM_LIBS_DEBUG}>
                     $<$<CONFIG:Debug>:${ceres-solver_Ceres_ceres_DEPENDENCIES_DEBUG}>
                     )

        ####### Find the libraries declared in cpp_info.component["xxx"].libs,
        ####### create an IMPORTED target for each one and link the 'ceres-solver_Ceres_ceres_DEPS_TARGET' to all of them
        conan_package_library_targets("${ceres-solver_Ceres_ceres_LIBS_DEBUG}"
                              "${ceres-solver_Ceres_ceres_LIB_DIRS_DEBUG}"
                              "${ceres-solver_Ceres_ceres_BIN_DIRS_DEBUG}" # package_bindir
                              "${ceres-solver_Ceres_ceres_LIBRARY_TYPE_DEBUG}"
                              "${ceres-solver_Ceres_ceres_IS_HOST_WINDOWS_DEBUG}"
                              ceres-solver_Ceres_ceres_DEPS_TARGET
                              ceres-solver_Ceres_ceres_LIBRARIES_TARGETS
                              "_DEBUG"
                              "ceres-solver_Ceres_ceres"
                              "${ceres-solver_Ceres_ceres_NO_SONAME_MODE_DEBUG}")


        ########## TARGET PROPERTIES #####################################
        set_property(TARGET Ceres::ceres
                     APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                     $<$<CONFIG:Debug>:${ceres-solver_Ceres_ceres_OBJECTS_DEBUG}>
                     $<$<CONFIG:Debug>:${ceres-solver_Ceres_ceres_LIBRARIES_TARGETS}>
                     )

        if("${ceres-solver_Ceres_ceres_LIBS_DEBUG}" STREQUAL "")
            # If the component is not declaring any "cpp_info.components['foo'].libs" the system, frameworks etc are not
            # linked to the imported targets and we need to do it to the global target
            set_property(TARGET Ceres::ceres
                         APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                         ceres-solver_Ceres_ceres_DEPS_TARGET)
        endif()

        set_property(TARGET Ceres::ceres APPEND PROPERTY INTERFACE_LINK_OPTIONS
                     $<$<CONFIG:Debug>:${ceres-solver_Ceres_ceres_LINKER_FLAGS_DEBUG}>)
        set_property(TARGET Ceres::ceres APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                     $<$<CONFIG:Debug>:${ceres-solver_Ceres_ceres_INCLUDE_DIRS_DEBUG}>)
        set_property(TARGET Ceres::ceres APPEND PROPERTY INTERFACE_LINK_DIRECTORIES
                     $<$<CONFIG:Debug>:${ceres-solver_Ceres_ceres_LIB_DIRS_DEBUG}>)
        set_property(TARGET Ceres::ceres APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS
                     $<$<CONFIG:Debug>:${ceres-solver_Ceres_ceres_COMPILE_DEFINITIONS_DEBUG}>)
        set_property(TARGET Ceres::ceres APPEND PROPERTY INTERFACE_COMPILE_OPTIONS
                     $<$<CONFIG:Debug>:${ceres-solver_Ceres_ceres_COMPILE_OPTIONS_DEBUG}>)

    ########## AGGREGATED GLOBAL TARGET WITH THE COMPONENTS #####################
    set_property(TARGET Ceres::ceres APPEND PROPERTY INTERFACE_LINK_LIBRARIES Ceres::ceres)

########## For the modules (FindXXX)
set(ceres-solver_LIBRARIES_DEBUG Ceres::ceres)
