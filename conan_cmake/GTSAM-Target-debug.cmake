# Avoid multiple calls to find_package to append duplicated properties to the targets
include_guard()########### VARIABLES #######################################################################
#############################################################################################
set(gtsam_FRAMEWORKS_FOUND_DEBUG "") # Will be filled later
conan_find_apple_frameworks(gtsam_FRAMEWORKS_FOUND_DEBUG "${gtsam_FRAMEWORKS_DEBUG}" "${gtsam_FRAMEWORK_DIRS_DEBUG}")

set(gtsam_LIBRARIES_TARGETS "") # Will be filled later


######## Create an interface target to contain all the dependencies (frameworks, system and conan deps)
if(NOT TARGET gtsam_DEPS_TARGET)
    add_library(gtsam_DEPS_TARGET INTERFACE IMPORTED)
endif()

set_property(TARGET gtsam_DEPS_TARGET
             APPEND PROPERTY INTERFACE_LINK_LIBRARIES
             $<$<CONFIG:Debug>:${gtsam_FRAMEWORKS_FOUND_DEBUG}>
             $<$<CONFIG:Debug>:${gtsam_SYSTEM_LIBS_DEBUG}>
             $<$<CONFIG:Debug>:Boost::chrono;Boost::date_time;Boost::filesystem;Boost::program_options;Boost::regex;Boost::serialization;Boost::system;Boost::thread;Boost::timer;Eigen3::Eigen;onetbb::onetbb;metis::metis;gtsam;boost::boost>)

####### Find the libraries declared in cpp_info.libs, create an IMPORTED target for each one and link the
####### gtsam_DEPS_TARGET to all of them
conan_package_library_targets("${gtsam_LIBS_DEBUG}"    # libraries
                              "${gtsam_LIB_DIRS_DEBUG}" # package_libdir
                              "${gtsam_BIN_DIRS_DEBUG}" # package_bindir
                              "${gtsam_LIBRARY_TYPE_DEBUG}"
                              "${gtsam_IS_HOST_WINDOWS_DEBUG}"
                              gtsam_DEPS_TARGET
                              gtsam_LIBRARIES_TARGETS  # out_libraries_targets
                              "_DEBUG"
                              "gtsam"    # package_name
                              "${gtsam_NO_SONAME_MODE_DEBUG}")  # soname

# FIXME: What is the result of this for multi-config? All configs adding themselves to path?
set(CMAKE_MODULE_PATH ${gtsam_BUILD_DIRS_DEBUG} ${CMAKE_MODULE_PATH})

########## COMPONENTS TARGET PROPERTIES Debug ########################################

    ########## COMPONENT gtsam_unstable #############

        set(gtsam_gtsam_unstable_FRAMEWORKS_FOUND_DEBUG "")
        conan_find_apple_frameworks(gtsam_gtsam_unstable_FRAMEWORKS_FOUND_DEBUG "${gtsam_gtsam_unstable_FRAMEWORKS_DEBUG}" "${gtsam_gtsam_unstable_FRAMEWORK_DIRS_DEBUG}")

        set(gtsam_gtsam_unstable_LIBRARIES_TARGETS "")

        ######## Create an interface target to contain all the dependencies (frameworks, system and conan deps)
        if(NOT TARGET gtsam_gtsam_unstable_DEPS_TARGET)
            add_library(gtsam_gtsam_unstable_DEPS_TARGET INTERFACE IMPORTED)
        endif()

        set_property(TARGET gtsam_gtsam_unstable_DEPS_TARGET
                     APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                     $<$<CONFIG:Debug>:${gtsam_gtsam_unstable_FRAMEWORKS_FOUND_DEBUG}>
                     $<$<CONFIG:Debug>:${gtsam_gtsam_unstable_SYSTEM_LIBS_DEBUG}>
                     $<$<CONFIG:Debug>:${gtsam_gtsam_unstable_DEPENDENCIES_DEBUG}>
                     )

        ####### Find the libraries declared in cpp_info.component["xxx"].libs,
        ####### create an IMPORTED target for each one and link the 'gtsam_gtsam_unstable_DEPS_TARGET' to all of them
        conan_package_library_targets("${gtsam_gtsam_unstable_LIBS_DEBUG}"
                              "${gtsam_gtsam_unstable_LIB_DIRS_DEBUG}"
                              "${gtsam_gtsam_unstable_BIN_DIRS_DEBUG}" # package_bindir
                              "${gtsam_gtsam_unstable_LIBRARY_TYPE_DEBUG}"
                              "${gtsam_gtsam_unstable_IS_HOST_WINDOWS_DEBUG}"
                              gtsam_gtsam_unstable_DEPS_TARGET
                              gtsam_gtsam_unstable_LIBRARIES_TARGETS
                              "_DEBUG"
                              "gtsam_gtsam_unstable"
                              "${gtsam_gtsam_unstable_NO_SONAME_MODE_DEBUG}")


        ########## TARGET PROPERTIES #####################################
        set_property(TARGET gtsam_unstable
                     APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                     $<$<CONFIG:Debug>:${gtsam_gtsam_unstable_OBJECTS_DEBUG}>
                     $<$<CONFIG:Debug>:${gtsam_gtsam_unstable_LIBRARIES_TARGETS}>
                     )

        if("${gtsam_gtsam_unstable_LIBS_DEBUG}" STREQUAL "")
            # If the component is not declaring any "cpp_info.components['foo'].libs" the system, frameworks etc are not
            # linked to the imported targets and we need to do it to the global target
            set_property(TARGET gtsam_unstable
                         APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                         gtsam_gtsam_unstable_DEPS_TARGET)
        endif()

        set_property(TARGET gtsam_unstable APPEND PROPERTY INTERFACE_LINK_OPTIONS
                     $<$<CONFIG:Debug>:${gtsam_gtsam_unstable_LINKER_FLAGS_DEBUG}>)
        set_property(TARGET gtsam_unstable APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                     $<$<CONFIG:Debug>:${gtsam_gtsam_unstable_INCLUDE_DIRS_DEBUG}>)
        set_property(TARGET gtsam_unstable APPEND PROPERTY INTERFACE_LINK_DIRECTORIES
                     $<$<CONFIG:Debug>:${gtsam_gtsam_unstable_LIB_DIRS_DEBUG}>)
        set_property(TARGET gtsam_unstable APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS
                     $<$<CONFIG:Debug>:${gtsam_gtsam_unstable_COMPILE_DEFINITIONS_DEBUG}>)
        set_property(TARGET gtsam_unstable APPEND PROPERTY INTERFACE_COMPILE_OPTIONS
                     $<$<CONFIG:Debug>:${gtsam_gtsam_unstable_COMPILE_OPTIONS_DEBUG}>)

    ########## COMPONENT CppUnitLite #############

        set(gtsam_CppUnitLite_FRAMEWORKS_FOUND_DEBUG "")
        conan_find_apple_frameworks(gtsam_CppUnitLite_FRAMEWORKS_FOUND_DEBUG "${gtsam_CppUnitLite_FRAMEWORKS_DEBUG}" "${gtsam_CppUnitLite_FRAMEWORK_DIRS_DEBUG}")

        set(gtsam_CppUnitLite_LIBRARIES_TARGETS "")

        ######## Create an interface target to contain all the dependencies (frameworks, system and conan deps)
        if(NOT TARGET gtsam_CppUnitLite_DEPS_TARGET)
            add_library(gtsam_CppUnitLite_DEPS_TARGET INTERFACE IMPORTED)
        endif()

        set_property(TARGET gtsam_CppUnitLite_DEPS_TARGET
                     APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                     $<$<CONFIG:Debug>:${gtsam_CppUnitLite_FRAMEWORKS_FOUND_DEBUG}>
                     $<$<CONFIG:Debug>:${gtsam_CppUnitLite_SYSTEM_LIBS_DEBUG}>
                     $<$<CONFIG:Debug>:${gtsam_CppUnitLite_DEPENDENCIES_DEBUG}>
                     )

        ####### Find the libraries declared in cpp_info.component["xxx"].libs,
        ####### create an IMPORTED target for each one and link the 'gtsam_CppUnitLite_DEPS_TARGET' to all of them
        conan_package_library_targets("${gtsam_CppUnitLite_LIBS_DEBUG}"
                              "${gtsam_CppUnitLite_LIB_DIRS_DEBUG}"
                              "${gtsam_CppUnitLite_BIN_DIRS_DEBUG}" # package_bindir
                              "${gtsam_CppUnitLite_LIBRARY_TYPE_DEBUG}"
                              "${gtsam_CppUnitLite_IS_HOST_WINDOWS_DEBUG}"
                              gtsam_CppUnitLite_DEPS_TARGET
                              gtsam_CppUnitLite_LIBRARIES_TARGETS
                              "_DEBUG"
                              "gtsam_CppUnitLite"
                              "${gtsam_CppUnitLite_NO_SONAME_MODE_DEBUG}")


        ########## TARGET PROPERTIES #####################################
        set_property(TARGET CppUnitLite
                     APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                     $<$<CONFIG:Debug>:${gtsam_CppUnitLite_OBJECTS_DEBUG}>
                     $<$<CONFIG:Debug>:${gtsam_CppUnitLite_LIBRARIES_TARGETS}>
                     )

        if("${gtsam_CppUnitLite_LIBS_DEBUG}" STREQUAL "")
            # If the component is not declaring any "cpp_info.components['foo'].libs" the system, frameworks etc are not
            # linked to the imported targets and we need to do it to the global target
            set_property(TARGET CppUnitLite
                         APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                         gtsam_CppUnitLite_DEPS_TARGET)
        endif()

        set_property(TARGET CppUnitLite APPEND PROPERTY INTERFACE_LINK_OPTIONS
                     $<$<CONFIG:Debug>:${gtsam_CppUnitLite_LINKER_FLAGS_DEBUG}>)
        set_property(TARGET CppUnitLite APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                     $<$<CONFIG:Debug>:${gtsam_CppUnitLite_INCLUDE_DIRS_DEBUG}>)
        set_property(TARGET CppUnitLite APPEND PROPERTY INTERFACE_LINK_DIRECTORIES
                     $<$<CONFIG:Debug>:${gtsam_CppUnitLite_LIB_DIRS_DEBUG}>)
        set_property(TARGET CppUnitLite APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS
                     $<$<CONFIG:Debug>:${gtsam_CppUnitLite_COMPILE_DEFINITIONS_DEBUG}>)
        set_property(TARGET CppUnitLite APPEND PROPERTY INTERFACE_COMPILE_OPTIONS
                     $<$<CONFIG:Debug>:${gtsam_CppUnitLite_COMPILE_OPTIONS_DEBUG}>)

    ########## COMPONENT gtsam #############

        set(gtsam_gtsam_FRAMEWORKS_FOUND_DEBUG "")
        conan_find_apple_frameworks(gtsam_gtsam_FRAMEWORKS_FOUND_DEBUG "${gtsam_gtsam_FRAMEWORKS_DEBUG}" "${gtsam_gtsam_FRAMEWORK_DIRS_DEBUG}")

        set(gtsam_gtsam_LIBRARIES_TARGETS "")

        ######## Create an interface target to contain all the dependencies (frameworks, system and conan deps)
        if(NOT TARGET gtsam_gtsam_DEPS_TARGET)
            add_library(gtsam_gtsam_DEPS_TARGET INTERFACE IMPORTED)
        endif()

        set_property(TARGET gtsam_gtsam_DEPS_TARGET
                     APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                     $<$<CONFIG:Debug>:${gtsam_gtsam_FRAMEWORKS_FOUND_DEBUG}>
                     $<$<CONFIG:Debug>:${gtsam_gtsam_SYSTEM_LIBS_DEBUG}>
                     $<$<CONFIG:Debug>:${gtsam_gtsam_DEPENDENCIES_DEBUG}>
                     )

        ####### Find the libraries declared in cpp_info.component["xxx"].libs,
        ####### create an IMPORTED target for each one and link the 'gtsam_gtsam_DEPS_TARGET' to all of them
        conan_package_library_targets("${gtsam_gtsam_LIBS_DEBUG}"
                              "${gtsam_gtsam_LIB_DIRS_DEBUG}"
                              "${gtsam_gtsam_BIN_DIRS_DEBUG}" # package_bindir
                              "${gtsam_gtsam_LIBRARY_TYPE_DEBUG}"
                              "${gtsam_gtsam_IS_HOST_WINDOWS_DEBUG}"
                              gtsam_gtsam_DEPS_TARGET
                              gtsam_gtsam_LIBRARIES_TARGETS
                              "_DEBUG"
                              "gtsam_gtsam"
                              "${gtsam_gtsam_NO_SONAME_MODE_DEBUG}")


        ########## TARGET PROPERTIES #####################################
        set_property(TARGET gtsam
                     APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                     $<$<CONFIG:Debug>:${gtsam_gtsam_OBJECTS_DEBUG}>
                     $<$<CONFIG:Debug>:${gtsam_gtsam_LIBRARIES_TARGETS}>
                     )

        if("${gtsam_gtsam_LIBS_DEBUG}" STREQUAL "")
            # If the component is not declaring any "cpp_info.components['foo'].libs" the system, frameworks etc are not
            # linked to the imported targets and we need to do it to the global target
            set_property(TARGET gtsam
                         APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                         gtsam_gtsam_DEPS_TARGET)
        endif()

        set_property(TARGET gtsam APPEND PROPERTY INTERFACE_LINK_OPTIONS
                     $<$<CONFIG:Debug>:${gtsam_gtsam_LINKER_FLAGS_DEBUG}>)
        set_property(TARGET gtsam APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                     $<$<CONFIG:Debug>:${gtsam_gtsam_INCLUDE_DIRS_DEBUG}>)
        set_property(TARGET gtsam APPEND PROPERTY INTERFACE_LINK_DIRECTORIES
                     $<$<CONFIG:Debug>:${gtsam_gtsam_LIB_DIRS_DEBUG}>)
        set_property(TARGET gtsam APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS
                     $<$<CONFIG:Debug>:${gtsam_gtsam_COMPILE_DEFINITIONS_DEBUG}>)
        set_property(TARGET gtsam APPEND PROPERTY INTERFACE_COMPILE_OPTIONS
                     $<$<CONFIG:Debug>:${gtsam_gtsam_COMPILE_OPTIONS_DEBUG}>)

    ########## AGGREGATED GLOBAL TARGET WITH THE COMPONENTS #####################
    set_property(TARGET gtsam::gtsam APPEND PROPERTY INTERFACE_LINK_LIBRARIES gtsam_unstable)
    set_property(TARGET gtsam::gtsam APPEND PROPERTY INTERFACE_LINK_LIBRARIES CppUnitLite)
    set_property(TARGET gtsam::gtsam APPEND PROPERTY INTERFACE_LINK_LIBRARIES gtsam)

########## For the modules (FindXXX)
set(gtsam_LIBRARIES_DEBUG gtsam::gtsam)
