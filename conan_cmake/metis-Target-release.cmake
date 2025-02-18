# Avoid multiple calls to find_package to append duplicated properties to the targets
include_guard()########### VARIABLES #######################################################################
#############################################################################################
set(metis_FRAMEWORKS_FOUND_RELEASE "") # Will be filled later
conan_find_apple_frameworks(metis_FRAMEWORKS_FOUND_RELEASE "${metis_FRAMEWORKS_RELEASE}" "${metis_FRAMEWORK_DIRS_RELEASE}")

set(metis_LIBRARIES_TARGETS "") # Will be filled later


######## Create an interface target to contain all the dependencies (frameworks, system and conan deps)
if(NOT TARGET metis_DEPS_TARGET)
    add_library(metis_DEPS_TARGET INTERFACE IMPORTED)
endif()

set_property(TARGET metis_DEPS_TARGET
             APPEND PROPERTY INTERFACE_LINK_LIBRARIES
             $<$<CONFIG:Release>:${metis_FRAMEWORKS_FOUND_RELEASE}>
             $<$<CONFIG:Release>:${metis_SYSTEM_LIBS_RELEASE}>
             $<$<CONFIG:Release>:gklib::gklib>)

####### Find the libraries declared in cpp_info.libs, create an IMPORTED target for each one and link the
####### metis_DEPS_TARGET to all of them
conan_package_library_targets("${metis_LIBS_RELEASE}"    # libraries
                              "${metis_LIB_DIRS_RELEASE}" # package_libdir
                              "${metis_BIN_DIRS_RELEASE}" # package_bindir
                              "${metis_LIBRARY_TYPE_RELEASE}"
                              "${metis_IS_HOST_WINDOWS_RELEASE}"
                              metis_DEPS_TARGET
                              metis_LIBRARIES_TARGETS  # out_libraries_targets
                              "_RELEASE"
                              "metis"    # package_name
                              "${metis_NO_SONAME_MODE_RELEASE}")  # soname

# FIXME: What is the result of this for multi-config? All configs adding themselves to path?
set(CMAKE_MODULE_PATH ${metis_BUILD_DIRS_RELEASE} ${CMAKE_MODULE_PATH})

########## GLOBAL TARGET PROPERTIES Release ########################################
    set_property(TARGET metis::metis
                 APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                 $<$<CONFIG:Release>:${metis_OBJECTS_RELEASE}>
                 $<$<CONFIG:Release>:${metis_LIBRARIES_TARGETS}>
                 )

    if("${metis_LIBS_RELEASE}" STREQUAL "")
        # If the package is not declaring any "cpp_info.libs" the package deps, system libs,
        # frameworks etc are not linked to the imported targets and we need to do it to the
        # global target
        set_property(TARGET metis::metis
                     APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                     metis_DEPS_TARGET)
    endif()

    set_property(TARGET metis::metis
                 APPEND PROPERTY INTERFACE_LINK_OPTIONS
                 $<$<CONFIG:Release>:${metis_LINKER_FLAGS_RELEASE}>)
    set_property(TARGET metis::metis
                 APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                 $<$<CONFIG:Release>:${metis_INCLUDE_DIRS_RELEASE}>)
    # Necessary to find LINK shared libraries in Linux
    set_property(TARGET metis::metis
                 APPEND PROPERTY INTERFACE_LINK_DIRECTORIES
                 $<$<CONFIG:Release>:${metis_LIB_DIRS_RELEASE}>)
    set_property(TARGET metis::metis
                 APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS
                 $<$<CONFIG:Release>:${metis_COMPILE_DEFINITIONS_RELEASE}>)
    set_property(TARGET metis::metis
                 APPEND PROPERTY INTERFACE_COMPILE_OPTIONS
                 $<$<CONFIG:Release>:${metis_COMPILE_OPTIONS_RELEASE}>)

########## For the modules (FindXXX)
set(metis_LIBRARIES_RELEASE metis::metis)
