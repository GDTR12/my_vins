# Avoid multiple calls to find_package to append duplicated properties to the targets
include_guard()########### VARIABLES #######################################################################
#############################################################################################
set(metis_FRAMEWORKS_FOUND_DEBUG "") # Will be filled later
conan_find_apple_frameworks(metis_FRAMEWORKS_FOUND_DEBUG "${metis_FRAMEWORKS_DEBUG}" "${metis_FRAMEWORK_DIRS_DEBUG}")

set(metis_LIBRARIES_TARGETS "") # Will be filled later


######## Create an interface target to contain all the dependencies (frameworks, system and conan deps)
if(NOT TARGET metis_DEPS_TARGET)
    add_library(metis_DEPS_TARGET INTERFACE IMPORTED)
endif()

set_property(TARGET metis_DEPS_TARGET
             APPEND PROPERTY INTERFACE_LINK_LIBRARIES
             $<$<CONFIG:Debug>:${metis_FRAMEWORKS_FOUND_DEBUG}>
             $<$<CONFIG:Debug>:${metis_SYSTEM_LIBS_DEBUG}>
             $<$<CONFIG:Debug>:gklib::gklib>)

####### Find the libraries declared in cpp_info.libs, create an IMPORTED target for each one and link the
####### metis_DEPS_TARGET to all of them
conan_package_library_targets("${metis_LIBS_DEBUG}"    # libraries
                              "${metis_LIB_DIRS_DEBUG}" # package_libdir
                              "${metis_BIN_DIRS_DEBUG}" # package_bindir
                              "${metis_LIBRARY_TYPE_DEBUG}"
                              "${metis_IS_HOST_WINDOWS_DEBUG}"
                              metis_DEPS_TARGET
                              metis_LIBRARIES_TARGETS  # out_libraries_targets
                              "_DEBUG"
                              "metis"    # package_name
                              "${metis_NO_SONAME_MODE_DEBUG}")  # soname

# FIXME: What is the result of this for multi-config? All configs adding themselves to path?
set(CMAKE_MODULE_PATH ${metis_BUILD_DIRS_DEBUG} ${CMAKE_MODULE_PATH})

########## GLOBAL TARGET PROPERTIES Debug ########################################
    set_property(TARGET metis::metis
                 APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                 $<$<CONFIG:Debug>:${metis_OBJECTS_DEBUG}>
                 $<$<CONFIG:Debug>:${metis_LIBRARIES_TARGETS}>
                 )

    if("${metis_LIBS_DEBUG}" STREQUAL "")
        # If the package is not declaring any "cpp_info.libs" the package deps, system libs,
        # frameworks etc are not linked to the imported targets and we need to do it to the
        # global target
        set_property(TARGET metis::metis
                     APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                     metis_DEPS_TARGET)
    endif()

    set_property(TARGET metis::metis
                 APPEND PROPERTY INTERFACE_LINK_OPTIONS
                 $<$<CONFIG:Debug>:${metis_LINKER_FLAGS_DEBUG}>)
    set_property(TARGET metis::metis
                 APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                 $<$<CONFIG:Debug>:${metis_INCLUDE_DIRS_DEBUG}>)
    # Necessary to find LINK shared libraries in Linux
    set_property(TARGET metis::metis
                 APPEND PROPERTY INTERFACE_LINK_DIRECTORIES
                 $<$<CONFIG:Debug>:${metis_LIB_DIRS_DEBUG}>)
    set_property(TARGET metis::metis
                 APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS
                 $<$<CONFIG:Debug>:${metis_COMPILE_DEFINITIONS_DEBUG}>)
    set_property(TARGET metis::metis
                 APPEND PROPERTY INTERFACE_COMPILE_OPTIONS
                 $<$<CONFIG:Debug>:${metis_COMPILE_OPTIONS_DEBUG}>)

########## For the modules (FindXXX)
set(metis_LIBRARIES_DEBUG metis::metis)
