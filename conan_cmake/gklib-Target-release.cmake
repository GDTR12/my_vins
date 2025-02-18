# Avoid multiple calls to find_package to append duplicated properties to the targets
include_guard()########### VARIABLES #######################################################################
#############################################################################################
set(gklib_FRAMEWORKS_FOUND_RELEASE "") # Will be filled later
conan_find_apple_frameworks(gklib_FRAMEWORKS_FOUND_RELEASE "${gklib_FRAMEWORKS_RELEASE}" "${gklib_FRAMEWORK_DIRS_RELEASE}")

set(gklib_LIBRARIES_TARGETS "") # Will be filled later


######## Create an interface target to contain all the dependencies (frameworks, system and conan deps)
if(NOT TARGET gklib_DEPS_TARGET)
    add_library(gklib_DEPS_TARGET INTERFACE IMPORTED)
endif()

set_property(TARGET gklib_DEPS_TARGET
             APPEND PROPERTY INTERFACE_LINK_LIBRARIES
             $<$<CONFIG:Release>:${gklib_FRAMEWORKS_FOUND_RELEASE}>
             $<$<CONFIG:Release>:${gklib_SYSTEM_LIBS_RELEASE}>
             $<$<CONFIG:Release>:>)

####### Find the libraries declared in cpp_info.libs, create an IMPORTED target for each one and link the
####### gklib_DEPS_TARGET to all of them
conan_package_library_targets("${gklib_LIBS_RELEASE}"    # libraries
                              "${gklib_LIB_DIRS_RELEASE}" # package_libdir
                              "${gklib_BIN_DIRS_RELEASE}" # package_bindir
                              "${gklib_LIBRARY_TYPE_RELEASE}"
                              "${gklib_IS_HOST_WINDOWS_RELEASE}"
                              gklib_DEPS_TARGET
                              gklib_LIBRARIES_TARGETS  # out_libraries_targets
                              "_RELEASE"
                              "gklib"    # package_name
                              "${gklib_NO_SONAME_MODE_RELEASE}")  # soname

# FIXME: What is the result of this for multi-config? All configs adding themselves to path?
set(CMAKE_MODULE_PATH ${gklib_BUILD_DIRS_RELEASE} ${CMAKE_MODULE_PATH})

########## GLOBAL TARGET PROPERTIES Release ########################################
    set_property(TARGET gklib::gklib
                 APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                 $<$<CONFIG:Release>:${gklib_OBJECTS_RELEASE}>
                 $<$<CONFIG:Release>:${gklib_LIBRARIES_TARGETS}>
                 )

    if("${gklib_LIBS_RELEASE}" STREQUAL "")
        # If the package is not declaring any "cpp_info.libs" the package deps, system libs,
        # frameworks etc are not linked to the imported targets and we need to do it to the
        # global target
        set_property(TARGET gklib::gklib
                     APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                     gklib_DEPS_TARGET)
    endif()

    set_property(TARGET gklib::gklib
                 APPEND PROPERTY INTERFACE_LINK_OPTIONS
                 $<$<CONFIG:Release>:${gklib_LINKER_FLAGS_RELEASE}>)
    set_property(TARGET gklib::gklib
                 APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                 $<$<CONFIG:Release>:${gklib_INCLUDE_DIRS_RELEASE}>)
    # Necessary to find LINK shared libraries in Linux
    set_property(TARGET gklib::gklib
                 APPEND PROPERTY INTERFACE_LINK_DIRECTORIES
                 $<$<CONFIG:Release>:${gklib_LIB_DIRS_RELEASE}>)
    set_property(TARGET gklib::gklib
                 APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS
                 $<$<CONFIG:Release>:${gklib_COMPILE_DEFINITIONS_RELEASE}>)
    set_property(TARGET gklib::gklib
                 APPEND PROPERTY INTERFACE_COMPILE_OPTIONS
                 $<$<CONFIG:Release>:${gklib_COMPILE_OPTIONS_RELEASE}>)

########## For the modules (FindXXX)
set(gklib_LIBRARIES_RELEASE gklib::gklib)
