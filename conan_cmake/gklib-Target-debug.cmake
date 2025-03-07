# Avoid multiple calls to find_package to append duplicated properties to the targets
include_guard()########### VARIABLES #######################################################################
#############################################################################################
set(gklib_FRAMEWORKS_FOUND_DEBUG "") # Will be filled later
conan_find_apple_frameworks(gklib_FRAMEWORKS_FOUND_DEBUG "${gklib_FRAMEWORKS_DEBUG}" "${gklib_FRAMEWORK_DIRS_DEBUG}")

set(gklib_LIBRARIES_TARGETS "") # Will be filled later


######## Create an interface target to contain all the dependencies (frameworks, system and conan deps)
if(NOT TARGET gklib_DEPS_TARGET)
    add_library(gklib_DEPS_TARGET INTERFACE IMPORTED)
endif()

set_property(TARGET gklib_DEPS_TARGET
             APPEND PROPERTY INTERFACE_LINK_LIBRARIES
             $<$<CONFIG:Debug>:${gklib_FRAMEWORKS_FOUND_DEBUG}>
             $<$<CONFIG:Debug>:${gklib_SYSTEM_LIBS_DEBUG}>
             $<$<CONFIG:Debug>:>)

####### Find the libraries declared in cpp_info.libs, create an IMPORTED target for each one and link the
####### gklib_DEPS_TARGET to all of them
conan_package_library_targets("${gklib_LIBS_DEBUG}"    # libraries
                              "${gklib_LIB_DIRS_DEBUG}" # package_libdir
                              "${gklib_BIN_DIRS_DEBUG}" # package_bindir
                              "${gklib_LIBRARY_TYPE_DEBUG}"
                              "${gklib_IS_HOST_WINDOWS_DEBUG}"
                              gklib_DEPS_TARGET
                              gklib_LIBRARIES_TARGETS  # out_libraries_targets
                              "_DEBUG"
                              "gklib"    # package_name
                              "${gklib_NO_SONAME_MODE_DEBUG}")  # soname

# FIXME: What is the result of this for multi-config? All configs adding themselves to path?
set(CMAKE_MODULE_PATH ${gklib_BUILD_DIRS_DEBUG} ${CMAKE_MODULE_PATH})

########## GLOBAL TARGET PROPERTIES Debug ########################################
    set_property(TARGET gklib::gklib
                 APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                 $<$<CONFIG:Debug>:${gklib_OBJECTS_DEBUG}>
                 $<$<CONFIG:Debug>:${gklib_LIBRARIES_TARGETS}>
                 )

    if("${gklib_LIBS_DEBUG}" STREQUAL "")
        # If the package is not declaring any "cpp_info.libs" the package deps, system libs,
        # frameworks etc are not linked to the imported targets and we need to do it to the
        # global target
        set_property(TARGET gklib::gklib
                     APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                     gklib_DEPS_TARGET)
    endif()

    set_property(TARGET gklib::gklib
                 APPEND PROPERTY INTERFACE_LINK_OPTIONS
                 $<$<CONFIG:Debug>:${gklib_LINKER_FLAGS_DEBUG}>)
    set_property(TARGET gklib::gklib
                 APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                 $<$<CONFIG:Debug>:${gklib_INCLUDE_DIRS_DEBUG}>)
    # Necessary to find LINK shared libraries in Linux
    set_property(TARGET gklib::gklib
                 APPEND PROPERTY INTERFACE_LINK_DIRECTORIES
                 $<$<CONFIG:Debug>:${gklib_LIB_DIRS_DEBUG}>)
    set_property(TARGET gklib::gklib
                 APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS
                 $<$<CONFIG:Debug>:${gklib_COMPILE_DEFINITIONS_DEBUG}>)
    set_property(TARGET gklib::gklib
                 APPEND PROPERTY INTERFACE_COMPILE_OPTIONS
                 $<$<CONFIG:Debug>:${gklib_COMPILE_OPTIONS_DEBUG}>)

########## For the modules (FindXXX)
set(gklib_LIBRARIES_DEBUG gklib::gklib)
