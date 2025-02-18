# Avoid multiple calls to find_package to append duplicated properties to the targets
include_guard()########### VARIABLES #######################################################################
#############################################################################################
set(hwloc_FRAMEWORKS_FOUND_DEBUG "") # Will be filled later
conan_find_apple_frameworks(hwloc_FRAMEWORKS_FOUND_DEBUG "${hwloc_FRAMEWORKS_DEBUG}" "${hwloc_FRAMEWORK_DIRS_DEBUG}")

set(hwloc_LIBRARIES_TARGETS "") # Will be filled later


######## Create an interface target to contain all the dependencies (frameworks, system and conan deps)
if(NOT TARGET hwloc_DEPS_TARGET)
    add_library(hwloc_DEPS_TARGET INTERFACE IMPORTED)
endif()

set_property(TARGET hwloc_DEPS_TARGET
             APPEND PROPERTY INTERFACE_LINK_LIBRARIES
             $<$<CONFIG:Debug>:${hwloc_FRAMEWORKS_FOUND_DEBUG}>
             $<$<CONFIG:Debug>:${hwloc_SYSTEM_LIBS_DEBUG}>
             $<$<CONFIG:Debug>:>)

####### Find the libraries declared in cpp_info.libs, create an IMPORTED target for each one and link the
####### hwloc_DEPS_TARGET to all of them
conan_package_library_targets("${hwloc_LIBS_DEBUG}"    # libraries
                              "${hwloc_LIB_DIRS_DEBUG}" # package_libdir
                              "${hwloc_BIN_DIRS_DEBUG}" # package_bindir
                              "${hwloc_LIBRARY_TYPE_DEBUG}"
                              "${hwloc_IS_HOST_WINDOWS_DEBUG}"
                              hwloc_DEPS_TARGET
                              hwloc_LIBRARIES_TARGETS  # out_libraries_targets
                              "_DEBUG"
                              "hwloc"    # package_name
                              "${hwloc_NO_SONAME_MODE_DEBUG}")  # soname

# FIXME: What is the result of this for multi-config? All configs adding themselves to path?
set(CMAKE_MODULE_PATH ${hwloc_BUILD_DIRS_DEBUG} ${CMAKE_MODULE_PATH})

########## GLOBAL TARGET PROPERTIES Debug ########################################
    set_property(TARGET hwloc::hwloc
                 APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                 $<$<CONFIG:Debug>:${hwloc_OBJECTS_DEBUG}>
                 $<$<CONFIG:Debug>:${hwloc_LIBRARIES_TARGETS}>
                 )

    if("${hwloc_LIBS_DEBUG}" STREQUAL "")
        # If the package is not declaring any "cpp_info.libs" the package deps, system libs,
        # frameworks etc are not linked to the imported targets and we need to do it to the
        # global target
        set_property(TARGET hwloc::hwloc
                     APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                     hwloc_DEPS_TARGET)
    endif()

    set_property(TARGET hwloc::hwloc
                 APPEND PROPERTY INTERFACE_LINK_OPTIONS
                 $<$<CONFIG:Debug>:${hwloc_LINKER_FLAGS_DEBUG}>)
    set_property(TARGET hwloc::hwloc
                 APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                 $<$<CONFIG:Debug>:${hwloc_INCLUDE_DIRS_DEBUG}>)
    # Necessary to find LINK shared libraries in Linux
    set_property(TARGET hwloc::hwloc
                 APPEND PROPERTY INTERFACE_LINK_DIRECTORIES
                 $<$<CONFIG:Debug>:${hwloc_LIB_DIRS_DEBUG}>)
    set_property(TARGET hwloc::hwloc
                 APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS
                 $<$<CONFIG:Debug>:${hwloc_COMPILE_DEFINITIONS_DEBUG}>)
    set_property(TARGET hwloc::hwloc
                 APPEND PROPERTY INTERFACE_COMPILE_OPTIONS
                 $<$<CONFIG:Debug>:${hwloc_COMPILE_OPTIONS_DEBUG}>)

########## For the modules (FindXXX)
set(hwloc_LIBRARIES_DEBUG hwloc::hwloc)
