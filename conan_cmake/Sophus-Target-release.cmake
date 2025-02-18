# Avoid multiple calls to find_package to append duplicated properties to the targets
include_guard()########### VARIABLES #######################################################################
#############################################################################################
set(sophus_FRAMEWORKS_FOUND_RELEASE "") # Will be filled later
conan_find_apple_frameworks(sophus_FRAMEWORKS_FOUND_RELEASE "${sophus_FRAMEWORKS_RELEASE}" "${sophus_FRAMEWORK_DIRS_RELEASE}")

set(sophus_LIBRARIES_TARGETS "") # Will be filled later


######## Create an interface target to contain all the dependencies (frameworks, system and conan deps)
if(NOT TARGET sophus_DEPS_TARGET)
    add_library(sophus_DEPS_TARGET INTERFACE IMPORTED)
endif()

set_property(TARGET sophus_DEPS_TARGET
             APPEND PROPERTY INTERFACE_LINK_LIBRARIES
             $<$<CONFIG:Release>:${sophus_FRAMEWORKS_FOUND_RELEASE}>
             $<$<CONFIG:Release>:${sophus_SYSTEM_LIBS_RELEASE}>
             $<$<CONFIG:Release>:Eigen3::Eigen;fmt::fmt>)

####### Find the libraries declared in cpp_info.libs, create an IMPORTED target for each one and link the
####### sophus_DEPS_TARGET to all of them
conan_package_library_targets("${sophus_LIBS_RELEASE}"    # libraries
                              "${sophus_LIB_DIRS_RELEASE}" # package_libdir
                              "${sophus_BIN_DIRS_RELEASE}" # package_bindir
                              "${sophus_LIBRARY_TYPE_RELEASE}"
                              "${sophus_IS_HOST_WINDOWS_RELEASE}"
                              sophus_DEPS_TARGET
                              sophus_LIBRARIES_TARGETS  # out_libraries_targets
                              "_RELEASE"
                              "sophus"    # package_name
                              "${sophus_NO_SONAME_MODE_RELEASE}")  # soname

# FIXME: What is the result of this for multi-config? All configs adding themselves to path?
set(CMAKE_MODULE_PATH ${sophus_BUILD_DIRS_RELEASE} ${CMAKE_MODULE_PATH})

########## GLOBAL TARGET PROPERTIES Release ########################################
    set_property(TARGET Sophus::Sophus
                 APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                 $<$<CONFIG:Release>:${sophus_OBJECTS_RELEASE}>
                 $<$<CONFIG:Release>:${sophus_LIBRARIES_TARGETS}>
                 )

    if("${sophus_LIBS_RELEASE}" STREQUAL "")
        # If the package is not declaring any "cpp_info.libs" the package deps, system libs,
        # frameworks etc are not linked to the imported targets and we need to do it to the
        # global target
        set_property(TARGET Sophus::Sophus
                     APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                     sophus_DEPS_TARGET)
    endif()

    set_property(TARGET Sophus::Sophus
                 APPEND PROPERTY INTERFACE_LINK_OPTIONS
                 $<$<CONFIG:Release>:${sophus_LINKER_FLAGS_RELEASE}>)
    set_property(TARGET Sophus::Sophus
                 APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                 $<$<CONFIG:Release>:${sophus_INCLUDE_DIRS_RELEASE}>)
    # Necessary to find LINK shared libraries in Linux
    set_property(TARGET Sophus::Sophus
                 APPEND PROPERTY INTERFACE_LINK_DIRECTORIES
                 $<$<CONFIG:Release>:${sophus_LIB_DIRS_RELEASE}>)
    set_property(TARGET Sophus::Sophus
                 APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS
                 $<$<CONFIG:Release>:${sophus_COMPILE_DEFINITIONS_RELEASE}>)
    set_property(TARGET Sophus::Sophus
                 APPEND PROPERTY INTERFACE_COMPILE_OPTIONS
                 $<$<CONFIG:Release>:${sophus_COMPILE_OPTIONS_RELEASE}>)

########## For the modules (FindXXX)
set(sophus_LIBRARIES_RELEASE Sophus::Sophus)
