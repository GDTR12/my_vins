# Avoid multiple calls to find_package to append duplicated properties to the targets
include_guard()########### VARIABLES #######################################################################
#############################################################################################
set(eigen_FRAMEWORKS_FOUND_DEBUG "") # Will be filled later
conan_find_apple_frameworks(eigen_FRAMEWORKS_FOUND_DEBUG "${eigen_FRAMEWORKS_DEBUG}" "${eigen_FRAMEWORK_DIRS_DEBUG}")

set(eigen_LIBRARIES_TARGETS "") # Will be filled later


######## Create an interface target to contain all the dependencies (frameworks, system and conan deps)
if(NOT TARGET eigen_DEPS_TARGET)
    add_library(eigen_DEPS_TARGET INTERFACE IMPORTED)
endif()

set_property(TARGET eigen_DEPS_TARGET
             APPEND PROPERTY INTERFACE_LINK_LIBRARIES
             $<$<CONFIG:Debug>:${eigen_FRAMEWORKS_FOUND_DEBUG}>
             $<$<CONFIG:Debug>:${eigen_SYSTEM_LIBS_DEBUG}>
             $<$<CONFIG:Debug>:>)

####### Find the libraries declared in cpp_info.libs, create an IMPORTED target for each one and link the
####### eigen_DEPS_TARGET to all of them
conan_package_library_targets("${eigen_LIBS_DEBUG}"    # libraries
                              "${eigen_LIB_DIRS_DEBUG}" # package_libdir
                              "${eigen_BIN_DIRS_DEBUG}" # package_bindir
                              "${eigen_LIBRARY_TYPE_DEBUG}"
                              "${eigen_IS_HOST_WINDOWS_DEBUG}"
                              eigen_DEPS_TARGET
                              eigen_LIBRARIES_TARGETS  # out_libraries_targets
                              "_DEBUG"
                              "eigen"    # package_name
                              "${eigen_NO_SONAME_MODE_DEBUG}")  # soname

# FIXME: What is the result of this for multi-config? All configs adding themselves to path?
set(CMAKE_MODULE_PATH ${eigen_BUILD_DIRS_DEBUG} ${CMAKE_MODULE_PATH})

########## COMPONENTS TARGET PROPERTIES Debug ########################################

    ########## COMPONENT Eigen3::Eigen #############

        set(eigen_Eigen3_Eigen_FRAMEWORKS_FOUND_DEBUG "")
        conan_find_apple_frameworks(eigen_Eigen3_Eigen_FRAMEWORKS_FOUND_DEBUG "${eigen_Eigen3_Eigen_FRAMEWORKS_DEBUG}" "${eigen_Eigen3_Eigen_FRAMEWORK_DIRS_DEBUG}")

        set(eigen_Eigen3_Eigen_LIBRARIES_TARGETS "")

        ######## Create an interface target to contain all the dependencies (frameworks, system and conan deps)
        if(NOT TARGET eigen_Eigen3_Eigen_DEPS_TARGET)
            add_library(eigen_Eigen3_Eigen_DEPS_TARGET INTERFACE IMPORTED)
        endif()

        set_property(TARGET eigen_Eigen3_Eigen_DEPS_TARGET
                     APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                     $<$<CONFIG:Debug>:${eigen_Eigen3_Eigen_FRAMEWORKS_FOUND_DEBUG}>
                     $<$<CONFIG:Debug>:${eigen_Eigen3_Eigen_SYSTEM_LIBS_DEBUG}>
                     $<$<CONFIG:Debug>:${eigen_Eigen3_Eigen_DEPENDENCIES_DEBUG}>
                     )

        ####### Find the libraries declared in cpp_info.component["xxx"].libs,
        ####### create an IMPORTED target for each one and link the 'eigen_Eigen3_Eigen_DEPS_TARGET' to all of them
        conan_package_library_targets("${eigen_Eigen3_Eigen_LIBS_DEBUG}"
                              "${eigen_Eigen3_Eigen_LIB_DIRS_DEBUG}"
                              "${eigen_Eigen3_Eigen_BIN_DIRS_DEBUG}" # package_bindir
                              "${eigen_Eigen3_Eigen_LIBRARY_TYPE_DEBUG}"
                              "${eigen_Eigen3_Eigen_IS_HOST_WINDOWS_DEBUG}"
                              eigen_Eigen3_Eigen_DEPS_TARGET
                              eigen_Eigen3_Eigen_LIBRARIES_TARGETS
                              "_DEBUG"
                              "eigen_Eigen3_Eigen"
                              "${eigen_Eigen3_Eigen_NO_SONAME_MODE_DEBUG}")


        ########## TARGET PROPERTIES #####################################
        set_property(TARGET Eigen3::Eigen
                     APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                     $<$<CONFIG:Debug>:${eigen_Eigen3_Eigen_OBJECTS_DEBUG}>
                     $<$<CONFIG:Debug>:${eigen_Eigen3_Eigen_LIBRARIES_TARGETS}>
                     )

        if("${eigen_Eigen3_Eigen_LIBS_DEBUG}" STREQUAL "")
            # If the component is not declaring any "cpp_info.components['foo'].libs" the system, frameworks etc are not
            # linked to the imported targets and we need to do it to the global target
            set_property(TARGET Eigen3::Eigen
                         APPEND PROPERTY INTERFACE_LINK_LIBRARIES
                         eigen_Eigen3_Eigen_DEPS_TARGET)
        endif()

        set_property(TARGET Eigen3::Eigen APPEND PROPERTY INTERFACE_LINK_OPTIONS
                     $<$<CONFIG:Debug>:${eigen_Eigen3_Eigen_LINKER_FLAGS_DEBUG}>)
        set_property(TARGET Eigen3::Eigen APPEND PROPERTY INTERFACE_INCLUDE_DIRECTORIES
                     $<$<CONFIG:Debug>:${eigen_Eigen3_Eigen_INCLUDE_DIRS_DEBUG}>)
        set_property(TARGET Eigen3::Eigen APPEND PROPERTY INTERFACE_LINK_DIRECTORIES
                     $<$<CONFIG:Debug>:${eigen_Eigen3_Eigen_LIB_DIRS_DEBUG}>)
        set_property(TARGET Eigen3::Eigen APPEND PROPERTY INTERFACE_COMPILE_DEFINITIONS
                     $<$<CONFIG:Debug>:${eigen_Eigen3_Eigen_COMPILE_DEFINITIONS_DEBUG}>)
        set_property(TARGET Eigen3::Eigen APPEND PROPERTY INTERFACE_COMPILE_OPTIONS
                     $<$<CONFIG:Debug>:${eigen_Eigen3_Eigen_COMPILE_OPTIONS_DEBUG}>)

    ########## AGGREGATED GLOBAL TARGET WITH THE COMPONENTS #####################
    set_property(TARGET Eigen3::Eigen APPEND PROPERTY INTERFACE_LINK_LIBRARIES Eigen3::Eigen)

########## For the modules (FindXXX)
set(eigen_LIBRARIES_DEBUG Eigen3::Eigen)
