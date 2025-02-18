########### AGGREGATED COMPONENTS AND DEPENDENCIES FOR THE MULTI CONFIG #####################
#############################################################################################

list(APPEND eigen_COMPONENT_NAMES Eigen3::Eigen)
list(REMOVE_DUPLICATES eigen_COMPONENT_NAMES)
if(DEFINED eigen_FIND_DEPENDENCY_NAMES)
  list(APPEND eigen_FIND_DEPENDENCY_NAMES )
  list(REMOVE_DUPLICATES eigen_FIND_DEPENDENCY_NAMES)
else()
  set(eigen_FIND_DEPENDENCY_NAMES )
endif()

########### VARIABLES #######################################################################
#############################################################################################
set(eigen_PACKAGE_FOLDER_DEBUG "/root/.conan2/p/eigen3d88c0279cc26/p")
set(eigen_BUILD_MODULES_PATHS_DEBUG )


set(eigen_INCLUDE_DIRS_DEBUG "${eigen_PACKAGE_FOLDER_DEBUG}/include/eigen3")
set(eigen_RES_DIRS_DEBUG )
set(eigen_DEFINITIONS_DEBUG )
set(eigen_SHARED_LINK_FLAGS_DEBUG )
set(eigen_EXE_LINK_FLAGS_DEBUG )
set(eigen_OBJECTS_DEBUG )
set(eigen_COMPILE_DEFINITIONS_DEBUG )
set(eigen_COMPILE_OPTIONS_C_DEBUG )
set(eigen_COMPILE_OPTIONS_CXX_DEBUG )
set(eigen_LIB_DIRS_DEBUG )
set(eigen_BIN_DIRS_DEBUG )
set(eigen_LIBRARY_TYPE_DEBUG UNKNOWN)
set(eigen_IS_HOST_WINDOWS_DEBUG 0)
set(eigen_LIBS_DEBUG )
set(eigen_SYSTEM_LIBS_DEBUG m)
set(eigen_FRAMEWORK_DIRS_DEBUG )
set(eigen_FRAMEWORKS_DEBUG )
set(eigen_BUILD_DIRS_DEBUG )
set(eigen_NO_SONAME_MODE_DEBUG FALSE)


# COMPOUND VARIABLES
set(eigen_COMPILE_OPTIONS_DEBUG
    "$<$<COMPILE_LANGUAGE:CXX>:${eigen_COMPILE_OPTIONS_CXX_DEBUG}>"
    "$<$<COMPILE_LANGUAGE:C>:${eigen_COMPILE_OPTIONS_C_DEBUG}>")
set(eigen_LINKER_FLAGS_DEBUG
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${eigen_SHARED_LINK_FLAGS_DEBUG}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${eigen_SHARED_LINK_FLAGS_DEBUG}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${eigen_EXE_LINK_FLAGS_DEBUG}>")


set(eigen_COMPONENTS_DEBUG Eigen3::Eigen)
########### COMPONENT Eigen3::Eigen VARIABLES ############################################

set(eigen_Eigen3_Eigen_INCLUDE_DIRS_DEBUG "${eigen_PACKAGE_FOLDER_DEBUG}/include/eigen3")
set(eigen_Eigen3_Eigen_LIB_DIRS_DEBUG )
set(eigen_Eigen3_Eigen_BIN_DIRS_DEBUG )
set(eigen_Eigen3_Eigen_LIBRARY_TYPE_DEBUG UNKNOWN)
set(eigen_Eigen3_Eigen_IS_HOST_WINDOWS_DEBUG 0)
set(eigen_Eigen3_Eigen_RES_DIRS_DEBUG )
set(eigen_Eigen3_Eigen_DEFINITIONS_DEBUG )
set(eigen_Eigen3_Eigen_OBJECTS_DEBUG )
set(eigen_Eigen3_Eigen_COMPILE_DEFINITIONS_DEBUG )
set(eigen_Eigen3_Eigen_COMPILE_OPTIONS_C_DEBUG "")
set(eigen_Eigen3_Eigen_COMPILE_OPTIONS_CXX_DEBUG "")
set(eigen_Eigen3_Eigen_LIBS_DEBUG )
set(eigen_Eigen3_Eigen_SYSTEM_LIBS_DEBUG m)
set(eigen_Eigen3_Eigen_FRAMEWORK_DIRS_DEBUG )
set(eigen_Eigen3_Eigen_FRAMEWORKS_DEBUG )
set(eigen_Eigen3_Eigen_DEPENDENCIES_DEBUG )
set(eigen_Eigen3_Eigen_SHARED_LINK_FLAGS_DEBUG )
set(eigen_Eigen3_Eigen_EXE_LINK_FLAGS_DEBUG )
set(eigen_Eigen3_Eigen_NO_SONAME_MODE_DEBUG FALSE)

# COMPOUND VARIABLES
set(eigen_Eigen3_Eigen_LINKER_FLAGS_DEBUG
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${eigen_Eigen3_Eigen_SHARED_LINK_FLAGS_DEBUG}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${eigen_Eigen3_Eigen_SHARED_LINK_FLAGS_DEBUG}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${eigen_Eigen3_Eigen_EXE_LINK_FLAGS_DEBUG}>
)
set(eigen_Eigen3_Eigen_COMPILE_OPTIONS_DEBUG
    "$<$<COMPILE_LANGUAGE:CXX>:${eigen_Eigen3_Eigen_COMPILE_OPTIONS_CXX_DEBUG}>"
    "$<$<COMPILE_LANGUAGE:C>:${eigen_Eigen3_Eigen_COMPILE_OPTIONS_C_DEBUG}>")