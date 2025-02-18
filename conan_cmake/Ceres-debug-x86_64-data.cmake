########### AGGREGATED COMPONENTS AND DEPENDENCIES FOR THE MULTI CONFIG #####################
#############################################################################################

list(APPEND ceres-solver_COMPONENT_NAMES Ceres::ceres)
list(REMOVE_DUPLICATES ceres-solver_COMPONENT_NAMES)
if(DEFINED ceres-solver_FIND_DEPENDENCY_NAMES)
  list(APPEND ceres-solver_FIND_DEPENDENCY_NAMES Eigen3)
  list(REMOVE_DUPLICATES ceres-solver_FIND_DEPENDENCY_NAMES)
else()
  set(ceres-solver_FIND_DEPENDENCY_NAMES Eigen3)
endif()
set(Eigen3_FIND_MODE "NO_MODULE")

########### VARIABLES #######################################################################
#############################################################################################
set(ceres-solver_PACKAGE_FOLDER_DEBUG "/root/.conan2/p/b/ceres7559e225d63a0/p")
set(ceres-solver_BUILD_MODULES_PATHS_DEBUG "${ceres-solver_PACKAGE_FOLDER_DEBUG}/lib/cmake/conan-official-ceres-solver-variables.cmake")


set(ceres-solver_INCLUDE_DIRS_DEBUG "${ceres-solver_PACKAGE_FOLDER_DEBUG}/include"
			"${ceres-solver_PACKAGE_FOLDER_DEBUG}/include/ceres"
			"${ceres-solver_PACKAGE_FOLDER_DEBUG}/include/ceres/internal/miniglog")
set(ceres-solver_RES_DIRS_DEBUG )
set(ceres-solver_DEFINITIONS_DEBUG )
set(ceres-solver_SHARED_LINK_FLAGS_DEBUG )
set(ceres-solver_EXE_LINK_FLAGS_DEBUG )
set(ceres-solver_OBJECTS_DEBUG )
set(ceres-solver_COMPILE_DEFINITIONS_DEBUG )
set(ceres-solver_COMPILE_OPTIONS_C_DEBUG )
set(ceres-solver_COMPILE_OPTIONS_CXX_DEBUG )
set(ceres-solver_LIB_DIRS_DEBUG "${ceres-solver_PACKAGE_FOLDER_DEBUG}/lib")
set(ceres-solver_BIN_DIRS_DEBUG "${ceres-solver_PACKAGE_FOLDER_DEBUG}/bin")
set(ceres-solver_LIBRARY_TYPE_DEBUG SHARED)
set(ceres-solver_IS_HOST_WINDOWS_DEBUG 0)
set(ceres-solver_LIBS_DEBUG ceres-debug)
set(ceres-solver_SYSTEM_LIBS_DEBUG m pthread)
set(ceres-solver_FRAMEWORK_DIRS_DEBUG )
set(ceres-solver_FRAMEWORKS_DEBUG )
set(ceres-solver_BUILD_DIRS_DEBUG )
set(ceres-solver_NO_SONAME_MODE_DEBUG FALSE)


# COMPOUND VARIABLES
set(ceres-solver_COMPILE_OPTIONS_DEBUG
    "$<$<COMPILE_LANGUAGE:CXX>:${ceres-solver_COMPILE_OPTIONS_CXX_DEBUG}>"
    "$<$<COMPILE_LANGUAGE:C>:${ceres-solver_COMPILE_OPTIONS_C_DEBUG}>")
set(ceres-solver_LINKER_FLAGS_DEBUG
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${ceres-solver_SHARED_LINK_FLAGS_DEBUG}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${ceres-solver_SHARED_LINK_FLAGS_DEBUG}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${ceres-solver_EXE_LINK_FLAGS_DEBUG}>")


set(ceres-solver_COMPONENTS_DEBUG Ceres::ceres)
########### COMPONENT Ceres::ceres VARIABLES ############################################

set(ceres-solver_Ceres_ceres_INCLUDE_DIRS_DEBUG "${ceres-solver_PACKAGE_FOLDER_DEBUG}/include"
			"${ceres-solver_PACKAGE_FOLDER_DEBUG}/include/ceres"
			"${ceres-solver_PACKAGE_FOLDER_DEBUG}/include/ceres/internal/miniglog")
set(ceres-solver_Ceres_ceres_LIB_DIRS_DEBUG "${ceres-solver_PACKAGE_FOLDER_DEBUG}/lib")
set(ceres-solver_Ceres_ceres_BIN_DIRS_DEBUG "${ceres-solver_PACKAGE_FOLDER_DEBUG}/bin")
set(ceres-solver_Ceres_ceres_LIBRARY_TYPE_DEBUG SHARED)
set(ceres-solver_Ceres_ceres_IS_HOST_WINDOWS_DEBUG 0)
set(ceres-solver_Ceres_ceres_RES_DIRS_DEBUG )
set(ceres-solver_Ceres_ceres_DEFINITIONS_DEBUG )
set(ceres-solver_Ceres_ceres_OBJECTS_DEBUG )
set(ceres-solver_Ceres_ceres_COMPILE_DEFINITIONS_DEBUG )
set(ceres-solver_Ceres_ceres_COMPILE_OPTIONS_C_DEBUG "")
set(ceres-solver_Ceres_ceres_COMPILE_OPTIONS_CXX_DEBUG "")
set(ceres-solver_Ceres_ceres_LIBS_DEBUG ceres-debug)
set(ceres-solver_Ceres_ceres_SYSTEM_LIBS_DEBUG m pthread)
set(ceres-solver_Ceres_ceres_FRAMEWORK_DIRS_DEBUG )
set(ceres-solver_Ceres_ceres_FRAMEWORKS_DEBUG )
set(ceres-solver_Ceres_ceres_DEPENDENCIES_DEBUG Eigen3::Eigen)
set(ceres-solver_Ceres_ceres_SHARED_LINK_FLAGS_DEBUG )
set(ceres-solver_Ceres_ceres_EXE_LINK_FLAGS_DEBUG )
set(ceres-solver_Ceres_ceres_NO_SONAME_MODE_DEBUG FALSE)

# COMPOUND VARIABLES
set(ceres-solver_Ceres_ceres_LINKER_FLAGS_DEBUG
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${ceres-solver_Ceres_ceres_SHARED_LINK_FLAGS_DEBUG}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${ceres-solver_Ceres_ceres_SHARED_LINK_FLAGS_DEBUG}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${ceres-solver_Ceres_ceres_EXE_LINK_FLAGS_DEBUG}>
)
set(ceres-solver_Ceres_ceres_COMPILE_OPTIONS_DEBUG
    "$<$<COMPILE_LANGUAGE:CXX>:${ceres-solver_Ceres_ceres_COMPILE_OPTIONS_CXX_DEBUG}>"
    "$<$<COMPILE_LANGUAGE:C>:${ceres-solver_Ceres_ceres_COMPILE_OPTIONS_C_DEBUG}>")