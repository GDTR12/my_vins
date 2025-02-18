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
set(ceres-solver_PACKAGE_FOLDER_RELEASE "/root/.conan2/p/b/ceresfbb374a6ae484/p")
set(ceres-solver_BUILD_MODULES_PATHS_RELEASE "${ceres-solver_PACKAGE_FOLDER_RELEASE}/lib/cmake/conan-official-ceres-solver-variables.cmake")


set(ceres-solver_INCLUDE_DIRS_RELEASE "${ceres-solver_PACKAGE_FOLDER_RELEASE}/include"
			"${ceres-solver_PACKAGE_FOLDER_RELEASE}/include/ceres"
			"${ceres-solver_PACKAGE_FOLDER_RELEASE}/include/ceres/internal/miniglog")
set(ceres-solver_RES_DIRS_RELEASE )
set(ceres-solver_DEFINITIONS_RELEASE )
set(ceres-solver_SHARED_LINK_FLAGS_RELEASE )
set(ceres-solver_EXE_LINK_FLAGS_RELEASE )
set(ceres-solver_OBJECTS_RELEASE )
set(ceres-solver_COMPILE_DEFINITIONS_RELEASE )
set(ceres-solver_COMPILE_OPTIONS_C_RELEASE )
set(ceres-solver_COMPILE_OPTIONS_CXX_RELEASE )
set(ceres-solver_LIB_DIRS_RELEASE "${ceres-solver_PACKAGE_FOLDER_RELEASE}/lib")
set(ceres-solver_BIN_DIRS_RELEASE "${ceres-solver_PACKAGE_FOLDER_RELEASE}/bin")
set(ceres-solver_LIBRARY_TYPE_RELEASE SHARED)
set(ceres-solver_IS_HOST_WINDOWS_RELEASE 0)
set(ceres-solver_LIBS_RELEASE ceres)
set(ceres-solver_SYSTEM_LIBS_RELEASE m pthread)
set(ceres-solver_FRAMEWORK_DIRS_RELEASE )
set(ceres-solver_FRAMEWORKS_RELEASE )
set(ceres-solver_BUILD_DIRS_RELEASE )
set(ceres-solver_NO_SONAME_MODE_RELEASE FALSE)


# COMPOUND VARIABLES
set(ceres-solver_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${ceres-solver_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${ceres-solver_COMPILE_OPTIONS_C_RELEASE}>")
set(ceres-solver_LINKER_FLAGS_RELEASE
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${ceres-solver_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${ceres-solver_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${ceres-solver_EXE_LINK_FLAGS_RELEASE}>")


set(ceres-solver_COMPONENTS_RELEASE Ceres::ceres)
########### COMPONENT Ceres::ceres VARIABLES ############################################

set(ceres-solver_Ceres_ceres_INCLUDE_DIRS_RELEASE "${ceres-solver_PACKAGE_FOLDER_RELEASE}/include"
			"${ceres-solver_PACKAGE_FOLDER_RELEASE}/include/ceres"
			"${ceres-solver_PACKAGE_FOLDER_RELEASE}/include/ceres/internal/miniglog")
set(ceres-solver_Ceres_ceres_LIB_DIRS_RELEASE "${ceres-solver_PACKAGE_FOLDER_RELEASE}/lib")
set(ceres-solver_Ceres_ceres_BIN_DIRS_RELEASE "${ceres-solver_PACKAGE_FOLDER_RELEASE}/bin")
set(ceres-solver_Ceres_ceres_LIBRARY_TYPE_RELEASE SHARED)
set(ceres-solver_Ceres_ceres_IS_HOST_WINDOWS_RELEASE 0)
set(ceres-solver_Ceres_ceres_RES_DIRS_RELEASE )
set(ceres-solver_Ceres_ceres_DEFINITIONS_RELEASE )
set(ceres-solver_Ceres_ceres_OBJECTS_RELEASE )
set(ceres-solver_Ceres_ceres_COMPILE_DEFINITIONS_RELEASE )
set(ceres-solver_Ceres_ceres_COMPILE_OPTIONS_C_RELEASE "")
set(ceres-solver_Ceres_ceres_COMPILE_OPTIONS_CXX_RELEASE "")
set(ceres-solver_Ceres_ceres_LIBS_RELEASE ceres)
set(ceres-solver_Ceres_ceres_SYSTEM_LIBS_RELEASE m pthread)
set(ceres-solver_Ceres_ceres_FRAMEWORK_DIRS_RELEASE )
set(ceres-solver_Ceres_ceres_FRAMEWORKS_RELEASE )
set(ceres-solver_Ceres_ceres_DEPENDENCIES_RELEASE Eigen3::Eigen)
set(ceres-solver_Ceres_ceres_SHARED_LINK_FLAGS_RELEASE )
set(ceres-solver_Ceres_ceres_EXE_LINK_FLAGS_RELEASE )
set(ceres-solver_Ceres_ceres_NO_SONAME_MODE_RELEASE FALSE)

# COMPOUND VARIABLES
set(ceres-solver_Ceres_ceres_LINKER_FLAGS_RELEASE
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${ceres-solver_Ceres_ceres_SHARED_LINK_FLAGS_RELEASE}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${ceres-solver_Ceres_ceres_SHARED_LINK_FLAGS_RELEASE}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${ceres-solver_Ceres_ceres_EXE_LINK_FLAGS_RELEASE}>
)
set(ceres-solver_Ceres_ceres_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${ceres-solver_Ceres_ceres_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${ceres-solver_Ceres_ceres_COMPILE_OPTIONS_C_RELEASE}>")