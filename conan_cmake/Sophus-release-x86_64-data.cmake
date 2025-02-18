########### AGGREGATED COMPONENTS AND DEPENDENCIES FOR THE MULTI CONFIG #####################
#############################################################################################

set(sophus_COMPONENT_NAMES "")
if(DEFINED sophus_FIND_DEPENDENCY_NAMES)
  list(APPEND sophus_FIND_DEPENDENCY_NAMES Eigen3 fmt)
  list(REMOVE_DUPLICATES sophus_FIND_DEPENDENCY_NAMES)
else()
  set(sophus_FIND_DEPENDENCY_NAMES Eigen3 fmt)
endif()
set(Eigen3_FIND_MODE "NO_MODULE")
set(fmt_FIND_MODE "NO_MODULE")

########### VARIABLES #######################################################################
#############################################################################################
set(sophus_PACKAGE_FOLDER_RELEASE "/root/.conan2/p/sophu16cffce94eaa4/p")
set(sophus_BUILD_MODULES_PATHS_RELEASE )


set(sophus_INCLUDE_DIRS_RELEASE "${sophus_PACKAGE_FOLDER_RELEASE}/include")
set(sophus_RES_DIRS_RELEASE )
set(sophus_DEFINITIONS_RELEASE )
set(sophus_SHARED_LINK_FLAGS_RELEASE )
set(sophus_EXE_LINK_FLAGS_RELEASE )
set(sophus_OBJECTS_RELEASE )
set(sophus_COMPILE_DEFINITIONS_RELEASE )
set(sophus_COMPILE_OPTIONS_C_RELEASE )
set(sophus_COMPILE_OPTIONS_CXX_RELEASE )
set(sophus_LIB_DIRS_RELEASE )
set(sophus_BIN_DIRS_RELEASE )
set(sophus_LIBRARY_TYPE_RELEASE UNKNOWN)
set(sophus_IS_HOST_WINDOWS_RELEASE 0)
set(sophus_LIBS_RELEASE )
set(sophus_SYSTEM_LIBS_RELEASE )
set(sophus_FRAMEWORK_DIRS_RELEASE )
set(sophus_FRAMEWORKS_RELEASE )
set(sophus_BUILD_DIRS_RELEASE )
set(sophus_NO_SONAME_MODE_RELEASE FALSE)


# COMPOUND VARIABLES
set(sophus_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${sophus_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${sophus_COMPILE_OPTIONS_C_RELEASE}>")
set(sophus_LINKER_FLAGS_RELEASE
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${sophus_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${sophus_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${sophus_EXE_LINK_FLAGS_RELEASE}>")


set(sophus_COMPONENTS_RELEASE )