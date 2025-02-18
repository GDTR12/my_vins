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
set(sophus_PACKAGE_FOLDER_DEBUG "/root/.conan2/p/sophu16cffce94eaa4/p")
set(sophus_BUILD_MODULES_PATHS_DEBUG )


set(sophus_INCLUDE_DIRS_DEBUG "${sophus_PACKAGE_FOLDER_DEBUG}/include")
set(sophus_RES_DIRS_DEBUG )
set(sophus_DEFINITIONS_DEBUG )
set(sophus_SHARED_LINK_FLAGS_DEBUG )
set(sophus_EXE_LINK_FLAGS_DEBUG )
set(sophus_OBJECTS_DEBUG )
set(sophus_COMPILE_DEFINITIONS_DEBUG )
set(sophus_COMPILE_OPTIONS_C_DEBUG )
set(sophus_COMPILE_OPTIONS_CXX_DEBUG )
set(sophus_LIB_DIRS_DEBUG )
set(sophus_BIN_DIRS_DEBUG )
set(sophus_LIBRARY_TYPE_DEBUG UNKNOWN)
set(sophus_IS_HOST_WINDOWS_DEBUG 0)
set(sophus_LIBS_DEBUG )
set(sophus_SYSTEM_LIBS_DEBUG )
set(sophus_FRAMEWORK_DIRS_DEBUG )
set(sophus_FRAMEWORKS_DEBUG )
set(sophus_BUILD_DIRS_DEBUG )
set(sophus_NO_SONAME_MODE_DEBUG FALSE)


# COMPOUND VARIABLES
set(sophus_COMPILE_OPTIONS_DEBUG
    "$<$<COMPILE_LANGUAGE:CXX>:${sophus_COMPILE_OPTIONS_CXX_DEBUG}>"
    "$<$<COMPILE_LANGUAGE:C>:${sophus_COMPILE_OPTIONS_C_DEBUG}>")
set(sophus_LINKER_FLAGS_DEBUG
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${sophus_SHARED_LINK_FLAGS_DEBUG}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${sophus_SHARED_LINK_FLAGS_DEBUG}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${sophus_EXE_LINK_FLAGS_DEBUG}>")


set(sophus_COMPONENTS_DEBUG )