########### AGGREGATED COMPONENTS AND DEPENDENCIES FOR THE MULTI CONFIG #####################
#############################################################################################

set(hwloc_COMPONENT_NAMES "")
if(DEFINED hwloc_FIND_DEPENDENCY_NAMES)
  list(APPEND hwloc_FIND_DEPENDENCY_NAMES )
  list(REMOVE_DUPLICATES hwloc_FIND_DEPENDENCY_NAMES)
else()
  set(hwloc_FIND_DEPENDENCY_NAMES )
endif()

########### VARIABLES #######################################################################
#############################################################################################
set(hwloc_PACKAGE_FOLDER_RELEASE "/root/.conan2/p/b/hwlocd1e9d75d1f1c6/p")
set(hwloc_BUILD_MODULES_PATHS_RELEASE )


set(hwloc_INCLUDE_DIRS_RELEASE )
set(hwloc_RES_DIRS_RELEASE )
set(hwloc_DEFINITIONS_RELEASE )
set(hwloc_SHARED_LINK_FLAGS_RELEASE )
set(hwloc_EXE_LINK_FLAGS_RELEASE )
set(hwloc_OBJECTS_RELEASE )
set(hwloc_COMPILE_DEFINITIONS_RELEASE )
set(hwloc_COMPILE_OPTIONS_C_RELEASE )
set(hwloc_COMPILE_OPTIONS_CXX_RELEASE )
set(hwloc_LIB_DIRS_RELEASE "${hwloc_PACKAGE_FOLDER_RELEASE}/lib")
set(hwloc_BIN_DIRS_RELEASE "${hwloc_PACKAGE_FOLDER_RELEASE}/bin")
set(hwloc_LIBRARY_TYPE_RELEASE SHARED)
set(hwloc_IS_HOST_WINDOWS_RELEASE 0)
set(hwloc_LIBS_RELEASE )
set(hwloc_SYSTEM_LIBS_RELEASE )
set(hwloc_FRAMEWORK_DIRS_RELEASE )
set(hwloc_FRAMEWORKS_RELEASE )
set(hwloc_BUILD_DIRS_RELEASE )
set(hwloc_NO_SONAME_MODE_RELEASE FALSE)


# COMPOUND VARIABLES
set(hwloc_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${hwloc_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${hwloc_COMPILE_OPTIONS_C_RELEASE}>")
set(hwloc_LINKER_FLAGS_RELEASE
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${hwloc_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${hwloc_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${hwloc_EXE_LINK_FLAGS_RELEASE}>")


set(hwloc_COMPONENTS_RELEASE )