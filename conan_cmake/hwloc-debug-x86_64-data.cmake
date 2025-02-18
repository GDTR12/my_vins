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
set(hwloc_PACKAGE_FOLDER_DEBUG "/root/.conan2/p/b/hwlocac4c0a1154c1a/p")
set(hwloc_BUILD_MODULES_PATHS_DEBUG )


set(hwloc_INCLUDE_DIRS_DEBUG )
set(hwloc_RES_DIRS_DEBUG )
set(hwloc_DEFINITIONS_DEBUG )
set(hwloc_SHARED_LINK_FLAGS_DEBUG )
set(hwloc_EXE_LINK_FLAGS_DEBUG )
set(hwloc_OBJECTS_DEBUG )
set(hwloc_COMPILE_DEFINITIONS_DEBUG )
set(hwloc_COMPILE_OPTIONS_C_DEBUG )
set(hwloc_COMPILE_OPTIONS_CXX_DEBUG )
set(hwloc_LIB_DIRS_DEBUG "${hwloc_PACKAGE_FOLDER_DEBUG}/lib")
set(hwloc_BIN_DIRS_DEBUG "${hwloc_PACKAGE_FOLDER_DEBUG}/bin")
set(hwloc_LIBRARY_TYPE_DEBUG SHARED)
set(hwloc_IS_HOST_WINDOWS_DEBUG 0)
set(hwloc_LIBS_DEBUG )
set(hwloc_SYSTEM_LIBS_DEBUG )
set(hwloc_FRAMEWORK_DIRS_DEBUG )
set(hwloc_FRAMEWORKS_DEBUG )
set(hwloc_BUILD_DIRS_DEBUG )
set(hwloc_NO_SONAME_MODE_DEBUG FALSE)


# COMPOUND VARIABLES
set(hwloc_COMPILE_OPTIONS_DEBUG
    "$<$<COMPILE_LANGUAGE:CXX>:${hwloc_COMPILE_OPTIONS_CXX_DEBUG}>"
    "$<$<COMPILE_LANGUAGE:C>:${hwloc_COMPILE_OPTIONS_C_DEBUG}>")
set(hwloc_LINKER_FLAGS_DEBUG
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${hwloc_SHARED_LINK_FLAGS_DEBUG}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${hwloc_SHARED_LINK_FLAGS_DEBUG}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${hwloc_EXE_LINK_FLAGS_DEBUG}>")


set(hwloc_COMPONENTS_DEBUG )