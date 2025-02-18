########### AGGREGATED COMPONENTS AND DEPENDENCIES FOR THE MULTI CONFIG #####################
#############################################################################################

list(APPEND onetbb_COMPONENT_NAMES TBB::tbb TBB::tbbmalloc TBB::tbbmalloc_proxy)
list(REMOVE_DUPLICATES onetbb_COMPONENT_NAMES)
if(DEFINED onetbb_FIND_DEPENDENCY_NAMES)
  list(APPEND onetbb_FIND_DEPENDENCY_NAMES hwloc)
  list(REMOVE_DUPLICATES onetbb_FIND_DEPENDENCY_NAMES)
else()
  set(onetbb_FIND_DEPENDENCY_NAMES hwloc)
endif()
set(hwloc_FIND_MODE "NO_MODULE")

########### VARIABLES #######################################################################
#############################################################################################
set(onetbb_PACKAGE_FOLDER_DEBUG "/root/.conan2/p/b/onetbfe249136fcbde/p")
set(onetbb_BUILD_MODULES_PATHS_DEBUG )


set(onetbb_INCLUDE_DIRS_DEBUG "${onetbb_PACKAGE_FOLDER_DEBUG}/include")
set(onetbb_RES_DIRS_DEBUG )
set(onetbb_DEFINITIONS_DEBUG )
set(onetbb_SHARED_LINK_FLAGS_DEBUG )
set(onetbb_EXE_LINK_FLAGS_DEBUG )
set(onetbb_OBJECTS_DEBUG )
set(onetbb_COMPILE_DEFINITIONS_DEBUG )
set(onetbb_COMPILE_OPTIONS_C_DEBUG )
set(onetbb_COMPILE_OPTIONS_CXX_DEBUG )
set(onetbb_LIB_DIRS_DEBUG "${onetbb_PACKAGE_FOLDER_DEBUG}/lib")
set(onetbb_BIN_DIRS_DEBUG "${onetbb_PACKAGE_FOLDER_DEBUG}/bin")
set(onetbb_LIBRARY_TYPE_DEBUG SHARED)
set(onetbb_IS_HOST_WINDOWS_DEBUG 0)
set(onetbb_LIBS_DEBUG tbbmalloc_proxy_debug tbbmalloc_debug tbb_debug)
set(onetbb_SYSTEM_LIBS_DEBUG m dl pthread rt)
set(onetbb_FRAMEWORK_DIRS_DEBUG )
set(onetbb_FRAMEWORKS_DEBUG )
set(onetbb_BUILD_DIRS_DEBUG )
set(onetbb_NO_SONAME_MODE_DEBUG FALSE)


# COMPOUND VARIABLES
set(onetbb_COMPILE_OPTIONS_DEBUG
    "$<$<COMPILE_LANGUAGE:CXX>:${onetbb_COMPILE_OPTIONS_CXX_DEBUG}>"
    "$<$<COMPILE_LANGUAGE:C>:${onetbb_COMPILE_OPTIONS_C_DEBUG}>")
set(onetbb_LINKER_FLAGS_DEBUG
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${onetbb_SHARED_LINK_FLAGS_DEBUG}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${onetbb_SHARED_LINK_FLAGS_DEBUG}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${onetbb_EXE_LINK_FLAGS_DEBUG}>")


set(onetbb_COMPONENTS_DEBUG TBB::tbb TBB::tbbmalloc TBB::tbbmalloc_proxy)
########### COMPONENT TBB::tbbmalloc_proxy VARIABLES ############################################

set(onetbb_TBB_tbbmalloc_proxy_INCLUDE_DIRS_DEBUG "${onetbb_PACKAGE_FOLDER_DEBUG}/include")
set(onetbb_TBB_tbbmalloc_proxy_LIB_DIRS_DEBUG "${onetbb_PACKAGE_FOLDER_DEBUG}/lib")
set(onetbb_TBB_tbbmalloc_proxy_BIN_DIRS_DEBUG "${onetbb_PACKAGE_FOLDER_DEBUG}/bin")
set(onetbb_TBB_tbbmalloc_proxy_LIBRARY_TYPE_DEBUG SHARED)
set(onetbb_TBB_tbbmalloc_proxy_IS_HOST_WINDOWS_DEBUG 0)
set(onetbb_TBB_tbbmalloc_proxy_RES_DIRS_DEBUG )
set(onetbb_TBB_tbbmalloc_proxy_DEFINITIONS_DEBUG )
set(onetbb_TBB_tbbmalloc_proxy_OBJECTS_DEBUG )
set(onetbb_TBB_tbbmalloc_proxy_COMPILE_DEFINITIONS_DEBUG )
set(onetbb_TBB_tbbmalloc_proxy_COMPILE_OPTIONS_C_DEBUG "")
set(onetbb_TBB_tbbmalloc_proxy_COMPILE_OPTIONS_CXX_DEBUG "")
set(onetbb_TBB_tbbmalloc_proxy_LIBS_DEBUG tbbmalloc_proxy_debug)
set(onetbb_TBB_tbbmalloc_proxy_SYSTEM_LIBS_DEBUG m dl pthread)
set(onetbb_TBB_tbbmalloc_proxy_FRAMEWORK_DIRS_DEBUG )
set(onetbb_TBB_tbbmalloc_proxy_FRAMEWORKS_DEBUG )
set(onetbb_TBB_tbbmalloc_proxy_DEPENDENCIES_DEBUG TBB::tbbmalloc)
set(onetbb_TBB_tbbmalloc_proxy_SHARED_LINK_FLAGS_DEBUG )
set(onetbb_TBB_tbbmalloc_proxy_EXE_LINK_FLAGS_DEBUG )
set(onetbb_TBB_tbbmalloc_proxy_NO_SONAME_MODE_DEBUG FALSE)

# COMPOUND VARIABLES
set(onetbb_TBB_tbbmalloc_proxy_LINKER_FLAGS_DEBUG
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${onetbb_TBB_tbbmalloc_proxy_SHARED_LINK_FLAGS_DEBUG}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${onetbb_TBB_tbbmalloc_proxy_SHARED_LINK_FLAGS_DEBUG}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${onetbb_TBB_tbbmalloc_proxy_EXE_LINK_FLAGS_DEBUG}>
)
set(onetbb_TBB_tbbmalloc_proxy_COMPILE_OPTIONS_DEBUG
    "$<$<COMPILE_LANGUAGE:CXX>:${onetbb_TBB_tbbmalloc_proxy_COMPILE_OPTIONS_CXX_DEBUG}>"
    "$<$<COMPILE_LANGUAGE:C>:${onetbb_TBB_tbbmalloc_proxy_COMPILE_OPTIONS_C_DEBUG}>")
########### COMPONENT TBB::tbbmalloc VARIABLES ############################################

set(onetbb_TBB_tbbmalloc_INCLUDE_DIRS_DEBUG "${onetbb_PACKAGE_FOLDER_DEBUG}/include")
set(onetbb_TBB_tbbmalloc_LIB_DIRS_DEBUG "${onetbb_PACKAGE_FOLDER_DEBUG}/lib")
set(onetbb_TBB_tbbmalloc_BIN_DIRS_DEBUG "${onetbb_PACKAGE_FOLDER_DEBUG}/bin")
set(onetbb_TBB_tbbmalloc_LIBRARY_TYPE_DEBUG SHARED)
set(onetbb_TBB_tbbmalloc_IS_HOST_WINDOWS_DEBUG 0)
set(onetbb_TBB_tbbmalloc_RES_DIRS_DEBUG )
set(onetbb_TBB_tbbmalloc_DEFINITIONS_DEBUG )
set(onetbb_TBB_tbbmalloc_OBJECTS_DEBUG )
set(onetbb_TBB_tbbmalloc_COMPILE_DEFINITIONS_DEBUG )
set(onetbb_TBB_tbbmalloc_COMPILE_OPTIONS_C_DEBUG "")
set(onetbb_TBB_tbbmalloc_COMPILE_OPTIONS_CXX_DEBUG "")
set(onetbb_TBB_tbbmalloc_LIBS_DEBUG tbbmalloc_debug)
set(onetbb_TBB_tbbmalloc_SYSTEM_LIBS_DEBUG dl pthread)
set(onetbb_TBB_tbbmalloc_FRAMEWORK_DIRS_DEBUG )
set(onetbb_TBB_tbbmalloc_FRAMEWORKS_DEBUG )
set(onetbb_TBB_tbbmalloc_DEPENDENCIES_DEBUG )
set(onetbb_TBB_tbbmalloc_SHARED_LINK_FLAGS_DEBUG )
set(onetbb_TBB_tbbmalloc_EXE_LINK_FLAGS_DEBUG )
set(onetbb_TBB_tbbmalloc_NO_SONAME_MODE_DEBUG FALSE)

# COMPOUND VARIABLES
set(onetbb_TBB_tbbmalloc_LINKER_FLAGS_DEBUG
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${onetbb_TBB_tbbmalloc_SHARED_LINK_FLAGS_DEBUG}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${onetbb_TBB_tbbmalloc_SHARED_LINK_FLAGS_DEBUG}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${onetbb_TBB_tbbmalloc_EXE_LINK_FLAGS_DEBUG}>
)
set(onetbb_TBB_tbbmalloc_COMPILE_OPTIONS_DEBUG
    "$<$<COMPILE_LANGUAGE:CXX>:${onetbb_TBB_tbbmalloc_COMPILE_OPTIONS_CXX_DEBUG}>"
    "$<$<COMPILE_LANGUAGE:C>:${onetbb_TBB_tbbmalloc_COMPILE_OPTIONS_C_DEBUG}>")
########### COMPONENT TBB::tbb VARIABLES ############################################

set(onetbb_TBB_tbb_INCLUDE_DIRS_DEBUG "${onetbb_PACKAGE_FOLDER_DEBUG}/include")
set(onetbb_TBB_tbb_LIB_DIRS_DEBUG "${onetbb_PACKAGE_FOLDER_DEBUG}/lib")
set(onetbb_TBB_tbb_BIN_DIRS_DEBUG "${onetbb_PACKAGE_FOLDER_DEBUG}/bin")
set(onetbb_TBB_tbb_LIBRARY_TYPE_DEBUG SHARED)
set(onetbb_TBB_tbb_IS_HOST_WINDOWS_DEBUG 0)
set(onetbb_TBB_tbb_RES_DIRS_DEBUG )
set(onetbb_TBB_tbb_DEFINITIONS_DEBUG )
set(onetbb_TBB_tbb_OBJECTS_DEBUG )
set(onetbb_TBB_tbb_COMPILE_DEFINITIONS_DEBUG )
set(onetbb_TBB_tbb_COMPILE_OPTIONS_C_DEBUG "")
set(onetbb_TBB_tbb_COMPILE_OPTIONS_CXX_DEBUG "")
set(onetbb_TBB_tbb_LIBS_DEBUG tbb_debug)
set(onetbb_TBB_tbb_SYSTEM_LIBS_DEBUG m dl rt pthread)
set(onetbb_TBB_tbb_FRAMEWORK_DIRS_DEBUG )
set(onetbb_TBB_tbb_FRAMEWORKS_DEBUG )
set(onetbb_TBB_tbb_DEPENDENCIES_DEBUG )
set(onetbb_TBB_tbb_SHARED_LINK_FLAGS_DEBUG )
set(onetbb_TBB_tbb_EXE_LINK_FLAGS_DEBUG )
set(onetbb_TBB_tbb_NO_SONAME_MODE_DEBUG FALSE)

# COMPOUND VARIABLES
set(onetbb_TBB_tbb_LINKER_FLAGS_DEBUG
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${onetbb_TBB_tbb_SHARED_LINK_FLAGS_DEBUG}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${onetbb_TBB_tbb_SHARED_LINK_FLAGS_DEBUG}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${onetbb_TBB_tbb_EXE_LINK_FLAGS_DEBUG}>
)
set(onetbb_TBB_tbb_COMPILE_OPTIONS_DEBUG
    "$<$<COMPILE_LANGUAGE:CXX>:${onetbb_TBB_tbb_COMPILE_OPTIONS_CXX_DEBUG}>"
    "$<$<COMPILE_LANGUAGE:C>:${onetbb_TBB_tbb_COMPILE_OPTIONS_C_DEBUG}>")