########### AGGREGATED COMPONENTS AND DEPENDENCIES FOR THE MULTI CONFIG #####################
#############################################################################################

set(gklib_COMPONENT_NAMES "")
if(DEFINED gklib_FIND_DEPENDENCY_NAMES)
  list(APPEND gklib_FIND_DEPENDENCY_NAMES )
  list(REMOVE_DUPLICATES gklib_FIND_DEPENDENCY_NAMES)
else()
  set(gklib_FIND_DEPENDENCY_NAMES )
endif()

########### VARIABLES #######################################################################
#############################################################################################
set(gklib_PACKAGE_FOLDER_RELEASE "/root/.conan2/p/b/gklib959d1aac4a586/p")
set(gklib_BUILD_MODULES_PATHS_RELEASE )


set(gklib_INCLUDE_DIRS_RELEASE )
set(gklib_RES_DIRS_RELEASE )
set(gklib_DEFINITIONS_RELEASE )
set(gklib_SHARED_LINK_FLAGS_RELEASE )
set(gklib_EXE_LINK_FLAGS_RELEASE )
set(gklib_OBJECTS_RELEASE )
set(gklib_COMPILE_DEFINITIONS_RELEASE )
set(gklib_COMPILE_OPTIONS_C_RELEASE )
set(gklib_COMPILE_OPTIONS_CXX_RELEASE )
set(gklib_LIB_DIRS_RELEASE "${gklib_PACKAGE_FOLDER_RELEASE}/lib")
set(gklib_BIN_DIRS_RELEASE "${gklib_PACKAGE_FOLDER_RELEASE}/bin")
set(gklib_LIBRARY_TYPE_RELEASE SHARED)
set(gklib_IS_HOST_WINDOWS_RELEASE 0)
set(gklib_LIBS_RELEASE )
set(gklib_SYSTEM_LIBS_RELEASE m)
set(gklib_FRAMEWORK_DIRS_RELEASE )
set(gklib_FRAMEWORKS_RELEASE )
set(gklib_BUILD_DIRS_RELEASE )
set(gklib_NO_SONAME_MODE_RELEASE FALSE)


# COMPOUND VARIABLES
set(gklib_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${gklib_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${gklib_COMPILE_OPTIONS_C_RELEASE}>")
set(gklib_LINKER_FLAGS_RELEASE
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${gklib_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${gklib_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${gklib_EXE_LINK_FLAGS_RELEASE}>")


set(gklib_COMPONENTS_RELEASE )