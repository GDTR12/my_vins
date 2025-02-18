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
set(gklib_PACKAGE_FOLDER_DEBUG "/root/.conan2/p/b/gklib14be21712bd5b/p")
set(gklib_BUILD_MODULES_PATHS_DEBUG )


set(gklib_INCLUDE_DIRS_DEBUG )
set(gklib_RES_DIRS_DEBUG )
set(gklib_DEFINITIONS_DEBUG )
set(gklib_SHARED_LINK_FLAGS_DEBUG )
set(gklib_EXE_LINK_FLAGS_DEBUG )
set(gklib_OBJECTS_DEBUG )
set(gklib_COMPILE_DEFINITIONS_DEBUG )
set(gklib_COMPILE_OPTIONS_C_DEBUG )
set(gklib_COMPILE_OPTIONS_CXX_DEBUG )
set(gklib_LIB_DIRS_DEBUG "${gklib_PACKAGE_FOLDER_DEBUG}/lib")
set(gklib_BIN_DIRS_DEBUG "${gklib_PACKAGE_FOLDER_DEBUG}/bin")
set(gklib_LIBRARY_TYPE_DEBUG SHARED)
set(gklib_IS_HOST_WINDOWS_DEBUG 0)
set(gklib_LIBS_DEBUG )
set(gklib_SYSTEM_LIBS_DEBUG m)
set(gklib_FRAMEWORK_DIRS_DEBUG )
set(gklib_FRAMEWORKS_DEBUG )
set(gklib_BUILD_DIRS_DEBUG )
set(gklib_NO_SONAME_MODE_DEBUG FALSE)


# COMPOUND VARIABLES
set(gklib_COMPILE_OPTIONS_DEBUG
    "$<$<COMPILE_LANGUAGE:CXX>:${gklib_COMPILE_OPTIONS_CXX_DEBUG}>"
    "$<$<COMPILE_LANGUAGE:C>:${gklib_COMPILE_OPTIONS_C_DEBUG}>")
set(gklib_LINKER_FLAGS_DEBUG
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${gklib_SHARED_LINK_FLAGS_DEBUG}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${gklib_SHARED_LINK_FLAGS_DEBUG}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${gklib_EXE_LINK_FLAGS_DEBUG}>")


set(gklib_COMPONENTS_DEBUG )