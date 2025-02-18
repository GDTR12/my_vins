########### AGGREGATED COMPONENTS AND DEPENDENCIES FOR THE MULTI CONFIG #####################
#############################################################################################

set(metis_COMPONENT_NAMES "")
if(DEFINED metis_FIND_DEPENDENCY_NAMES)
  list(APPEND metis_FIND_DEPENDENCY_NAMES gklib)
  list(REMOVE_DUPLICATES metis_FIND_DEPENDENCY_NAMES)
else()
  set(metis_FIND_DEPENDENCY_NAMES gklib)
endif()
set(gklib_FIND_MODE "NO_MODULE")

########### VARIABLES #######################################################################
#############################################################################################
set(metis_PACKAGE_FOLDER_RELEASE "/root/.conan2/p/b/metis56d01366a7dba/p")
set(metis_BUILD_MODULES_PATHS_RELEASE )


set(metis_INCLUDE_DIRS_RELEASE "${metis_PACKAGE_FOLDER_RELEASE}/include")
set(metis_RES_DIRS_RELEASE )
set(metis_DEFINITIONS_RELEASE "-DLINUX"
			"-DIDXTYPEWIDTH=32"
			"-DREALTYPEWIDTH=32")
set(metis_SHARED_LINK_FLAGS_RELEASE )
set(metis_EXE_LINK_FLAGS_RELEASE )
set(metis_OBJECTS_RELEASE )
set(metis_COMPILE_DEFINITIONS_RELEASE "LINUX"
			"IDXTYPEWIDTH=32"
			"REALTYPEWIDTH=32")
set(metis_COMPILE_OPTIONS_C_RELEASE )
set(metis_COMPILE_OPTIONS_CXX_RELEASE )
set(metis_LIB_DIRS_RELEASE "${metis_PACKAGE_FOLDER_RELEASE}/lib")
set(metis_BIN_DIRS_RELEASE "${metis_PACKAGE_FOLDER_RELEASE}/bin")
set(metis_LIBRARY_TYPE_RELEASE SHARED)
set(metis_IS_HOST_WINDOWS_RELEASE 0)
set(metis_LIBS_RELEASE metis)
set(metis_SYSTEM_LIBS_RELEASE m)
set(metis_FRAMEWORK_DIRS_RELEASE )
set(metis_FRAMEWORKS_RELEASE )
set(metis_BUILD_DIRS_RELEASE )
set(metis_NO_SONAME_MODE_RELEASE FALSE)


# COMPOUND VARIABLES
set(metis_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${metis_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${metis_COMPILE_OPTIONS_C_RELEASE}>")
set(metis_LINKER_FLAGS_RELEASE
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${metis_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${metis_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${metis_EXE_LINK_FLAGS_RELEASE}>")


set(metis_COMPONENTS_RELEASE )