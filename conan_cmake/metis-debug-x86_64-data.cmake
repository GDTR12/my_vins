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
set(metis_PACKAGE_FOLDER_DEBUG "/root/.conan2/p/b/metisd77a7b51d0f09/p")
set(metis_BUILD_MODULES_PATHS_DEBUG )


set(metis_INCLUDE_DIRS_DEBUG "${metis_PACKAGE_FOLDER_DEBUG}/include")
set(metis_RES_DIRS_DEBUG )
set(metis_DEFINITIONS_DEBUG "-DLINUX"
			"-DIDXTYPEWIDTH=32"
			"-DREALTYPEWIDTH=32")
set(metis_SHARED_LINK_FLAGS_DEBUG )
set(metis_EXE_LINK_FLAGS_DEBUG )
set(metis_OBJECTS_DEBUG )
set(metis_COMPILE_DEFINITIONS_DEBUG "LINUX"
			"IDXTYPEWIDTH=32"
			"REALTYPEWIDTH=32")
set(metis_COMPILE_OPTIONS_C_DEBUG )
set(metis_COMPILE_OPTIONS_CXX_DEBUG )
set(metis_LIB_DIRS_DEBUG "${metis_PACKAGE_FOLDER_DEBUG}/lib")
set(metis_BIN_DIRS_DEBUG "${metis_PACKAGE_FOLDER_DEBUG}/bin")
set(metis_LIBRARY_TYPE_DEBUG SHARED)
set(metis_IS_HOST_WINDOWS_DEBUG 0)
set(metis_LIBS_DEBUG metis)
set(metis_SYSTEM_LIBS_DEBUG m)
set(metis_FRAMEWORK_DIRS_DEBUG )
set(metis_FRAMEWORKS_DEBUG )
set(metis_BUILD_DIRS_DEBUG )
set(metis_NO_SONAME_MODE_DEBUG FALSE)


# COMPOUND VARIABLES
set(metis_COMPILE_OPTIONS_DEBUG
    "$<$<COMPILE_LANGUAGE:CXX>:${metis_COMPILE_OPTIONS_CXX_DEBUG}>"
    "$<$<COMPILE_LANGUAGE:C>:${metis_COMPILE_OPTIONS_C_DEBUG}>")
set(metis_LINKER_FLAGS_DEBUG
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${metis_SHARED_LINK_FLAGS_DEBUG}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${metis_SHARED_LINK_FLAGS_DEBUG}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${metis_EXE_LINK_FLAGS_DEBUG}>")


set(metis_COMPONENTS_DEBUG )