########### AGGREGATED COMPONENTS AND DEPENDENCIES FOR THE MULTI CONFIG #####################
#############################################################################################

list(APPEND gtsam_COMPONENT_NAMES gtsam CppUnitLite gtsam_unstable)
list(REMOVE_DUPLICATES gtsam_COMPONENT_NAMES)
if(DEFINED gtsam_FIND_DEPENDENCY_NAMES)
  list(APPEND gtsam_FIND_DEPENDENCY_NAMES Boost TBB metis Eigen3)
  list(REMOVE_DUPLICATES gtsam_FIND_DEPENDENCY_NAMES)
else()
  set(gtsam_FIND_DEPENDENCY_NAMES Boost TBB metis Eigen3)
endif()
set(Boost_FIND_MODE "NO_MODULE")
set(TBB_FIND_MODE "NO_MODULE")
set(metis_FIND_MODE "NO_MODULE")
set(Eigen3_FIND_MODE "NO_MODULE")

########### VARIABLES #######################################################################
#############################################################################################
set(gtsam_PACKAGE_FOLDER_RELEASE "/root/.conan2/p/b/gtsamed2e2c7b741a1/p")
set(gtsam_BUILD_MODULES_PATHS_RELEASE )


set(gtsam_INCLUDE_DIRS_RELEASE "${gtsam_PACKAGE_FOLDER_RELEASE}/include")
set(gtsam_RES_DIRS_RELEASE )
set(gtsam_DEFINITIONS_RELEASE )
set(gtsam_SHARED_LINK_FLAGS_RELEASE )
set(gtsam_EXE_LINK_FLAGS_RELEASE )
set(gtsam_OBJECTS_RELEASE )
set(gtsam_COMPILE_DEFINITIONS_RELEASE )
set(gtsam_COMPILE_OPTIONS_C_RELEASE )
set(gtsam_COMPILE_OPTIONS_CXX_RELEASE )
set(gtsam_LIB_DIRS_RELEASE "${gtsam_PACKAGE_FOLDER_RELEASE}/lib")
set(gtsam_BIN_DIRS_RELEASE "${gtsam_PACKAGE_FOLDER_RELEASE}/bin")
set(gtsam_LIBRARY_TYPE_RELEASE SHARED)
set(gtsam_IS_HOST_WINDOWS_RELEASE 0)
set(gtsam_LIBS_RELEASE gtsam_unstable CppUnitLite gtsam)
set(gtsam_SYSTEM_LIBS_RELEASE )
set(gtsam_FRAMEWORK_DIRS_RELEASE )
set(gtsam_FRAMEWORKS_RELEASE )
set(gtsam_BUILD_DIRS_RELEASE )
set(gtsam_NO_SONAME_MODE_RELEASE FALSE)


# COMPOUND VARIABLES
set(gtsam_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${gtsam_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${gtsam_COMPILE_OPTIONS_C_RELEASE}>")
set(gtsam_LINKER_FLAGS_RELEASE
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${gtsam_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${gtsam_SHARED_LINK_FLAGS_RELEASE}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${gtsam_EXE_LINK_FLAGS_RELEASE}>")


set(gtsam_COMPONENTS_RELEASE gtsam CppUnitLite gtsam_unstable)
########### COMPONENT gtsam_unstable VARIABLES ############################################

set(gtsam_gtsam_unstable_INCLUDE_DIRS_RELEASE "${gtsam_PACKAGE_FOLDER_RELEASE}/include")
set(gtsam_gtsam_unstable_LIB_DIRS_RELEASE "${gtsam_PACKAGE_FOLDER_RELEASE}/lib")
set(gtsam_gtsam_unstable_BIN_DIRS_RELEASE "${gtsam_PACKAGE_FOLDER_RELEASE}/bin")
set(gtsam_gtsam_unstable_LIBRARY_TYPE_RELEASE SHARED)
set(gtsam_gtsam_unstable_IS_HOST_WINDOWS_RELEASE 0)
set(gtsam_gtsam_unstable_RES_DIRS_RELEASE )
set(gtsam_gtsam_unstable_DEFINITIONS_RELEASE )
set(gtsam_gtsam_unstable_OBJECTS_RELEASE )
set(gtsam_gtsam_unstable_COMPILE_DEFINITIONS_RELEASE )
set(gtsam_gtsam_unstable_COMPILE_OPTIONS_C_RELEASE "")
set(gtsam_gtsam_unstable_COMPILE_OPTIONS_CXX_RELEASE "")
set(gtsam_gtsam_unstable_LIBS_RELEASE gtsam_unstable)
set(gtsam_gtsam_unstable_SYSTEM_LIBS_RELEASE )
set(gtsam_gtsam_unstable_FRAMEWORK_DIRS_RELEASE )
set(gtsam_gtsam_unstable_FRAMEWORKS_RELEASE )
set(gtsam_gtsam_unstable_DEPENDENCIES_RELEASE gtsam)
set(gtsam_gtsam_unstable_SHARED_LINK_FLAGS_RELEASE )
set(gtsam_gtsam_unstable_EXE_LINK_FLAGS_RELEASE )
set(gtsam_gtsam_unstable_NO_SONAME_MODE_RELEASE FALSE)

# COMPOUND VARIABLES
set(gtsam_gtsam_unstable_LINKER_FLAGS_RELEASE
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${gtsam_gtsam_unstable_SHARED_LINK_FLAGS_RELEASE}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${gtsam_gtsam_unstable_SHARED_LINK_FLAGS_RELEASE}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${gtsam_gtsam_unstable_EXE_LINK_FLAGS_RELEASE}>
)
set(gtsam_gtsam_unstable_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${gtsam_gtsam_unstable_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${gtsam_gtsam_unstable_COMPILE_OPTIONS_C_RELEASE}>")
########### COMPONENT CppUnitLite VARIABLES ############################################

set(gtsam_CppUnitLite_INCLUDE_DIRS_RELEASE "${gtsam_PACKAGE_FOLDER_RELEASE}/include")
set(gtsam_CppUnitLite_LIB_DIRS_RELEASE "${gtsam_PACKAGE_FOLDER_RELEASE}/lib")
set(gtsam_CppUnitLite_BIN_DIRS_RELEASE "${gtsam_PACKAGE_FOLDER_RELEASE}/bin")
set(gtsam_CppUnitLite_LIBRARY_TYPE_RELEASE SHARED)
set(gtsam_CppUnitLite_IS_HOST_WINDOWS_RELEASE 0)
set(gtsam_CppUnitLite_RES_DIRS_RELEASE )
set(gtsam_CppUnitLite_DEFINITIONS_RELEASE )
set(gtsam_CppUnitLite_OBJECTS_RELEASE )
set(gtsam_CppUnitLite_COMPILE_DEFINITIONS_RELEASE )
set(gtsam_CppUnitLite_COMPILE_OPTIONS_C_RELEASE "")
set(gtsam_CppUnitLite_COMPILE_OPTIONS_CXX_RELEASE "")
set(gtsam_CppUnitLite_LIBS_RELEASE CppUnitLite)
set(gtsam_CppUnitLite_SYSTEM_LIBS_RELEASE )
set(gtsam_CppUnitLite_FRAMEWORK_DIRS_RELEASE )
set(gtsam_CppUnitLite_FRAMEWORKS_RELEASE )
set(gtsam_CppUnitLite_DEPENDENCIES_RELEASE boost::boost)
set(gtsam_CppUnitLite_SHARED_LINK_FLAGS_RELEASE )
set(gtsam_CppUnitLite_EXE_LINK_FLAGS_RELEASE )
set(gtsam_CppUnitLite_NO_SONAME_MODE_RELEASE FALSE)

# COMPOUND VARIABLES
set(gtsam_CppUnitLite_LINKER_FLAGS_RELEASE
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${gtsam_CppUnitLite_SHARED_LINK_FLAGS_RELEASE}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${gtsam_CppUnitLite_SHARED_LINK_FLAGS_RELEASE}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${gtsam_CppUnitLite_EXE_LINK_FLAGS_RELEASE}>
)
set(gtsam_CppUnitLite_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${gtsam_CppUnitLite_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${gtsam_CppUnitLite_COMPILE_OPTIONS_C_RELEASE}>")
########### COMPONENT gtsam VARIABLES ############################################

set(gtsam_gtsam_INCLUDE_DIRS_RELEASE "${gtsam_PACKAGE_FOLDER_RELEASE}/include")
set(gtsam_gtsam_LIB_DIRS_RELEASE "${gtsam_PACKAGE_FOLDER_RELEASE}/lib")
set(gtsam_gtsam_BIN_DIRS_RELEASE "${gtsam_PACKAGE_FOLDER_RELEASE}/bin")
set(gtsam_gtsam_LIBRARY_TYPE_RELEASE SHARED)
set(gtsam_gtsam_IS_HOST_WINDOWS_RELEASE 0)
set(gtsam_gtsam_RES_DIRS_RELEASE )
set(gtsam_gtsam_DEFINITIONS_RELEASE )
set(gtsam_gtsam_OBJECTS_RELEASE )
set(gtsam_gtsam_COMPILE_DEFINITIONS_RELEASE )
set(gtsam_gtsam_COMPILE_OPTIONS_C_RELEASE "")
set(gtsam_gtsam_COMPILE_OPTIONS_CXX_RELEASE "")
set(gtsam_gtsam_LIBS_RELEASE gtsam)
set(gtsam_gtsam_SYSTEM_LIBS_RELEASE )
set(gtsam_gtsam_FRAMEWORK_DIRS_RELEASE )
set(gtsam_gtsam_FRAMEWORKS_RELEASE )
set(gtsam_gtsam_DEPENDENCIES_RELEASE Boost::chrono Boost::date_time Boost::filesystem Boost::program_options Boost::regex Boost::serialization Boost::system Boost::thread Boost::timer Eigen3::Eigen onetbb::onetbb metis::metis)
set(gtsam_gtsam_SHARED_LINK_FLAGS_RELEASE )
set(gtsam_gtsam_EXE_LINK_FLAGS_RELEASE )
set(gtsam_gtsam_NO_SONAME_MODE_RELEASE FALSE)

# COMPOUND VARIABLES
set(gtsam_gtsam_LINKER_FLAGS_RELEASE
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${gtsam_gtsam_SHARED_LINK_FLAGS_RELEASE}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${gtsam_gtsam_SHARED_LINK_FLAGS_RELEASE}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${gtsam_gtsam_EXE_LINK_FLAGS_RELEASE}>
)
set(gtsam_gtsam_COMPILE_OPTIONS_RELEASE
    "$<$<COMPILE_LANGUAGE:CXX>:${gtsam_gtsam_COMPILE_OPTIONS_CXX_RELEASE}>"
    "$<$<COMPILE_LANGUAGE:C>:${gtsam_gtsam_COMPILE_OPTIONS_C_RELEASE}>")