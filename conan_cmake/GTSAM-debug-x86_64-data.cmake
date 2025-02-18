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
set(gtsam_PACKAGE_FOLDER_DEBUG "/root/.conan2/p/b/gtsam1b524b6f7db93/p")
set(gtsam_BUILD_MODULES_PATHS_DEBUG )


set(gtsam_INCLUDE_DIRS_DEBUG "${gtsam_PACKAGE_FOLDER_DEBUG}/include")
set(gtsam_RES_DIRS_DEBUG )
set(gtsam_DEFINITIONS_DEBUG )
set(gtsam_SHARED_LINK_FLAGS_DEBUG )
set(gtsam_EXE_LINK_FLAGS_DEBUG )
set(gtsam_OBJECTS_DEBUG )
set(gtsam_COMPILE_DEFINITIONS_DEBUG )
set(gtsam_COMPILE_OPTIONS_C_DEBUG )
set(gtsam_COMPILE_OPTIONS_CXX_DEBUG )
set(gtsam_LIB_DIRS_DEBUG "${gtsam_PACKAGE_FOLDER_DEBUG}/lib")
set(gtsam_BIN_DIRS_DEBUG "${gtsam_PACKAGE_FOLDER_DEBUG}/bin")
set(gtsam_LIBRARY_TYPE_DEBUG SHARED)
set(gtsam_IS_HOST_WINDOWS_DEBUG 0)
set(gtsam_LIBS_DEBUG gtsam_unstableDebug CppUnitLiteDebug gtsamDebug)
set(gtsam_SYSTEM_LIBS_DEBUG )
set(gtsam_FRAMEWORK_DIRS_DEBUG )
set(gtsam_FRAMEWORKS_DEBUG )
set(gtsam_BUILD_DIRS_DEBUG )
set(gtsam_NO_SONAME_MODE_DEBUG FALSE)


# COMPOUND VARIABLES
set(gtsam_COMPILE_OPTIONS_DEBUG
    "$<$<COMPILE_LANGUAGE:CXX>:${gtsam_COMPILE_OPTIONS_CXX_DEBUG}>"
    "$<$<COMPILE_LANGUAGE:C>:${gtsam_COMPILE_OPTIONS_C_DEBUG}>")
set(gtsam_LINKER_FLAGS_DEBUG
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${gtsam_SHARED_LINK_FLAGS_DEBUG}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${gtsam_SHARED_LINK_FLAGS_DEBUG}>"
    "$<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${gtsam_EXE_LINK_FLAGS_DEBUG}>")


set(gtsam_COMPONENTS_DEBUG gtsam CppUnitLite gtsam_unstable)
########### COMPONENT gtsam_unstable VARIABLES ############################################

set(gtsam_gtsam_unstable_INCLUDE_DIRS_DEBUG "${gtsam_PACKAGE_FOLDER_DEBUG}/include")
set(gtsam_gtsam_unstable_LIB_DIRS_DEBUG "${gtsam_PACKAGE_FOLDER_DEBUG}/lib")
set(gtsam_gtsam_unstable_BIN_DIRS_DEBUG "${gtsam_PACKAGE_FOLDER_DEBUG}/bin")
set(gtsam_gtsam_unstable_LIBRARY_TYPE_DEBUG SHARED)
set(gtsam_gtsam_unstable_IS_HOST_WINDOWS_DEBUG 0)
set(gtsam_gtsam_unstable_RES_DIRS_DEBUG )
set(gtsam_gtsam_unstable_DEFINITIONS_DEBUG )
set(gtsam_gtsam_unstable_OBJECTS_DEBUG )
set(gtsam_gtsam_unstable_COMPILE_DEFINITIONS_DEBUG )
set(gtsam_gtsam_unstable_COMPILE_OPTIONS_C_DEBUG "")
set(gtsam_gtsam_unstable_COMPILE_OPTIONS_CXX_DEBUG "")
set(gtsam_gtsam_unstable_LIBS_DEBUG gtsam_unstableDebug)
set(gtsam_gtsam_unstable_SYSTEM_LIBS_DEBUG )
set(gtsam_gtsam_unstable_FRAMEWORK_DIRS_DEBUG )
set(gtsam_gtsam_unstable_FRAMEWORKS_DEBUG )
set(gtsam_gtsam_unstable_DEPENDENCIES_DEBUG gtsam)
set(gtsam_gtsam_unstable_SHARED_LINK_FLAGS_DEBUG )
set(gtsam_gtsam_unstable_EXE_LINK_FLAGS_DEBUG )
set(gtsam_gtsam_unstable_NO_SONAME_MODE_DEBUG FALSE)

# COMPOUND VARIABLES
set(gtsam_gtsam_unstable_LINKER_FLAGS_DEBUG
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${gtsam_gtsam_unstable_SHARED_LINK_FLAGS_DEBUG}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${gtsam_gtsam_unstable_SHARED_LINK_FLAGS_DEBUG}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${gtsam_gtsam_unstable_EXE_LINK_FLAGS_DEBUG}>
)
set(gtsam_gtsam_unstable_COMPILE_OPTIONS_DEBUG
    "$<$<COMPILE_LANGUAGE:CXX>:${gtsam_gtsam_unstable_COMPILE_OPTIONS_CXX_DEBUG}>"
    "$<$<COMPILE_LANGUAGE:C>:${gtsam_gtsam_unstable_COMPILE_OPTIONS_C_DEBUG}>")
########### COMPONENT CppUnitLite VARIABLES ############################################

set(gtsam_CppUnitLite_INCLUDE_DIRS_DEBUG "${gtsam_PACKAGE_FOLDER_DEBUG}/include")
set(gtsam_CppUnitLite_LIB_DIRS_DEBUG "${gtsam_PACKAGE_FOLDER_DEBUG}/lib")
set(gtsam_CppUnitLite_BIN_DIRS_DEBUG "${gtsam_PACKAGE_FOLDER_DEBUG}/bin")
set(gtsam_CppUnitLite_LIBRARY_TYPE_DEBUG SHARED)
set(gtsam_CppUnitLite_IS_HOST_WINDOWS_DEBUG 0)
set(gtsam_CppUnitLite_RES_DIRS_DEBUG )
set(gtsam_CppUnitLite_DEFINITIONS_DEBUG )
set(gtsam_CppUnitLite_OBJECTS_DEBUG )
set(gtsam_CppUnitLite_COMPILE_DEFINITIONS_DEBUG )
set(gtsam_CppUnitLite_COMPILE_OPTIONS_C_DEBUG "")
set(gtsam_CppUnitLite_COMPILE_OPTIONS_CXX_DEBUG "")
set(gtsam_CppUnitLite_LIBS_DEBUG CppUnitLiteDebug)
set(gtsam_CppUnitLite_SYSTEM_LIBS_DEBUG )
set(gtsam_CppUnitLite_FRAMEWORK_DIRS_DEBUG )
set(gtsam_CppUnitLite_FRAMEWORKS_DEBUG )
set(gtsam_CppUnitLite_DEPENDENCIES_DEBUG boost::boost)
set(gtsam_CppUnitLite_SHARED_LINK_FLAGS_DEBUG )
set(gtsam_CppUnitLite_EXE_LINK_FLAGS_DEBUG )
set(gtsam_CppUnitLite_NO_SONAME_MODE_DEBUG FALSE)

# COMPOUND VARIABLES
set(gtsam_CppUnitLite_LINKER_FLAGS_DEBUG
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${gtsam_CppUnitLite_SHARED_LINK_FLAGS_DEBUG}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${gtsam_CppUnitLite_SHARED_LINK_FLAGS_DEBUG}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${gtsam_CppUnitLite_EXE_LINK_FLAGS_DEBUG}>
)
set(gtsam_CppUnitLite_COMPILE_OPTIONS_DEBUG
    "$<$<COMPILE_LANGUAGE:CXX>:${gtsam_CppUnitLite_COMPILE_OPTIONS_CXX_DEBUG}>"
    "$<$<COMPILE_LANGUAGE:C>:${gtsam_CppUnitLite_COMPILE_OPTIONS_C_DEBUG}>")
########### COMPONENT gtsam VARIABLES ############################################

set(gtsam_gtsam_INCLUDE_DIRS_DEBUG "${gtsam_PACKAGE_FOLDER_DEBUG}/include")
set(gtsam_gtsam_LIB_DIRS_DEBUG "${gtsam_PACKAGE_FOLDER_DEBUG}/lib")
set(gtsam_gtsam_BIN_DIRS_DEBUG "${gtsam_PACKAGE_FOLDER_DEBUG}/bin")
set(gtsam_gtsam_LIBRARY_TYPE_DEBUG SHARED)
set(gtsam_gtsam_IS_HOST_WINDOWS_DEBUG 0)
set(gtsam_gtsam_RES_DIRS_DEBUG )
set(gtsam_gtsam_DEFINITIONS_DEBUG )
set(gtsam_gtsam_OBJECTS_DEBUG )
set(gtsam_gtsam_COMPILE_DEFINITIONS_DEBUG )
set(gtsam_gtsam_COMPILE_OPTIONS_C_DEBUG "")
set(gtsam_gtsam_COMPILE_OPTIONS_CXX_DEBUG "")
set(gtsam_gtsam_LIBS_DEBUG gtsamDebug)
set(gtsam_gtsam_SYSTEM_LIBS_DEBUG )
set(gtsam_gtsam_FRAMEWORK_DIRS_DEBUG )
set(gtsam_gtsam_FRAMEWORKS_DEBUG )
set(gtsam_gtsam_DEPENDENCIES_DEBUG Boost::chrono Boost::date_time Boost::filesystem Boost::program_options Boost::regex Boost::serialization Boost::system Boost::thread Boost::timer Eigen3::Eigen onetbb::onetbb metis::metis)
set(gtsam_gtsam_SHARED_LINK_FLAGS_DEBUG )
set(gtsam_gtsam_EXE_LINK_FLAGS_DEBUG )
set(gtsam_gtsam_NO_SONAME_MODE_DEBUG FALSE)

# COMPOUND VARIABLES
set(gtsam_gtsam_LINKER_FLAGS_DEBUG
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,SHARED_LIBRARY>:${gtsam_gtsam_SHARED_LINK_FLAGS_DEBUG}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,MODULE_LIBRARY>:${gtsam_gtsam_SHARED_LINK_FLAGS_DEBUG}>
        $<$<STREQUAL:$<TARGET_PROPERTY:TYPE>,EXECUTABLE>:${gtsam_gtsam_EXE_LINK_FLAGS_DEBUG}>
)
set(gtsam_gtsam_COMPILE_OPTIONS_DEBUG
    "$<$<COMPILE_LANGUAGE:CXX>:${gtsam_gtsam_COMPILE_OPTIONS_CXX_DEBUG}>"
    "$<$<COMPILE_LANGUAGE:C>:${gtsam_gtsam_COMPILE_OPTIONS_C_DEBUG}>")