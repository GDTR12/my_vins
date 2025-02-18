# Conan automatically generated toolchain file
# DO NOT EDIT MANUALLY, it will be overwritten

# Avoid including toolchain file several times (bad if appending to variables like
#   CMAKE_CXX_FLAGS. See https://github.com/android/ndk/issues/323
include_guard()
message(STATUS "Using Conan toolchain: ${CMAKE_CURRENT_LIST_FILE}")
if(${CMAKE_VERSION} VERSION_LESS "3.15")
    message(FATAL_ERROR "The 'CMakeToolchain' generator only works with CMake >= 3.15")
endif()

########## 'user_toolchain' block #############
# Include one or more CMake user toolchain from tools.cmake.cmaketoolchain:user_toolchain



########## 'generic_system' block #############
# Definition of system, platform and toolset





########## 'compilers' block #############



########## 'fpic' block #############
# Defining CMAKE_POSITION_INDEPENDENT_CODE for static libraries when necessary

message(STATUS "Conan toolchain: Setting CMAKE_POSITION_INDEPENDENT_CODE=ON (options.fPIC)")
set(CMAKE_POSITION_INDEPENDENT_CODE ON CACHE BOOL "Position independent code")


########## 'arch_flags' block #############
# Define C++ flags, C flags and linker flags from 'settings.arch'

message(STATUS "Conan toolchain: Defining architecture flag: -m64")
string(APPEND CONAN_CXX_FLAGS " -m64")
string(APPEND CONAN_C_FLAGS " -m64")
string(APPEND CONAN_SHARED_LINKER_FLAGS " -m64")
string(APPEND CONAN_EXE_LINKER_FLAGS " -m64")


########## 'libcxx' block #############
# Definition of libcxx from 'compiler.libcxx' setting, defining the
# right CXX_FLAGS for that libcxx



########## 'cppstd' block #############
# Define the C++ and C standards from 'compiler.cppstd' and 'compiler.cstd'

function(conan_modify_std_watch variable access value current_list_file stack)
    set(conan_watched_std_variable "17")
    if (${variable} STREQUAL "CMAKE_C_STANDARD")
        set(conan_watched_std_variable "")
    endif()
    if ("${access}" STREQUAL "MODIFIED_ACCESS" AND NOT "${value}" STREQUAL "${conan_watched_std_variable}")
        message(STATUS "Warning: Standard ${variable} value defined in conan_toolchain.cmake to ${conan_watched_std_variable} has been modified to ${value} by ${current_list_file}")
    endif()
    unset(conan_watched_std_variable)
endfunction()

message(STATUS "Conan toolchain: C++ Standard 17 with extensions OFF")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
variable_watch(CMAKE_CXX_STANDARD conan_modify_std_watch)


########## 'extra_flags' block #############
# Include extra C++, C and linker flags from configuration tools.build:<type>flags
# and from CMakeToolchain.extra_<type>_flags

# Conan conf flags start: 
# Conan conf flags end


########## 'cmake_flags_init' block #############
# Define CMAKE_<XXX>_FLAGS from CONAN_<XXX>_FLAGS

foreach(config IN LISTS CMAKE_CONFIGURATION_TYPES)
    string(TOUPPER ${config} config)
    if(DEFINED CONAN_CXX_FLAGS_${config})
      string(APPEND CMAKE_CXX_FLAGS_${config}_INIT " ${CONAN_CXX_FLAGS_${config}}")
    endif()
    if(DEFINED CONAN_C_FLAGS_${config})
      string(APPEND CMAKE_C_FLAGS_${config}_INIT " ${CONAN_C_FLAGS_${config}}")
    endif()
    if(DEFINED CONAN_SHARED_LINKER_FLAGS_${config})
      string(APPEND CMAKE_SHARED_LINKER_FLAGS_${config}_INIT " ${CONAN_SHARED_LINKER_FLAGS_${config}}")
    endif()
    if(DEFINED CONAN_EXE_LINKER_FLAGS_${config})
      string(APPEND CMAKE_EXE_LINKER_FLAGS_${config}_INIT " ${CONAN_EXE_LINKER_FLAGS_${config}}")
    endif()
endforeach()

if(DEFINED CONAN_CXX_FLAGS)
  string(APPEND CMAKE_CXX_FLAGS_INIT " ${CONAN_CXX_FLAGS}")
endif()
if(DEFINED CONAN_C_FLAGS)
  string(APPEND CMAKE_C_FLAGS_INIT " ${CONAN_C_FLAGS}")
endif()
if(DEFINED CONAN_SHARED_LINKER_FLAGS)
  string(APPEND CMAKE_SHARED_LINKER_FLAGS_INIT " ${CONAN_SHARED_LINKER_FLAGS}")
endif()
if(DEFINED CONAN_EXE_LINKER_FLAGS)
  string(APPEND CMAKE_EXE_LINKER_FLAGS_INIT " ${CONAN_EXE_LINKER_FLAGS}")
endif()


########## 'extra_variables' block #############
# Definition of extra CMake variables from tools.cmake.cmaketoolchain:extra_variables



########## 'try_compile' block #############
# Blocks after this one will not be added when running CMake try/checks

get_property( _CMAKE_IN_TRY_COMPILE GLOBAL PROPERTY IN_TRY_COMPILE )
if(_CMAKE_IN_TRY_COMPILE)
    message(STATUS "Running toolchain IN_TRY_COMPILE")
    return()
endif()


########## 'find_paths' block #############
# Define paths to find packages, programs, libraries, etc.
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/conan_cmakedeps_paths.cmake")
  message(STATUS "Conan toolchain: Including CMakeDeps generated conan_cmakedeps_paths.cmake")
  include("${CMAKE_CURRENT_LIST_DIR}/conan_cmakedeps_paths.cmake")
else()

set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)

# Definition of CMAKE_MODULE_PATH
# the generators folder (where conan generates files, like this toolchain)
list(PREPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

# Definition of CMAKE_PREFIX_PATH, CMAKE_XXXXX_PATH
# The Conan local "generators" folder, where this toolchain is saved.
list(PREPEND CMAKE_PREFIX_PATH ${CMAKE_CURRENT_LIST_DIR} )
list(PREPEND CMAKE_LIBRARY_PATH "/root/.conan2/p/b/ceres7559e225d63a0/p/lib" "/root/.conan2/p/b/gtsam1b524b6f7db93/p/lib" "/root/.conan2/p/b/boostb9624c9ff04a8/p/lib" "/root/.conan2/p/b/zlib1b8f81f2e5881/p/lib" "/root/.conan2/p/b/bzip25c07710f128fc/p/lib" "/root/.conan2/p/b/libba2b4b7bc0f0381/p/lib" "/root/.conan2/p/b/onetbfe249136fcbde/p/lib" "/root/.conan2/p/b/hwlocac4c0a1154c1a/p/lib" "/root/.conan2/p/b/metisd77a7b51d0f09/p/lib" "/root/.conan2/p/b/gklib14be21712bd5b/p/lib" "/root/.conan2/p/b/fmt1a579931855d1/p/lib")
list(PREPEND CMAKE_INCLUDE_PATH "/root/.conan2/p/b/ceres7559e225d63a0/p/include" "/root/.conan2/p/b/ceres7559e225d63a0/p/include/ceres" "/root/.conan2/p/b/ceres7559e225d63a0/p/include/ceres/internal/miniglog" "/root/.conan2/p/b/gtsam1b524b6f7db93/p/include" "/root/.conan2/p/b/boostb9624c9ff04a8/p/include" "/root/.conan2/p/b/zlib1b8f81f2e5881/p/include" "/root/.conan2/p/b/bzip25c07710f128fc/p/include" "/root/.conan2/p/b/libba2b4b7bc0f0381/p/include" "/root/.conan2/p/b/onetbfe249136fcbde/p/include" "/root/.conan2/p/b/hwlocac4c0a1154c1a/p/include" "/root/.conan2/p/b/metisd77a7b51d0f09/p/include" "/root/.conan2/p/b/gklib14be21712bd5b/p/include" "/root/.conan2/p/sophu16cffce94eaa4/p/include" "/root/.conan2/p/eigen3d88c0279cc26/p/include/eigen3" "/root/.conan2/p/b/fmt1a579931855d1/p/include")
set(CONAN_RUNTIME_LIB_DIRS "/root/.conan2/p/b/ceres7559e225d63a0/p/lib" "/root/.conan2/p/b/gtsam1b524b6f7db93/p/lib" "/root/.conan2/p/b/boostb9624c9ff04a8/p/lib" "/root/.conan2/p/b/zlib1b8f81f2e5881/p/lib" "/root/.conan2/p/b/bzip25c07710f128fc/p/lib" "/root/.conan2/p/b/libba2b4b7bc0f0381/p/lib" "/root/.conan2/p/b/onetbfe249136fcbde/p/lib" "/root/.conan2/p/b/hwlocac4c0a1154c1a/p/lib" "/root/.conan2/p/b/metisd77a7b51d0f09/p/lib" "/root/.conan2/p/b/gklib14be21712bd5b/p/lib" "/root/.conan2/p/b/fmt1a579931855d1/p/lib" )

endif()


########## 'pkg_config' block #############
# Define pkg-config from 'tools.gnu:pkg_config' executable and paths

if (DEFINED ENV{PKG_CONFIG_PATH})
set(ENV{PKG_CONFIG_PATH} "${CMAKE_CURRENT_LIST_DIR}:$ENV{PKG_CONFIG_PATH}")
else()
set(ENV{PKG_CONFIG_PATH} "${CMAKE_CURRENT_LIST_DIR}:")
endif()


########## 'rpath' block #############
# Defining CMAKE_SKIP_RPATH



########## 'shared' block #############
# Define BUILD_SHARED_LIBS for shared libraries

message(STATUS "Conan toolchain: Setting BUILD_SHARED_LIBS = ON")
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries")


########## 'output_dirs' block #############
# Definition of CMAKE_INSTALL_XXX folders

set(CMAKE_INSTALL_BINDIR "bin")
set(CMAKE_INSTALL_SBINDIR "bin")
set(CMAKE_INSTALL_LIBEXECDIR "bin")
set(CMAKE_INSTALL_LIBDIR "lib")
set(CMAKE_INSTALL_INCLUDEDIR "include")
set(CMAKE_INSTALL_OLDINCLUDEDIR "include")


########## 'variables' block #############
# Definition of CMake variables from CMakeToolchain.variables values

# Variables
# Variables  per configuration



########## 'preprocessor' block #############
# Preprocessor definitions from CMakeToolchain.preprocessor_definitions values

# Preprocessor definitions per configuration



if(CMAKE_POLICY_DEFAULT_CMP0091)  # Avoid unused and not-initialized warnings
endif()
