cmake_minimum_required(VERSION 3.15.1)
project(my_vins)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
set(g2o_DIR "/root/workspace/packages/g2o/install/lib/cmake/g2o/")
find_package(g2o REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

set(MY_PROJECT_INCLUDE  
  "./src"
  ${EIGEN3_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(MY_PROJECT_LIBS_INCLUDE
)
message(${EIGEN3_INCLUDE_DIR}=====================)

set(MY_PROJECT_LIBS
  yaml-cpp
  ${OpenCV_LIBS}
)
set(AMENT_TARGET_DEPS
  "rclcpp"
  "Eigen3"
  "g2o"
  "sensor_msgs"
  "cv_bridge"
)

file(GLOB_RECURSE LIBRARY_SRCS "./src/*.c*")
file(GLOB_RECURSE NODE_SRCS "./src/app/*.cpp")
list(REMOVE_ITEM LIBRARY_SRCS ${NODE_SRCS})

if(NOT DEFINED CMAKE_NODE_NAME)
  set(CMAKE_NODE_NAME "all")
endif(NOT DEFINED CMAKE_NODE_NAME)

if(${CMAKE_NODE_NAME} STREQUAL "all")
  message("=========================")
  message("[Make]: all node")
  message("=========================")
  foreach(NODE_PATH ${NODE_SRCS})
    # message("node path:  ${NODE_PATH}")
    get_filename_component(NODE_FILE ${NODE_PATH} NAME_WE)
    # message("node file:  ${NODE_FILE}")
    add_executable(${NODE_FILE} ${LIBRARY_SRCS} ${NODE_PATH})
    target_include_directories(${NODE_FILE} PUBLIC ${MY_PROJECT_INCLUDE})
    target_link_directories(${NODE_FILE} PUBLIC ${MY_PROJECT_LIBS_INCLUDE})
    target_link_libraries(${NODE_FILE} ${MY_PROJECT_LIBS})
    ament_target_dependencies(${NODE_FILE} ${AMENT_TARGET_DEPS})

    target_compile_features(${NODE_FILE} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
    install(TARGETS ${NODE_FILE} 
      DESTINATION lib/${PROJECT_NAME}) 
  endforeach()
else()
  message("=========================")
  message("[Make]: ${CMAKE_NODE_NAME}")
  message("=========================")
  foreach(NODE_PATH ${NODE_SRCS})
    get_filename_component(NODE_FILE ${NODE_PATH} NAME_WE)
    if(${NODE_FILE} STREQUAL ${CMAKE_NODE_NAME})
      add_executable(${CMAKE_NODE_NAME}  ${LIBRARY_SRCS} ${NODE_PATH})
      target_include_directories(${CMAKE_NODE_NAME} PUBLIC ${MY_PROJECT_INCLUDE})
      target_link_directories(${CMAKE_NODE_NAME} PUBLIC ${MY_PROJECT_LIBS_INCLUDE})
      target_link_libraries(${CMAKE_NODE_NAME} ${MY_PROJECT_LIBS})
      ament_target_dependencies(${CMAKE_NODE_NAME} ${AMENT_TARGET_DEPS})

      target_compile_features(${CMAKE_NODE_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
      install(TARGETS ${CMAKE_NODE_NAME} 
        DESTINATION lib/${PROJECT_NAME})    
    endif()
  endforeach()
endif()

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()