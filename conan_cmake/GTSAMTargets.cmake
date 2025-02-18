# Load the debug and release variables
file(GLOB DATA_FILES "${CMAKE_CURRENT_LIST_DIR}/GTSAM-*-data.cmake")

foreach(f ${DATA_FILES})
    include(${f})
endforeach()

# Create the targets for all the components
foreach(_COMPONENT ${gtsam_COMPONENT_NAMES} )
    if(NOT TARGET ${_COMPONENT})
        add_library(${_COMPONENT} INTERFACE IMPORTED)
        message(${GTSAM_MESSAGE_MODE} "Conan: Component target declared '${_COMPONENT}'")
    endif()
endforeach()

if(NOT TARGET gtsam::gtsam)
    add_library(gtsam::gtsam INTERFACE IMPORTED)
    message(${GTSAM_MESSAGE_MODE} "Conan: Target declared 'gtsam::gtsam'")
endif()
# Load the debug and release library finders
file(GLOB CONFIG_FILES "${CMAKE_CURRENT_LIST_DIR}/GTSAM-Target-*.cmake")

foreach(f ${CONFIG_FILES})
    include(${f})
endforeach()