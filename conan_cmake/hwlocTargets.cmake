# Load the debug and release variables
file(GLOB DATA_FILES "${CMAKE_CURRENT_LIST_DIR}/hwloc-*-data.cmake")

foreach(f ${DATA_FILES})
    include(${f})
endforeach()

# Create the targets for all the components
foreach(_COMPONENT ${hwloc_COMPONENT_NAMES} )
    if(NOT TARGET ${_COMPONENT})
        add_library(${_COMPONENT} INTERFACE IMPORTED)
        message(${hwloc_MESSAGE_MODE} "Conan: Component target declared '${_COMPONENT}'")
    endif()
endforeach()

if(NOT TARGET hwloc::hwloc)
    add_library(hwloc::hwloc INTERFACE IMPORTED)
    message(${hwloc_MESSAGE_MODE} "Conan: Target declared 'hwloc::hwloc'")
endif()
# Load the debug and release library finders
file(GLOB CONFIG_FILES "${CMAKE_CURRENT_LIST_DIR}/hwloc-Target-*.cmake")

foreach(f ${CONFIG_FILES})
    include(${f})
endforeach()