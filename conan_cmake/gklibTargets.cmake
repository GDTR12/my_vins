# Load the debug and release variables
file(GLOB DATA_FILES "${CMAKE_CURRENT_LIST_DIR}/gklib-*-data.cmake")

foreach(f ${DATA_FILES})
    include(${f})
endforeach()

# Create the targets for all the components
foreach(_COMPONENT ${gklib_COMPONENT_NAMES} )
    if(NOT TARGET ${_COMPONENT})
        add_library(${_COMPONENT} INTERFACE IMPORTED)
        message(${gklib_MESSAGE_MODE} "Conan: Component target declared '${_COMPONENT}'")
    endif()
endforeach()

if(NOT TARGET gklib::gklib)
    add_library(gklib::gklib INTERFACE IMPORTED)
    message(${gklib_MESSAGE_MODE} "Conan: Target declared 'gklib::gklib'")
endif()
# Load the debug and release library finders
file(GLOB CONFIG_FILES "${CMAKE_CURRENT_LIST_DIR}/gklib-Target-*.cmake")

foreach(f ${CONFIG_FILES})
    include(${f})
endforeach()