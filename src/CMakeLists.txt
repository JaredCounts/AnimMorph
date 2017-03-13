set(BIN_NAME AnimMorph)

file(GLOB_RECURSE CPP_FILES
    "*.cpp"
)

file(GLOB_RECURSE CPP_HEADERS
	"*.h"
)

set(LIBS_USED
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARIES}
    )

if (CMAKE_SYSTEM_NAME STREQUAL Linux)                        
    set(LIBS_USED ${LIBS_USED} dl)                           
endif()                                                      

add_executable(${BIN_NAME} ${CPP_FILES} ${CPP_HEADERS})

target_link_libraries(${BIN_NAME} ${LIBS_USED})

install(TARGETS ${BIN_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX})

function(assign_source_group)
    foreach(_source IN ITEMS ${ARGN})
        if (IS_ABSOLUTE "${_source}")
            file(RELATIVE_PATH _source_rel "${CMAKE_CURRENT_SOURCE_DIR}" "${_source}")
        else()
            set(source_rel "${_source}")
        endif()
        get_filename_component(_source_path "${_source_rel}" PATH)
        string(REPLACE "/" "\\" _source_path_msvc "${_source_path}")
        source_group("src\\${_source_path_msvc}" FILES "${_source}")
    endforeach()
endfunction(assign_source_group)

assign_source_group(${CPP_FILES})
assign_source_group(${CPP_HEADERS})
