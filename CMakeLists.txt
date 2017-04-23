cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0015 NEW)

# ==================
# Required libraries...
# (Fill these in!)

# Eigen3 // for linear math
set(EIGEN3_LOCATION ${CMAKE_SOURCE_DIR}/../../../../Libraries/eigen/build17/install)

# glut/freeglut // for rendering
set(GLUT_ROOT_PATH ${CMAKE_SOURCE_DIR}/../../../../Libraries/freeglut-3.0.0/build)

# RapidXML // XML (svg) parser
set(RAPIDXML_DIR ${CMAKE_SOURCE_DIR}/../../../../Libraries/rapidxml-1.13)

# ==================

# Project name
project(AnimMorph)

# our project's targets is organized by folders
set_property( GLOBAL PROPERTY USE_FOLDERS ON)

# tell MSVC to use static runtime libraries, not dynamic
if (MSVC)
    SET (
      CMAKE_CXX_FLAGS_DEBUG
        "${CMAKE_CXX_FLAGS_DEBUG} /MTd"
    )
    SET (
      CMAKE_CXX_FLAGS_RELEASE
        "${CMAKE_CXX_FLAGS_RELEASE} /MT"
    )
endif(MSVC)

# EIGEN
# =====
include_directories(${EIGEN3_LOCATION}/lib/cmake/eigen3)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

# OPENGL
# ======
find_package(OpenGL REQUIRED)

# GLFW
# ----
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

set(GLFW_LOCATION ${CMAKE_SOURCE_DIR}/../../../../Libraries/glfw-3.2.1/install)

find_package(GLFW REQUIRED)

include_directories(${GLFW_INCLUDE_DIR})
link_libraries(${GLFW_LIBRARIES})

# GLEW
# ----

set(GLEW_LOCATION ${CMAKE_SOURCE_DIR}/../../../../Libraries/glew-2.0.0/build/install)

find_package(GLEW REQUIRED)

MESSAGE(STATUS ${GLEW_INCLUDE_DIR})
include_directories(${GLEW_INCLUDE_DIR})
link_libraries(${GLEW_LIBRARY})

# GLUT
# ====
# find_package(GLUT REQUIRED)

# # xxx hacky -- FindGLUT for Windows doesn't play well with MSVC
# if (WIN32)
#     find_path( GLUT_INCLUDE_DIR NAMES GL/glut.h
#         PATHS  ${GLUT_ROOT_PATH}/../include )
# 
#     find_library( GLUT_LIBRARY NAMES glut glut32 freeglut
#         PATHS
#         ${OPENGL_LIBRARY_DIR}
#         ${GLUT_ROOT_PATH}/lib/Release
#         )
# 
#     find_library(GLUT_LIBRARY_DEBUG NAMES glutd glut32d freeglutd
#         PATHS
#         ${OPENGL_LIBRARY_DIR}
#         ${GLUT_ROOT_PATH}/lib/Debug
#         )
# 
#     set(GLUT_LIBRARIES ${GLUT_LIBRARY} debug ${GLUT_LIBRARY_DEBUG})
# 
# else ()
#     find_package(GLUT REQUIRED)
# 
# endif (WIN32)
# 
# include_directories(${GLUT_INCLUDE_DIR})
# link_libraries(${GLUT_LIBRARIES})

# RapidXML
# ========
include_directories(${RAPIDXML_DIR})

# Install directory
# =================
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    get_filename_component(CMAKE_SOURCE_DIR "." REALPATH)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/inst" CACHE PATH "install prefix" FORCE)
endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

# Source
# ======
add_subdirectory(src)
