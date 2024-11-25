cmake_minimum_required(VERSION 3.5)
project(ZED_Positional_Tracking)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_BUILD_TYPE "Release")

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if(NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

find_package(ZED REQUIRED)
message(STATUS "ZED_VERSION: " ${ZED_VERSION})
find_package(CUDA ${ZED_CUDA_VERSION} REQUIRED)
message(STATUS "CUDA_VERSION: " ${CUDA_VERSION})
find_package(GLUT REQUIRED)
message(STATUS "GLUT_VERSION: " ${Glut_VERSION})

find_package(GLEW REQUIRED)
set(OpenGL_GL_PREFERENCE GLVND)
message(STATUS "GLEW_VERSION: " ${GLEW_VERSION})

find_package(OpenGL REQUIRED)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV_VERSION: " ${OpenCV_VERSION})
message("")

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/positional_tracking/positional_tracking/cpp/include
    ${ZED_INCLUDE_DIRS}
    ${GLEW_INCLUDE_DIRS}
    ${GLUT_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})
# CMAKE_CURRENT_SOURCE_DIR: /home/dsg/zed-sdk/positional_tracking/positional_tracking/cpp
message("CMAKE_CURRENT_SOURCE_DIR: " ${CMAKE_CURRENT_SOURCE_DIR})
message("PROJECT_SOURCE_DIR: " ${PROJECT_SOURCE_DIR})

set(SRC_FILES
    src/GLViewer.cpp
    src/main.cpp)
set(HDR_FILES
    include/GLViewer.hpp
    include/ZEDModel.hpp)

add_executable(ZED_Positional_Tracking ${HDR_FILES} ${SRC_FILES})

IF(NOT WIN32)
    IF(BUILD_WITH_SANITIZER)
        message("!! Building with address sanitizer and -g !!")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer -fsanitize=address -Wall -Wextra -g")
        set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fno-omit-frame-pointer -fsanitize=address -Wall -Wextra -g")
        add_definitions(-g -fsanitize=address -fno-omit-frame-pointer -static-libasan -Wall -Wextra)
    ENDIF()

    IF(BUILD_WITH_DEBUGINFOS)
        message("!! Building with -g !!")
        add_definitions(-g)
    ENDIF()
ENDIF()

if(LINK_SHARED_ZED)
    SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

target_link_libraries(${PROJECT_NAME}
    ${ZED_LIBS}
    ${OPENGL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${GLUT_LIBRARIES}
    ${GLEW_LIBRARIES})

if(INSTALL_SAMPLES)
    LIST(APPEND SAMPLE_LIST ${PROJECT_NAME})
    SET(SAMPLE_LIST "${SAMPLE_LIST}" PARENT_SCOPE)
endif()
