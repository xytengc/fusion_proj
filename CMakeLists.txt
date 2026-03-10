cmake_minimum_required(VERSION 3.10)
project(Fusion_Proc)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_RKNN_SOURCE_INTEGRATION "Build with in-process RKNN YOLO source integration" ON)
set(RKNN_RUNTIME_LIB_DIR "${CMAKE_SOURCE_DIR}/rk3588_linux/lib" CACHE PATH "Path to RKNN runtime library directory")
set(RKNN_SDK_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../lubancat_ai_manual_code/example/3rdparty/rknpu2/include" CACHE PATH "Path to RKNN SDK include directory containing rknn_api.h")
set(RKNN_UTILS_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../lubancat_ai_manual_code/example/utils" CACHE PATH "Path to RKNN demo utils include directory containing common.h/image_utils.h")
set(RGA_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../lubancat_ai_manual_code/example/3rdparty/librga/include" CACHE PATH "Path to RGA include directory containing RgaUtils.h")

# 查找OpenCV
find_package(OpenCV REQUIRED)
# set(OpenCV_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../3rdparty/opencv/opencv-linux-aarch64/share/OpenCV)

# 启用OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# yolov5 files
set(rknpu_yolov5_file rknpu2/yolov5.cc)
if (TARGET_SOC STREQUAL "rv1106" OR TARGET_SOC STREQUAL "rv1103")
    add_definitions(-DRV1106_1103)
    set(rknpu_yolov5_file rknpu2/yolov5_rv1106_1103.cc)
elseif(TARGET_SOC STREQUAL "rk1808" OR TARGET_SOC STREQUAL "rv1109" OR TARGET_SOC STREQUAL "rv1126")
    add_definitions(-DRKNPU1)
    # dma
    include_directories(${CMAKE_SOURCE_DIR}/../lubancat_ai_manual_code/example/3rdparty/allocator/dma)
    set(rknpu_yolov5_file rknpu1/yolov5.cc)
endif()
# dma
include_directories(${CMAKE_SOURCE_DIR}/../lubancat_ai_manual_code/example/3rdparty/allocator/dma)

# RGA2 only support under 4G memory
if (ENABLE_DMA32)
    add_definitions(-DDMA_ALLOC_DMA32)
endif()
# support rknn zero copy api
if (ENABLE_ZERO_COPY)
    add_definitions(-DENABLE_ZERO_COPY)
endif()
add_subdirectory(${CMAKE_SOURCE_DIR}/../lubancat_ai_manual_code/example/3rdparty/ 3rdparty.out)
add_subdirectory(${CMAKE_SOURCE_DIR}/../lubancat_ai_manual_code/example/utils/ utils.out)

set(CMAKE_INSTALL_RPATH "$ORIGIN/lib")
# 包含头文件目录
include_directories(${CMAKE_SOURCE_DIR}/include)

# 添加源文件列表
file(GLOB SRC_FILES
    ${CMAKE_SOURCE_DIR}/src/*.cpp
)

# 添加可执行文件
add_executable(fusion_proc ${SRC_FILES})

if(ENABLE_RKNN_SOURCE_INTEGRATION)
    set(RKNN_REQUIRED_FILES
        "${CMAKE_SOURCE_DIR}/include/yolov5.h"
        "${CMAKE_SOURCE_DIR}/include/postprocess.h"
        "${CMAKE_SOURCE_DIR}/src/postprocess.cc"
        "${CMAKE_SOURCE_DIR}/rknpu2/yolov5.cc"
        "${RKNN_SDK_INCLUDE_DIR}/rknn_api.h"
        "${RKNN_UTILS_INCLUDE_DIR}/common.h"
        "${RKNN_UTILS_INCLUDE_DIR}/image_utils.h"
        "${RGA_INCLUDE_DIR}/RgaUtils.h"
    )

    foreach(req_file ${RKNN_REQUIRED_FILES})
        if(NOT EXISTS "${req_file}")
            message(FATAL_ERROR "Missing RKNN source dependency: ${req_file}")
        endif()
    endforeach()

    target_sources(fusion_proc PRIVATE
        "${CMAKE_SOURCE_DIR}/src/postprocess.cc"
        "${CMAKE_SOURCE_DIR}/rknpu2/yolov5.cc"
    )

    target_include_directories(fusion_proc PRIVATE
        "${CMAKE_SOURCE_DIR}"
        "${RKNN_SDK_INCLUDE_DIR}"
        "${RKNN_UTILS_INCLUDE_DIR}"
        "${RGA_INCLUDE_DIR}"
    )

    target_compile_definitions(fusion_proc PRIVATE ENABLE_RKNN_SOURCE_INTEGRATION=1)

    target_link_directories(fusion_proc PRIVATE "${RKNN_RUNTIME_LIB_DIR}")
    target_link_libraries(fusion_proc
        imageutils
        fileutils
        imagedrawing
        rknnrt
        rga
        dl
        pthread
    )

    if(EXISTS "${CMAKE_SOURCE_DIR}/rk3588_linux/model/classes_9_labels_list.txt")
        install(FILES "${CMAKE_SOURCE_DIR}/rk3588_linux/model/classes_9_labels_list.txt" DESTINATION model)
    endif()
    file(GLOB RKNN_FILES "${CMAKE_SOURCE_DIR}/rk3588_linux/model/*.rknn")
    if(RKNN_FILES)
        install(FILES ${RKNN_FILES} DESTINATION model)
    endif()
endif()

# 链接库
target_link_libraries(fusion_proc ${OpenCV_LIBS})

# 编译选项
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(fusion_proc PRIVATE -O3 -march=native -mtune=native)
endif()