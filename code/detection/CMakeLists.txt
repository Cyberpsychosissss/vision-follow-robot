cmake_minimum_required(VERSION 2.6)

project(yolov5)

add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

find_package(CUDA REQUIRED)

# Jetson AGX Xavier = Volta sm_72 (CUDA 10.0 supports it)
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode;arch=compute_72,code=sm_72)

if(WIN32)
enable_language(CUDA)
endif(WIN32)

include_directories(${PROJECT_SOURCE_DIR}/include)
# cuda
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# tensorrt (aarch64 / Jetson JetPack)
include_directories(/usr/include/aarch64-linux-gnu/)
link_directories(/usr/lib/aarch64-linux-gnu/)
# Jetson DLA/NvMedia libs that libnvinfer.so depends on transitively
link_directories(/usr/lib/aarch64-linux-gnu/tegra/)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Ofast -Wfatal-errors")
set(TEGRA_RPATH "-Wl,-rpath-link,/usr/lib/aarch64-linux-gnu/tegra -Wl,-rpath,/usr/lib/aarch64-linux-gnu/tegra")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} ${TEGRA_RPATH}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${TEGRA_RPATH}")

cuda_add_library(myplugins SHARED ${PROJECT_SOURCE_DIR}/yololayer.cu)
target_link_libraries(myplugins nvinfer cudart)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(yolov5 ${PROJECT_SOURCE_DIR}/calibrator.cpp ${PROJECT_SOURCE_DIR}/yolov5.cpp)
target_link_libraries(yolov5 nvinfer)
target_link_libraries(yolov5 cudart)
target_link_libraries(yolov5 myplugins)
target_link_libraries(yolov5 ${OpenCV_LIBS})

# 跟随集成: GPU 检 person → 写 target.json
add_executable(yolo_follow ${PROJECT_SOURCE_DIR}/yolo_follow.cpp)
target_link_libraries(yolo_follow nvinfer cudart myplugins ${OpenCV_LIBS} pthread)

if(UNIX)
add_definitions(-O2 -pthread)
endif(UNIX)
