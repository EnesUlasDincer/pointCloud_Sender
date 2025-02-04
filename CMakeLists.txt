# Minimum required cmake version
cmake_minimum_required(VERSION 3.10)
project(OBPointCloud)

# Find required packages
find_package(OpenCV REQUIRED)
find_package(cppzmq REQUIRED)
find_package(PCL REQUIRED)
find_package(CUDA REQUIRED)
find_package(CUDAToolkit REQUIRED)

#export CC=/usr/bin/gcc-11
#export CXX=/usr/bin/g++-11


# Enable OpenMP
find_package(OpenMP REQUIRED)

# CUDA configuration
set(CUDA_NVCC_FLAGS "-arch=compute_75 --expt-relaxed-constexpr" CACHE STRING "nvcc flags" FORCE)
set(CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
set(LIB_TYPE STATIC)

cmake_policy(SET CMP0146 NEW)
cmake_policy(SET CMP0074 NEW)

# Set OrbbecSDK paths
set(OrbbecSDK_LIBS_DIR /home/enes/Desktop/ORBBEC/OrbbecSDK/lib/linux_x64)
set(OrbbecSDK_INCLUDE /home/enes/Desktop/ORBBEC/OrbbecSDK/include) # Adjust this path as necessary
set(OrbbecSDK_INCLUDE_HPP /home/enes/Desktop/ORBBEC/OrbbecSDK/include/libobsensor/hpp) # Adjust this path as necessary
set(Eigen_Folder /home/enes/Desktop/PhD/Repos/eigen) # Adjust this path as necessary

# Add PCL configuration
set(PCL_DIR /usr/lib/x86_64-linux-gnu/cmake/pcl)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
message(STATUS "PCL include dirs: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL library dirs: ${PCL_LIBRARY_DIRS}")

# Include directories for Eigen and Orbbec SDK
include_directories(
    ${Eigen_Folder}
    ${OrbbecSDK_INCLUDE}
    ${OrbbecSDK_INCLUDE_HPP}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Build CUDA library for voxel grid
cuda_add_library(VoxelGrid ${LIB_TYPE} src/voxel_grid.cu)

# Define the OBPointCloud executable and link with libraries
add_executable(${PROJECT_NAME}
    src/PointCloud.cpp
    src/pointCloud_utils.cpp
    src/funcs.cpp
)

# Link libraries to OBPointCloud
target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    TBB::tbb
    ${OrbbecSDK_LIBS_DIR}/libOrbbecSDK.so
    ${OrbbecSDK_LIBS_DIR}/libdepthengine.so
    ${OrbbecSDK_LIBS_DIR}/libob_usb.so
    ${OrbbecSDK_LIBS_DIR}/liblive555.so
    cppzmq
    VoxelGrid
)

# Enable OpenMP if available
if(OpenMP_CXX_FOUND)
    target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
endif()

# Define the server executable
add_executable(server src/server.cpp)

# Link cppzmq to server
target_link_libraries(server PRIVATE cppzmq)

# Install configuration
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
