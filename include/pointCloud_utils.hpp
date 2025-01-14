
#pragma once

//#include "libobsensor/ObSensor.hpp"
#include <libobsensor/ObSensor.hpp>
#include <memory>
#include <vector>
#include <string>
#include <Eigen/Dense>

#include <unordered_map>
#include <tuple>
#include <cmath>
#include <algorithm>

#include <execution>
#include <random>
#include <fstream>

// #include <thrust/device_vector.h>
// #include <thrust/host_vector.h>
// #include <thrust/transform.h>
// #include <thrust/reduce.h>
// #include <thrust/iterator/zip_iterator.h>
// #include <thrust/unique.h>
// #include <thrust/sort.h>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <omp.h>

#include <cuda_runtime.h>
#include <unordered_map>

#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

// Save point cloud data to ply
void savePointsToPly(std::shared_ptr<ob::Frame> frame, std::string fileName);

// Save colored point cloud data to ply
void saveRGBFrameToPly(std::shared_ptr<ob::Frame> frame, std::string fileName);

// Save a vector of colored points to a PLY file
void saveRGBPointsToPly(const std::vector<OBColorPoint> &points, std::string fileName);

// Function to configure and enable the color stream
std::shared_ptr<ob::VideoStreamProfile> configureColorStream(ob::Pipeline &pipeline, std::shared_ptr<ob::Config> config);

// Convert a std::shared_ptr<ob::Frame> to a std::vector<OBColorPoint>
std::vector<OBColorPoint> frameToVector(const std::shared_ptr<ob::Frame> &frame);

// Function to process and save an RGBD PointCloud to a PLY file
void processAndSaveRGBDPointCloud(ob::Pipeline &pipeline, ob::PointCloudFilter &pointCloud);

// Function to process and return an RGBD PointCloud frame
std::shared_ptr<ob::Frame> processRGBDPointCloud(ob::Pipeline &pipeline, ob::PointCloudFilter &pointCloud);


// Function to process and save a Depth PointCloud to a PLY file
void processAndSaveDepthPointCloud(ob::Pipeline &pipeline, ob::PointCloudFilter &pointCloud);

void intro_prompt();

// Function to configure and retrieve depth stream profiles
std::shared_ptr<ob::StreamProfileList> configureDepthStream(ob::Pipeline &pipeline, 
                                                            std::shared_ptr<ob::VideoStreamProfile> colorProfile, 
                                                            OBAlignMode &alignMode,
                                                            std::shared_ptr<ob::Config> config);

// Function to transform a copy of the point cloud and return the transformed points
std::vector<OBColorPoint> getTransformedPointCloud(std::shared_ptr<ob::Frame> frame, const Eigen::Matrix4f &T_camera_to_robot);

// Function to transform a copy of the point cloud and return the transformed points
std::vector<OBColorPoint> getTransformedPointCloud2(std::shared_ptr<ob::Frame> frame, const Eigen::Matrix4f &T_camera_to_robot);

// Function to read the transformation matrix from a file
Eigen::Matrix4f readTransformationMatrix(const std::string& filePath);


// Function to crop a point cloud within a rectangular prism
std::vector<OBColorPoint> cropPointCloud(
    const std::vector<OBColorPoint>& points,
    float minX, float maxX,
    float minY, float maxY,
    float minZ, float maxZ);

std::vector<OBColorPoint> cropPointCloudParallel(
    const std::vector<OBColorPoint>& points,
    float minX, float maxX,
    float minY, float maxY,
    float minZ, float maxZ);


// Function to apply transformation
std::vector<OBColorPoint> transformPoints(
    const std::vector<OBColorPoint>& points,
    const Eigen::Matrix4f& T_camera_to_robot);

// Voxel grid downsampling
std::vector<OBColorPoint> voxelGridDownsample(const std::vector<OBColorPoint>& points, float voxelSize);

// Function to adjust the number of points to exactly targetCount
std::vector<OBColorPoint> adjustPointCount(const std::vector<OBColorPoint>& points, size_t targetCount);

// Main function to downsample to exactly 3000 points
std::vector<OBColorPoint> downsampleToTarget(const std::vector<OBColorPoint>& points, float voxelSize, size_t targetCount);

void adjustByteArray(const float* inputArray, float* outputArray, size_t currentCount, size_t targetCount);

// Target number of points after downsampling
extern size_t targetCount;

// Voxel size for downsampling
extern float voxelSize;

// Define the bounding box of the rectangular prism
extern float minX, maxX;
extern float minY, maxY;
extern float minZ, maxZ;

// // Parameters for downsampling
// extern float  *min_bound;
// extern float  *max_bound;
// extern float *voxel_size;

// // Allocate output buffer
// extern size_t max_voxels; // Adjust as needed
// extern float *byteArray; // Each voxel has {x, y, z, r, g, b}
