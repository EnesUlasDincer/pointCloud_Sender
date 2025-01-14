
#include "../include/pointCloud_utils.hpp"

// Save point cloud data to ply
void savePointsToPly(std::shared_ptr<ob::Frame> frame, std::string fileName) {
    int   pointsSize = frame->dataSize() / sizeof(OBPoint);
    FILE *fp         = fopen(fileName.c_str(), "wb+");
    fprintf(fp, "ply\n");
    fprintf(fp, "format ascii 1.0\n");
    fprintf(fp, "element vertex %d\n", pointsSize);
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "end_header\n");

    OBPoint *point = (OBPoint *)frame->data();
    for(int i = 0; i < pointsSize; i++) {
        fprintf(fp, "%.3f %.3f %.3f\n", point->x, point->y, point->z);
        point++;
    }

    fflush(fp);
    fclose(fp);
}

// Save colored point cloud data to ply
void saveRGBFrameToPly(std::shared_ptr<ob::Frame> frame, std::string fileName) {
    int   pointsSize = frame->dataSize() / sizeof(OBColorPoint);
    FILE *fp         = fopen(fileName.c_str(), "wb+");
    fprintf(fp, "ply\n");
    fprintf(fp, "format ascii 1.0\n");
    fprintf(fp, "element vertex %d\n", pointsSize);
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "property uchar red\n");
    fprintf(fp, "property uchar green\n");
    fprintf(fp, "property uchar blue\n");
    fprintf(fp, "end_header\n");

    OBColorPoint *point = (OBColorPoint *)frame->data();
    for(int i = 0; i < pointsSize; i++) {
        fprintf(fp, "%.3f %.3f %.3f %d %d %d\n", point->x, point->y, point->z, (int)point->r, (int)point->g, (int)point->b);
        point++;
    }

    fflush(fp);
    fclose(fp);
}

// Save a vector of colored points to a PLY file
void saveRGBPointsToPly(const std::vector<OBColorPoint> &points, std::string fileName) {
    FILE *fp = fopen(fileName.c_str(), "wb+");
    if (!fp) {
        std::cerr << "Failed to open file: " << fileName << std::endl;
        return;
    }

    // Write PLY header
    fprintf(fp, "ply\n");
    fprintf(fp, "format ascii 1.0\n");
    fprintf(fp, "element vertex %lu\n", points.size());
    fprintf(fp, "property float x\n");
    fprintf(fp, "property float y\n");
    fprintf(fp, "property float z\n");
    fprintf(fp, "property uchar red\n");
    fprintf(fp, "property uchar green\n");
    fprintf(fp, "property uchar blue\n");
    fprintf(fp, "end_header\n");

    // Write point data
    for (const auto &point : points) {
        fprintf(fp, "%.3f %.3f %.3f %d %d %d\n", 
                point.x, point.y, point.z, 
                static_cast<int>(point.r), 
                static_cast<int>(point.g), 
                static_cast<int>(point.b));
    }

    fclose(fp);
    std::cout << "File saved successfully: " << fileName << std::endl;
}

// Function to configure and enable the color stream
std::shared_ptr<ob::VideoStreamProfile> configureColorStream(ob::Pipeline &pipeline, std::shared_ptr<ob::Config> config) {
    std::shared_ptr<ob::VideoStreamProfile> colorProfile = nullptr;
    try {
        // Get all stream profiles of the color camera
        auto colorProfiles = pipeline.getStreamProfileList(OB_SENSOR_COLOR);
        if (colorProfiles) {
            // Select the default profile
            auto profile = colorProfiles->getProfile(OB_PROFILE_DEFAULT);
            colorProfile = profile->as<ob::VideoStreamProfile>();
            config->enableStream(colorProfile);
            std::cout << "Color stream enabled successfully." << std::endl;
        } else {
            std::cerr << "No color profiles available for the current device." << std::endl;
        }
    } catch (const ob::Error &e) {
        config->setAlignMode(ALIGN_DISABLE); // Disable alignment if color stream is unavailable
        std::cerr << "Failed to enable color stream: " << e.getMessage() << std::endl;
    }

    if (colorProfile) {
        //config->setAlignMode(ALIGN_D2C_HW_MODE); // Enable D2C alignment if color stream is available
        
        config->setAlignMode(ALIGN_D2C_SW_MODE); // Enable D2C alignment if color stream is available
    } else {
        config->setAlignMode(ALIGN_DISABLE);
    }

    return colorProfile;
}

// Convert a std::shared_ptr<ob::Frame> to a std::vector<OBColorPoint>
std::vector<OBColorPoint> frameToVector(const std::shared_ptr<ob::Frame> &frame) {
    // Check for null frame
    if (!frame) {
        std::cerr << "Invalid frame provided!" << std::endl;
        return {}; // Return empty vector
    }

    // Calculate the number of points
    size_t pointsSize = frame->dataSize() / sizeof(OBColorPoint);
    if (pointsSize == 0) {
        std::cerr << "No points found in the frame!" << std::endl;
        return {}; // Return empty vector
    }

    // Get raw point data
    auto *rawPoints = static_cast<OBColorPoint *>(frame->data());
    if (!rawPoints) {
        std::cerr << "Failed to retrieve point data from frame!" << std::endl;
        return {}; // Return empty vector
    }

    // Construct the vector directly
    return {rawPoints, rawPoints + pointsSize};
}

// Function to process and save an RGBD PointCloud to a PLY file
void processAndSaveRGBDPointCloud(ob::Pipeline &pipeline, ob::PointCloudFilter &pointCloud) {
    try {
        // Wait for up to 100ms for a frameset in blocking mode
        auto frameset = pipeline.waitForFrames(100);
        if (frameset && frameset->depthFrame() && frameset->colorFrame()) {
            // Scale depth values to millimeters (if necessary)
            auto depthValueScale = frameset->depthFrame()->getValueScale();
            pointCloud.setPositionDataScaled(depthValueScale);

            // Generate and save the colored point cloud
            std::cout << "Generating RGBD PointCloud..." << std::endl;
            pointCloud.setCreatePointFormat(OB_FORMAT_RGB_POINT);
            std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);

            // Convert the frame to a vector of OBColorPoint
            std::vector<OBColorPoint> framePointsVec = frameToVector(frame);

            saveRGBPointsToPly(framePointsVec, "RGBPoints.ply");
            std::cout << "RGBPoints.ply saved successfully!" << std::endl;
        } else {
            std::cerr << "Failed to retrieve a valid color or depth frame!" << std::endl;
        }
    } catch (const std::exception &e) {
        std::cerr << "Error processing RGBD PointCloud: " << e.what() << std::endl;
    }
}

// Function to process and return an RGBD PointCloud frame
std::shared_ptr<ob::Frame> processRGBDPointCloud(ob::Pipeline &pipeline, ob::PointCloudFilter &pointCloud) {
    try {
        // Wait for up to 100ms for a frameset in blocking mode
        auto frameset = pipeline.waitForFrames(100);
        if (frameset && frameset->depthFrame() && frameset->colorFrame()) {
            // Scale depth values to millimeters (if necessary)
            auto depthValueScale = frameset->depthFrame()->getValueScale();
            // print the depth scale
            
            pointCloud.setPositionDataScaled(depthValueScale);
            
            //pointCloud.setFrameAlignState(true); // Enable frame alignment

            // Generate the colored point cloud
            std::cout << "Generating RGBD PointCloud..." << std::endl;
            pointCloud.setCreatePointFormat(OB_FORMAT_RGB_POINT); // OB_FORMAT_RGB_POINT , OB_FORMAT_POINT
            std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);

            std::cout << "RGBD PointCloud generated successfully!" << std::endl;

            // Return the processed frame
            return frame;
        } else {
            std::cerr << "Failed to retrieve a valid color or depth frame!" << std::endl;
            return nullptr; // Return nullptr if frames are invalid
        }
    } catch (const std::exception &e) {
        std::cerr << "Error processing RGBD PointCloud: " << e.what() << std::endl;
        return nullptr; // Return nullptr in case of an exception
    }
}


// // Function to process and save a Depth PointCloud to a PLY file
// void processAndSaveDepthPointCloud(ob::Pipeline &pipeline, ob::PointCloudFilter &pointCloud) {
//     try {
//         // Wait for up to 100ms for a frameset in blocking mode
//         auto frameset = pipeline.waitForFrames(100);
//         if (frameset && frameset->depthFrame()) {
//             // Scale depth values to millimeters (if necessary)
//             auto depthValueScale = frameset->depthFrame()->getValueScale();
//             pointCloud.setPositionDataScaled(depthValueScale);

//             // Generate and save the point cloud
//             std::cout << "Generating Depth PointCloud..." << std::endl;
//             pointCloud.setCreatePointFormat(OB_FORMAT_POINT);
//             std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);

//             savePointsToPly(frame, "DepthPoints.ply");
//             std::cout << "DepthPoints.ply saved successfully!" << std::endl;
//         } else {
//             std::cerr << "Failed to retrieve a valid depth frame!" << std::endl;
//         }
//     } catch (const std::exception &e) {
//         std::cerr << "Error processing Depth PointCloud: " << e.what() << std::endl;
//     }
// }

void processAndSaveDepthPointCloud(ob::Pipeline &pipeline, ob::PointCloudFilter &pointCloud) {
    try {
        // Wait for up to 100ms for a frameset in blocking mode
        auto frameset = pipeline.waitForFrames(100);
        if (frameset && frameset->depthFrame()) {
            auto depthFrame = frameset->depthFrame();

            // Scale depth values to millimeters (if necessary)
            auto depthValueScale = depthFrame->getValueScale();
            pointCloud.setPositionDataScaled(depthValueScale);

            std::cout << "Depth Scale: " << depthValueScale << std::endl;
            std::cout << "Depth Width: " << depthFrame->width() << ", Height: " << depthFrame->height() << std::endl;

            // Generate and save the point cloud
            std::cout << "Generating Depth PointCloud..." << std::endl;
            pointCloud.setCreatePointFormat(OB_FORMAT_POINT);
            std::shared_ptr<ob::Frame> frame = pointCloud.process(frameset);

            savePointsToPly(frame, "DepthPoints.ply");
            std::cout << "DepthPoints.ply saved successfully!" << std::endl;
        } else {
            std::cerr << "Failed to retrieve a valid depth frame!" << std::endl;
        }
    } catch (const std::exception &e) {
        std::cerr << "Error processing Depth PointCloud: " << e.what() << std::endl;
    }
}

void intro_prompt() {
    std::cout << "-----------------------------------------" << std::endl;
    std::cout << "This sample demonstrates how to create a point cloud from the depth and color streams of a device." << std::endl;
    std::cout << "The sample will create a point cloud and save it to a PLY file." << std::endl;
    // operation prompt
    std::cout << "Press R or r to create RGBD PointCloud and save to ply file! " << std::endl;
    std::cout << "Press D or d to create Depth PointCloud and save to ply file! " << std::endl;
    std::cout << "Press ESC to exit! " << std::endl;
    std::cout << "-----------------------------------------" << std::endl;
}

// Function to configure and retrieve depth stream profiles
std::shared_ptr<ob::StreamProfileList> configureDepthStream(ob::Pipeline &pipeline, 
                                                            std::shared_ptr<ob::VideoStreamProfile> colorProfile, 
                                                            OBAlignMode &alignMode,
                                                            std::shared_ptr<ob::Config> config) {
    std::shared_ptr<ob::StreamProfileList> depthProfileList;

    try {
        // Try hardware-based D2C alignment if a color profile is available
        if (colorProfile) {
            depthProfileList = pipeline.getD2CDepthProfileList(colorProfile, ALIGN_D2C_HW_MODE);
            if (depthProfileList && depthProfileList->count() > 0) {
                alignMode = ALIGN_D2C_HW_MODE;
                std::cout << "Hardware-based D2C alignment enabled." << std::endl;
            } else {
                // Try software-based D2C alignment if hardware-based alignment fails
                depthProfileList = pipeline.getD2CDepthProfileList(colorProfile, ALIGN_D2C_SW_MODE);
                if (depthProfileList && depthProfileList->count() > 0) {
                    alignMode = ALIGN_D2C_SW_MODE;
                    std::cout << "Software-based D2C alignment enabled." << std::endl;
                } else {
                    std::cerr << "No compatible D2C alignment profiles found. Disabling alignment." << std::endl;
                }
            }
        }

        // If no D2C alignment is needed or supported, fallback to default depth profiles
        if (!depthProfileList || depthProfileList->count() == 0) {
            depthProfileList = pipeline.getStreamProfileList(OB_SENSOR_DEPTH);
            alignMode = ALIGN_DISABLE;
            std::cout << "No D2C alignment profiles found. Disabling alignment." << std::endl;
        }

        // Configure depth stream if profiles are available
        if (depthProfileList && depthProfileList->count() > 0) {
            std::shared_ptr<ob::StreamProfile> depthProfile;
            try {
                // Attempt to match the frame rate with the color profile
                if (colorProfile) {
                    std::cout << "Matching depth frame rate with color profile..." << std::endl;
                    depthProfile = depthProfileList->getVideoStreamProfile(OB_WIDTH_ANY, OB_HEIGHT_ANY, OB_FORMAT_ANY, colorProfile->fps());
                }
            } catch (...) {
                // Fallback to default profile if no matching frame rate is found
                depthProfile = nullptr;
            }

            if (!depthProfile) {
                depthProfile = depthProfileList->getProfile(OB_PROFILE_DEFAULT);
                std::cout << "Using default depth profile." << std::endl;
            }
            // Configure the depth stream explicitly
            depthProfile = depthProfileList->getVideoStreamProfile(640, 576, OB_FORMAT_Y16, 30);

            if (!depthProfile) {
                std::cerr << "Unable to find the requested depth stream profile: 640x576, 30 fps, Y16 format!" << std::endl;
                exit(EXIT_FAILURE);
            }
            std::cout << "config->enableStream" << std::endl;
            config->enableStream(depthProfile);
        } else {
            std::cerr << "No depth profiles available for the current device!" << std::endl;
        }

    } catch (const ob::Error &e) {
        std::cerr << "Error configuring depth stream: " << e.getMessage() << std::endl;
        alignMode = ALIGN_DISABLE;
        return nullptr; // Explicitly return nullptr on failure
    }

    config->setAlignMode(alignMode);

    if (depthProfileList && depthProfileList->count() > 0) {
        std::cout << "Depth stream successfully configured." << std::endl;
    } else {
        std::cerr << "Failed to configure depth stream." << std::endl;
    }

    return depthProfileList;
}


// Function to transform a copy of the point cloud and return the transformed points
std::vector<OBColorPoint> getTransformedPointCloud(std::shared_ptr<ob::Frame> frame, const Eigen::Matrix4f &T_camera_to_robot) {
    int pointsSize = frame->dataSize() / sizeof(OBColorPoint);
    OBColorPoint *originalPoints = (OBColorPoint *)frame->data();

    // Create a vector to store the transformed points
    std::vector<OBColorPoint> transformedPoints(pointsSize);

    for (int i = 0; i < pointsSize; i++) {
        // Convert the point to homogeneous coordinates
        Eigen::Vector4f point_homogeneous(originalPoints[i].x, originalPoints[i].y, originalPoints[i].z, 1.0f);

        // Apply the transformation
        Eigen::Vector4f transformed_point = T_camera_to_robot * point_homogeneous;

        // Populate the transformed points
        transformedPoints[i].x = transformed_point(0);
        transformedPoints[i].y = transformed_point(1);
        transformedPoints[i].z = transformed_point(2);

        // Copy the color information
        transformedPoints[i].r = originalPoints[i].r;
        transformedPoints[i].g = originalPoints[i].g;
        transformedPoints[i].b = originalPoints[i].b;
    }

    // Return the transformed points
    return transformedPoints;
}

// Function to transform a copy of the point cloud and return the transformed points
std::vector<OBColorPoint> getTransformedPointCloud2(std::shared_ptr<ob::Frame> frame, const Eigen::Matrix4f &T_camera_to_robot) {
    // Calculate the number of points in the frame
    int pointsSize = frame->dataSize() / sizeof(OBColorPoint);
    OBColorPoint *originalPoints = (OBColorPoint *)frame->data();

    // Create a vector to store the transformed points
    std::vector<OBColorPoint> transformedPoints(pointsSize);

    // Transform points in parallel using std::transform with parallel execution
    std::transform(
        std::execution::par, 
        originalPoints, originalPoints + pointsSize, transformedPoints.begin(),
        [&T_camera_to_robot](const OBColorPoint &point) {
            // Convert the point to homogeneous coordinates
            Eigen::Vector4f point_homogeneous(point.x, point.y, point.z, 1.0f);

            // Apply the transformation
            Eigen::Vector4f transformed_point = T_camera_to_robot * point_homogeneous;

            // Create a transformed OBColorPoint
            OBColorPoint transformed;
            transformed.x = transformed_point(0);
            transformed.y = transformed_point(1);
            transformed.z = transformed_point(2);

            // Copy the color information
            transformed.r = point.r;
            transformed.g = point.g;
            transformed.b = point.b;

            return transformed;
        }
    );

    // Return the transformed points
    return transformedPoints;
}

// Function to read the transformation matrix from a file
Eigen::Matrix4f readTransformationMatrix(const std::string& filePath) {
    // Initialize an empty Eigen::Matrix4f
    Eigen::Matrix4f T_camera_to_QR = Eigen::Matrix4f::Identity();

    // Open the file
    std::ifstream file(filePath);
    if (!file.is_open()) {
        throw std::runtime_error("Error: Unable to open file at " + filePath);
    }

    // Skip the header line
    std::string header;
    std::getline(file, header);
    if (header.find("Mean Transformation Matrix:") == std::string::npos) {
        throw std::runtime_error("Error: Unexpected file format. Header not found.");
    }

    // Read the matrix elements
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!(file >> T_camera_to_QR(i, j))) {
                throw std::runtime_error("Error: Invalid matrix format in file.");
            }
        }
    }

    // Close the file
    file.close();

    return T_camera_to_QR;
}


// // Function to crop a point cloud within a rectangular prism
// std::vector<OBColorPoint> cropPointCloud(
//     const std::vector<OBColorPoint>& points,
//     float minX, float maxX,
//     float minY, float maxY,
//     float minZ, float maxZ) 
// {
//     std::vector<OBColorPoint> croppedPoints;

//     for (const auto& point : points) {

//     bool cond1 = point.x >= minX && point.x <= maxX &&
//                  point.y >= minY && point.y <= maxY &&
//                  point.z >= minZ && point.z <= maxZ;

//     if (cond1)
//     {
//         croppedPoints.push_back(point);
//     }
    
            
//     }

//     return croppedPoints;
// }

// Function to crop a point cloud within a rectangular prism
std::vector<OBColorPoint> cropPointCloudParallel(
    const std::vector<OBColorPoint>& points,
    float minX, float maxX,
    float minY, float maxY,
    float minZ, float maxZ) 
{
    std::vector<OBColorPoint> croppedPoints;
    croppedPoints.reserve(points.size()); // Reserve full size for thread safety

    #pragma omp parallel
    {
        std::vector<OBColorPoint> localCroppedPoints;
        #pragma omp for nowait
        for (size_t i = 0; i < points.size(); ++i) {
            const auto& point = points[i];
            if (point.x >= minX && point.x <= maxX &&
                point.y >= minY && point.y <= maxY &&
                point.z >= minZ && point.z <= maxZ) 
            {
                localCroppedPoints.push_back(point);
            }
        }
        #pragma omp critical
        croppedPoints.insert(croppedPoints.end(), localCroppedPoints.begin(), localCroppedPoints.end());
    }

    return croppedPoints;
}


// Function to apply transformation
std::vector<OBColorPoint> transformPoints(
    const std::vector<OBColorPoint>& points,
    const Eigen::Matrix4f& T_camera_to_robot) 
{
    // Output vector to hold transformed points
    std::vector<OBColorPoint> transformedPoints(points.size());

    // Use std::transform with parallel execution policy
    std::transform(std::execution::par, points.begin(), points.end(), transformedPoints.begin(),
        [&T_camera_to_robot](const OBColorPoint& point) {
            // Convert point to homogeneous coordinates
            Eigen::Vector4f homogeneousPoint(point.x, point.y, point.z, 1.0f);

            // Transform point using the given matrix
            Eigen::Vector4f transformedHomogeneousPoint = T_camera_to_robot * homogeneousPoint;

            // Create a new OBColorPoint with transformed coordinates
            OBColorPoint transformedPoint = {
                transformedHomogeneousPoint(0),  // Transformed X
                transformedHomogeneousPoint(1),  // Transformed Y
                transformedHomogeneousPoint(2),  // Transformed Z
                point.r,  // Retain original color
                point.g,
                point.b
            };

            return transformedPoint;
        });

    return transformedPoints;
}

void adjustByteArray(const float* inputArray, float* outputArray, size_t currentCount, size_t targetCount) {
    constexpr size_t floatsPerPoint = 6; // Each point has 6 floats

    std::mt19937 rng(std::random_device{}());

    if (currentCount > targetCount) {
        // Randomly sample down to targetCount points
        std::vector<size_t> indices(currentCount);
        std::iota(indices.begin(), indices.end(), 0);
        std::shuffle(indices.begin(), indices.end(), rng);

        // Copy sampled points to output array
        for (size_t i = 0; i < targetCount; ++i) {
            size_t index = indices[i];
            std::copy_n(inputArray + index * floatsPerPoint, floatsPerPoint, outputArray + i * floatsPerPoint);
        }
    } else if (currentCount < targetCount) {
        // Copy all current points to output array
        std::copy_n(inputArray, currentCount * floatsPerPoint, outputArray);

        // Randomly duplicate points to fill remaining space
        std::uniform_int_distribution<size_t> dist(0, currentCount - 1);
        for (size_t i = currentCount; i < targetCount; ++i) {
            size_t randomIndex = dist(rng);
            std::copy_n(inputArray + randomIndex * floatsPerPoint, floatsPerPoint, outputArray + i * floatsPerPoint);
        }
    } else {
        // If currentCount == targetCount, directly copy input to output
        std::copy_n(inputArray, currentCount * floatsPerPoint, outputArray);
    }
}


//Function to adjust the number of points to exactly targetCount
std::vector<OBColorPoint> adjustPointCount(const std::vector<OBColorPoint>& points, size_t targetCount) {
    std::vector<OBColorPoint> adjustedPoints;
    adjustedPoints.reserve(targetCount); // Reserve possible final size

    size_t currentCount = points.size();
    std::cout << "Current count: " << currentCount << std::endl;

    if (currentCount > targetCount) {
        // Randomly sample down to targetCount points
        std::sample(points.begin(), points.end(),
                    std::back_inserter(adjustedPoints),
                    targetCount,
                    std::mt19937{std::random_device{}()});
    } else if (currentCount < targetCount) {
        // Copy points first
        adjustedPoints = points;
        // Randomly duplicate to reach targetCount
        std::mt19937 rng(std::random_device{}());
        std::uniform_int_distribution<size_t> dist(0, points.size() - 1);
        while (adjustedPoints.size() < targetCount) {
            adjustedPoints.push_back(points[dist(rng)]);
        }
    } else {
        // Already the correct size
        adjustedPoints = points;
    }

    std::cout << "Adjusted count: " << adjustedPoints.size() << std::endl;
    return adjustedPoints;
}

// Function to downsample a point cloud using voxel grid
pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleWithVoxelGrid(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float voxelSize) 
{
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(voxelSize, voxelSize, voxelSize);

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZ>());
    voxelGrid.filter(*downsampledCloud);

    return downsampledCloud;
}

std::vector<OBColorPoint> uniformSubsamplePointCloud(
    const std::vector<OBColorPoint>& points, size_t targetPointCount) 
{
    if (points.size() <= targetPointCount) {
        return points;
    }

    std::vector<OBColorPoint> downsampledPoints;
    downsampledPoints.reserve(targetPointCount);

    size_t step = points.size() / targetPointCount;

    for (size_t i = 0; i < points.size(); i += step) {
        if (downsampledPoints.size() < targetPointCount) {
            downsampledPoints.push_back(points[i]);
        }
    }

    return downsampledPoints;
}

// Hash function for voxel indexing
struct VoxelHash {
    size_t operator()(const std::tuple<int, int, int>& voxel) const {
        auto [x, y, z] = voxel;
        return std::hash<int>()(x) ^ (std::hash<int>()(y) << 1) ^ (std::hash<int>()(z) << 2);
    }
};

// Convert a std::vector<OBColorPoint> to a pcl::PointCloud<pcl::PointXYZRGB>
pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToPCLPointCloud(const std::vector<OBColorPoint>& points) {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    cloud->points.reserve(points.size()); // Reserve memory to avoid reallocation

    for (const auto& point : points) {
        cloud->points.emplace_back(pcl::PointXYZRGB{
            point.x, point.y, point.z,
            static_cast<uint8_t>(point.r),
            static_cast<uint8_t>(point.g),
            static_cast<uint8_t>(point.b)
        });
    }

    cloud->width = cloud->points.size();
    cloud->height = 1; // Unorganized point cloud
    cloud->is_dense = true;

    return cloud;
}

// Downsample a point cloud with color using a fast voxel grid
pcl::PointCloud<pcl::PointXYZRGB>::Ptr fastVoxelGridDownsampleWithColor(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, float voxelSize) {
    
    if (!cloud || cloud->empty()) {
        std::cerr << "Input cloud is empty!" << std::endl;
        return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    }

    // Map to aggregate points within each voxel
    std::unordered_map<std::tuple<int, int, int>, 
                       std::tuple<float, float, float, int, int, int, int>, 
                       VoxelHash> voxelMap;

    // Iterate through points and hash them into voxel grid
    for (const auto& point : cloud->points) {
        int vx = static_cast<int>(std::floor(point.x / voxelSize));
        int vy = static_cast<int>(std::floor(point.y / voxelSize));
        int vz = static_cast<int>(std::floor(point.z / voxelSize));
        auto voxelIndex = std::make_tuple(vx, vy, vz);

        // Aggregate points in the voxel
        if (voxelMap.find(voxelIndex) == voxelMap.end()) {
            voxelMap[voxelIndex] = std::make_tuple(point.x, point.y, point.z, 
                                                   point.r, point.g, point.b, 1);
        } else {
            auto& [xSum, ySum, zSum, rSum, gSum, bSum, count] = voxelMap[voxelIndex];
            xSum += point.x;
            ySum += point.y;
            zSum += point.z;
            rSum += point.r;
            gSum += point.g;
            bSum += point.b;
            count++;
        }
    }

    // Create downsampled point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    downsampledCloud->points.reserve(voxelMap.size());

    for (const auto& [key, value] : voxelMap) {
        const auto& [xSum, ySum, zSum, rSum, gSum, bSum, count] = value;
        pcl::PointXYZRGB downsampledPoint;
        downsampledPoint.x = xSum / count;
        downsampledPoint.y = ySum / count;
        downsampledPoint.z = zSum / count;
        downsampledPoint.r = static_cast<uint8_t>(rSum / count);
        downsampledPoint.g = static_cast<uint8_t>(gSum / count);
        downsampledPoint.b = static_cast<uint8_t>(bSum / count);
        downsampledCloud->points.push_back(downsampledPoint);
    }

    downsampledCloud->width = downsampledCloud->points.size();
    downsampledCloud->height = 1;
    downsampledCloud->is_dense = true;

    return downsampledCloud;
}

// Main function to downsample to exactly 3000 points
std::vector<OBColorPoint> downsampleToTarget(const std::vector<OBColorPoint>& points, float voxelSize, size_t targetCount) {
    // Step 1: Apply voxel grid downsampling
    // check the time 
    auto start = std::chrono::high_resolution_clock::now();
    
    // Convert OBColorPoint vector to PCL point cloud
    // auto pclCloud = convertToPCLPointCloud(points);
    // auto downsampledPCLCloud = fastVoxelGridDownsampleWithColor(pclCloud, voxelSize);
    // // If needed, convert the downsampled PCL cloud back to std::vector<OBColorPoint>
    // std::vector<OBColorPoint> downsampledPoints;
    // downsampledPoints.reserve(downsampledPCLCloud->points.size());
    // for (const auto& point : downsampledPCLCloud->points) {
    //     downsampledPoints.push_back(OBColorPoint{point.x, point.y, point.z, /* other fields if any */});
    // }

    auto downsampledPoints = uniformSubsamplePointCloud(points, targetCount);

    // auto downsampledPoints = VoxelGridDownsampleWithThrust(points, voxelSize, targetCount);


    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << "Voxel grid downsampling took: " << elapsed.count() << " seconds" << std::endl;

    // Step 2: Adjust the number of points to the target count
    size_t currentCount = downsampledPoints.size();
    if (currentCount > targetCount) {
        return adjustPointCount(downsampledPoints, targetCount);
    }else{
        return downsampledPoints;
    }

    
}

// Target number of points after downsampling
size_t targetCount = 10000; // 7500;

// Voxel size for downsampling
float voxelSize = 0.01f * 1000.0f;

// Define the bounding box of the rectangular prism
float minX = -300.0, maxX = 300.0;
float minY = -300.0, maxY = 300.0;
float minZ = -300.0, maxZ = 1500.0;

// // Parameters for downsampling
// float min_bound[3] = {-300.0f, -300.0f, -300.0f};
// float max_bound[3] = { 300.0f,   300.0f, 1500.0f};
// float voxel_size[3] = {0.5f, 0.5f, 0.5f}; // Voxel size

// // Allocate output buffer
// size_t max_voxels = 10000; // Adjust as needed
// float byteArray[max_voxels * 6]; // Each voxel has {x, y, z, r, g, b}