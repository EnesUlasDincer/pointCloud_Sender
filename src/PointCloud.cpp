
// #include "opencv2/opencv.hpp"
//#include <libobsensor/hpp/Utils.hpp>


#include "../include/funcs.hpp"
#include "../include/pointCloud_utils.hpp"
#include "../include/window.hpp"
#include "../include/voxel_grid.cuh"

#include <vector>

#include <chrono>

#include <zmq.hpp>
#include <iostream>
#include <sstream>
#include <string>
#include <thread> // For simulating delay

#define KEY_ESC 27
#define KEY_R 82
#define KEY_r 114


// Serialize the point cloud into a byte array
std::vector<uint8_t> serializePointCloudToBytes(const std::vector<OBColorPoint>& points) {
    std::vector<uint8_t> serializedData;
    serializedData.reserve(points.size() * (sizeof(float) * 3 + sizeof(uint8_t) * 3)); // Reserve memory for efficiency

    for (const auto& point : points) {
        // Serialize coordinates
        const uint8_t* xBytes = reinterpret_cast<const uint8_t*>(&point.x);
        const uint8_t* yBytes = reinterpret_cast<const uint8_t*>(&point.y);
        const uint8_t* zBytes = reinterpret_cast<const uint8_t*>(&point.z);
        serializedData.insert(serializedData.end(), xBytes, xBytes + sizeof(float));
        serializedData.insert(serializedData.end(), yBytes, yBytes + sizeof(float));
        serializedData.insert(serializedData.end(), zBytes, zBytes + sizeof(float));

        // Serialize color components
        serializedData.push_back(point.r);
        serializedData.push_back(point.g);
        serializedData.push_back(point.b);
    }

    return serializedData;
}

// Serialize the point cloud into a vector of floats
std::vector<float> serializePointCloudToFloats(const std::vector<OBColorPoint>& points) {
    std::vector<float> serializedData;
    serializedData.reserve(points.size() * 6); // 3 floats for position + 3 floats for color

    for (const auto& point : points) {
        // Append position coordinates
        serializedData.push_back(point.x);
        serializedData.push_back(point.y);
        serializedData.push_back(point.z);

        // Append color components as floats
        serializedData.push_back(point.r);
        serializedData.push_back(point.g);
        serializedData.push_back(point.b);
    }

    return serializedData;
}

// Serialize OBColorPoint to byte[]
uint8_t* serializePointCloudToByteArray(const std::vector<OBColorPoint>& points, size_t& byteArraySize) {
    // Calculate total size: number of points * size of each point
    size_t totalSize = points.size() * sizeof(OBColorPoint);
    byteArraySize = totalSize; // Output the size of the array

    // Allocate memory for the byte array
    uint8_t* byteArray = new uint8_t[totalSize];

    // Copy all points into the byte array
    std::memcpy(byteArray, points.data(), totalSize);

    return byteArray;
}


int main(int argc, char **argv) try {
    ob::Context::setLoggerSeverity(OB_LOG_SEVERITY_WARN);

    ob::Context ob_context;
    
    // Query the list of connected devices
    auto devList = ob_context.queryDeviceList();

    // ask user to choose the camera
    std::cout << "Choose the camera (1 or 2): ";
    int cameraChoice;
    std::cin >> cameraChoice;

    //auto dev = devList->getDevice(cameraChoice);
    std::shared_ptr<ob::Device> dev;

    // check the camera serial number if it is CL8A8420179 or CL8A84201GW 
    // select the correct device
    // Get the number of connected devices
    int devCount = devList->deviceCount();
    for (int i = 0; i < devCount; i++) {
        // if devList->serialNumber(i) == "CL8A8420179" or "CL8A84201GW"
        // select the device
        if (strcmp(devList->serialNumber(i), "CL8A8420179") == 0 && cameraChoice == 1) {
            dev = devList->getDevice(i);
        } else if (strcmp(devList->serialNumber(i), "CL8A84201GW") == 0 && cameraChoice == 2) {
            dev = devList->getDevice(i);
        }
    }

    std::cout << "pipeline is initliazed." << std::endl;

    // Create a pipeline with default device
    ob::Pipeline pipeline(dev);

    // Configure which streams to enable or disable for the Pipeline by creating a Config
    std::shared_ptr<ob::Config> config = std::make_shared<ob::Config>();

    // Turn on D2C alignment, which needs to be turned on when generating RGBD point clouds
    std::shared_ptr<ob::VideoStreamProfile> colorProfile = configureColorStream(pipeline, config);

    // Get all stream profiles of the depth camera, including stream resolution, frame rate, and frame format
    std::shared_ptr<ob::StreamProfileList> depthProfileList;
    OBAlignMode                            alignMode = ALIGN_DISABLE; // ALIGN_D2C_SW_MODE, ALIGN_DISABLE
    
    
    // Configure depth stream with or without color alignment
    depthProfileList = configureDepthStream(pipeline, colorProfile, alignMode, config);

    // start pipeline with config
    pipeline.start(config);
    
    // Create a point cloud Filter object (the device parameters will be obtained inside the Pipeline when the point cloud filter is created, so try to
    // configure the device before creating the filter)
    ob::PointCloudFilter pointCloud;

    // get camera intrinsic and extrinsic parameters form pipeline and set to point cloud filter
    auto cameraParam = pipeline.getCameraParam();
    pointCloud.setCameraParam(cameraParam);
    
    // Display the operation prompt
    intro_prompt();

    auto currentProfile = pipeline.getEnabledStreamProfileList()->getProfile(0)->as<ob::VideoStreamProfile>();
    std::cout << "Stream resolution: " << currentProfile->width() << "x" << currentProfile->height() << std::endl;
    // Create a window for rendering, and set the resolution of the window
    // Window app("PointCloud_Viewer", currentProfile->width(), currentProfile->height());

    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_PUSH);
    socket.connect("tcp://127.0.0.1:5553");

    // zmq::socket_t receiver(context, ZMQ_PULL);  // 🔹 Add receiver socket
    // receiver.connect("tcp://127.0.0.1:5543");

    // it was zmq::socket_t socket(context, zmq::socket_type::pub);
    // it was socket.bind("tcp://127.0.0.1:5556");
    // zmq::socket_t socket(context, zmq::socket_type::pub);
    // socket.bind("tcp://127.0.0.1:5556");

    // Parameters for downsampling
    float min_bound[3] = {-300.0f, -300.0f, -300.0f};
    float max_bound[3] = { 300.0f,   300.0f, 1500.0f};
    float voxel_size[3] = {0.5f, 0.5f, 0.5f}; // Voxel size

    // Allocate output buffer
    // size_t max_voxels = 10000; // Adjust as needed
    // float byteArray[10000 * 6]; // Each voxel has {x, y, z, r, g, b}
    // float* byteArray = new float[targetCount * 10 * 6];

    // Allocate output array
    float* outputArray = new float[targetCount * 6];


    // Path to the file
    std::string filePath_T_camera_to_QR = "/home/enesud/Desktop/Orbbec_codes/Orbbec_Codes/JPG_taker/build/Cam_1_pose_estimation.txt";
    // Read T_camera_to_QR matrix
    Eigen::Matrix4f T_camera_to_QR = readTransformationMatrix(filePath_T_camera_to_QR);

    // Eigen::Matrix4f T_camera_to_QR = Eigen::Matrix4f::Identity();

    // Print the transformation matrix
    std::cout << "Transformation Matrix (T_camera_to_QR):" << std::endl;
    std::cout << T_camera_to_QR << std::endl;

    T_camera_to_QR(0, 3) *= 1000.0;
    T_camera_to_QR(1, 3) *= 1000.0;
    T_camera_to_QR(2, 3) *= 1000.0;
    
    // Extract rotation (R) and translation (t)
    Eigen::Matrix3f R = T_camera_to_QR.block<3,3>(0, 0);
    Eigen::Vector3f t = T_camera_to_QR.block<3,1>(0, 3);

    // Compute the inverse efficiently
    Eigen::Matrix4f T_QR_to_camera = Eigen::Matrix4f::Identity();
    T_QR_to_camera.block<3,3>(0, 0) = R.transpose(); // Transpose of rotation
    T_QR_to_camera.block<3,1>(0, 3) = -R.transpose() * t; // Negative transformed translation

    
    // Define T_QR_to_robot (replace with actual values)
    Eigen::Matrix4f T_QR_to_robot;
    // Example: Fill in the matrix values here
    T_QR_to_robot <<  1, 0, 0, 0.0,
                      0, 1, 0, 0.0,
                      0, 0, 1, 0.0,
                      0, 0, 0, 1.0;
    

    // Compute T_camera_to_robot
    Eigen::Matrix4f T_camera_to_robot = T_QR_to_robot * T_QR_to_camera;

    while(true) {
        auto frameset = pipeline.waitForFrames(100);

        //auto pointCloudFrame_to_render = processRGBDPointCloud(pipeline, pointCloud);
        if(true )
        {
            // 🔹 Step 1: Wait for Capture Request
            // zmq::message_t captureSignal;
            // receiver.recv(captureSignal, zmq::recv_flags::none);

            // Process the RGBD point cloud and get the frame
            std::shared_ptr<ob::Frame> pointCloudFrame = processRGBDPointCloud(pipeline, pointCloud);
         
            //processAndSaveRGBDPointCloud(pipeline, pointCloud);
            if (pointCloudFrame) {
                // The frame can be used for further processing or saving
                std::cout << "PointCloud frame retrieved successfully!" << std::endl;

                auto start_frame_to_vector = std::chrono::high_resolution_clock::now();
                // Optionally save the frame to a PLY file
                // std::vector<OBColorPoint> pointCloudFrame_points = frameToVector(pointCloudFrame);
                FrameView pointCloudFrame_points = frameToPointer(pointCloudFrame);

                // float* byteArray = new float[pointCloudFrame_points.size() * 6];
                float* byteArray = new float[pointCloudFrame_points.size * 6];

                auto end_frame_to_vector = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed_frame_to_vector = end_frame_to_vector - start_frame_to_vector;
                std::cout << "Elapsed time for frame to vector: " << elapsed_frame_to_vector.count() << " seconds" << std::endl;

                auto start_downsampleToTarget = std::chrono::high_resolution_clock::now();
                // Downsample to exactly 3000 points
                // std::vector<OBColorPoint> cropped_and_downsampled_points = downsampleToTarget(pointCloudFrame_points, voxelSize, targetCount);

                // Perform voxel grid downsampling
                // uint32_t num_voxels = transformCropAndVoxelizeCenter(pointCloudFrame_points, byteArray, T_camera_to_QR);
                uint32_t num_voxels = transformCropAndVoxelizeCenter(pointCloudFrame_points.data, pointCloudFrame_points.size, byteArray, T_camera_to_robot);


                std::cout << "OUTSIDE; Number of voxels: " << num_voxels << std::endl;
                // Calculate the size of the byte array to send
                size_t byteArraySize = num_voxels * 6 * sizeof(float);

                auto end_downsampleToTarget = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> elapsed_downsampleToTarget = end_downsampleToTarget - start_downsampleToTarget;
                std::cout << "Elapsed time for downsampleToTarget: " << elapsed_downsampleToTarget.count() << " seconds" << std::endl;

                // Save the downsampled point cloud
                // saveRGBPointsToPly(cropped_and_downsampled_points, "downsampled_GeneratedRGBPoints.ply");
        
                // print the number of points
                // std::cout << "Before crop, Number of points in the frame: " << pointCloudFrame_points.size() << std::endl;
                std::cout << "Before crop, Number of points in the frame: " << pointCloudFrame_points.size << std::endl;
                // std::cout << "After crop, Number of points in the frame: " << croppedPoints.size() << std::endl;
                // std::cout << "After downsample, Number of points in the frame: " << cropped_and_downsampled_points.size() << std::endl;

                // Adjust the number of points
                adjustByteArray(byteArray, outputArray, num_voxels, targetCount);
                size_t output_byteArraySize = targetCount * 6 * sizeof(float);
                // Print size of serialized data
                std::cout << "Size of serialized data: " << output_byteArraySize << " bytes" << std::endl;

                

                // Send the serialized point cloud to the server
                zmq::message_t request(output_byteArraySize);
                std::memcpy(request.data(), outputArray, output_byteArraySize);
                socket.send(request, zmq::send_flags::none);

                // print send message
                std::cout << "Sent point cloud data:\n" << std::endl;

                // // Wait for acknowledgment
                // zmq::message_t reply;
                // auto result = socket.recv(reply, zmq::recv_flags::none);
                // if (result) {
                //     std::string replyMsg = reply.to_string();
                //     std::cout << "Server acknowledgment: " << replyMsg << std::endl;
                // }

                // // Simulate a delay for the next frame (e.g., 30 FPS -> ~33ms per frame)
                // std::this_thread::sleep_for(std::chrono::milliseconds(33));

                // Free allocated memory
                delete[] byteArray;

            }

        }

    }
    // stop the pipeline
    pipeline.stop();

    return 0;
}
catch(ob::Error &e) {
    std::cerr << "function:" << e.getName() << "\nargs:" << e.getArgs() << "\nmessage:" << e.getMessage() << "\ntype:" << e.getExceptionType() << std::endl;
    exit(EXIT_FAILURE);
}


