#include <zmq.hpp>
#include <iostream>
#include <vector>
#include <sstream>
#include <string>

// Struct for a 3D point with RGB
struct Point3D {
    float x, y, z;
    int r, g, b; // RGB color
};

// Deserialize a string with RGB values into a vector of Point3D
std::vector<Point3D> deserializePointCloud(const std::string& data) {
    std::vector<Point3D> points;
    std::istringstream iss(data);
    std::string line;

    while (std::getline(iss, line)) {
        std::istringstream lineStream(line);
        std::string xStr, yStr, zStr, rStr, gStr, bStr;

        if (std::getline(lineStream, xStr, ',') &&
            std::getline(lineStream, yStr, ',') &&
            std::getline(lineStream, zStr, ',') &&
            std::getline(lineStream, rStr, ',') &&
            std::getline(lineStream, gStr, ',') &&
            std::getline(lineStream, bStr, ',')) {
            points.push_back({
                std::stof(xStr), std::stof(yStr), std::stof(zStr),
                std::stoi(rStr), std::stoi(gStr), std::stoi(bStr)
            });
        }
    }

    return points;
}

int main() {
    zmq::context_t context(1);
    zmq::socket_t socket(context, zmq::socket_type::rep);
    socket.bind("tcp://*:2555");

    std::cout << "Server is running on tcp://127.0.0.1:5555\n";

    while (true) {
        zmq::message_t request;

        // Receive the point cloud
        auto result = socket.recv(request, zmq::recv_flags::none);
        if (!result) {
            std::cerr << "Failed to receive message\n";
            continue;
        }

        std::string receivedData = request.to_string();
        std::cout << "Received point cloud data:\n" << receivedData << std::endl;

        // Deserialize the point cloud
        std::vector<Point3D> pointCloud = deserializePointCloud(receivedData);

        // Process the deserialized points (example: print the first point)
        if (!pointCloud.empty()) {
            const auto& firstPoint = pointCloud[0];
            std::cout << "First Point: (" << firstPoint.x << ", " << firstPoint.y
                      << ", " << firstPoint.z << ") Color: ("
                      << firstPoint.r << ", " << firstPoint.g << ", "
                      << firstPoint.b << ")\n";
        }

        // Send acknowledgment to the client
        std::string replyMsg = "Frame received. Total points: " + std::to_string(pointCloud.size());
        zmq::message_t reply(replyMsg.size());
        memcpy(reply.data(), replyMsg.c_str(), replyMsg.size());
        socket.send(reply, zmq::send_flags::none);
    }

    return 0;
}
