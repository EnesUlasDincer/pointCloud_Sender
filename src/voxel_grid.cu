#include <cassert>
#include <iostream>
#include <vector>

#include "../include/voxel_grid.cuh"
#include <stdio.h>
#include <stdlib.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <Eigen/Dense>

#include <thrust/device_vector.h>
#include <thrust/remove.h>
#include <thrust/host_vector.h>
#include <thrust/sort.h>
#include <thrust/copy.h>
#include <thrust/iterator/counting_iterator.h>

using namespace std;

// Conversion function from OBColorPoint to Point
Point toPoint(const OBColorPoint& obPoint) {
    return Point(obPoint.x, obPoint.y, obPoint.z, obPoint.r, obPoint.g, obPoint.b);
}

// Helper structures for CUDA operations
struct PointToKey {
    ui32XYZ voxel_nums;
    fXYZ voxel_lengths;
    fXYZ min_xyz;

    PointToKey(ui32XYZ _voxel_nums, fXYZ _voxel_lengths, fXYZ _min_xyz)
        : voxel_nums(_voxel_nums), voxel_lengths(_voxel_lengths), min_xyz(_min_xyz) {}

    __host__ __device__
    uint32_t operator()(const Point v) const {
        if (isnan(v.x)) return UINT32_MAX;
        uint32_t idx_x = round((v.x - min_xyz.x) / voxel_lengths.x);
        uint32_t idx_y = round((v.y - min_xyz.y) / voxel_lengths.y);
        uint32_t idx_z = round((v.z - min_xyz.z) / voxel_lengths.z);
        return idx_x + idx_y * voxel_nums.x + idx_z * voxel_nums.x * voxel_nums.y;
    }
};

struct PointToColor {
    __host__ __device__
    ui64RGB operator()(const Point p) const {
        return ui64RGB(p.r * p.r, p.g * p.g, p.b * p.b);
    }
};

struct IdxColorWeightToPoint {
    ui32XYZ voxel_nums;
    fXYZ voxel_lengths;
    fXYZ min_xyz;

    IdxColorWeightToPoint(ui32XYZ _voxel_nums, fXYZ _voxel_lengths, fXYZ _min_xyz)
        : voxel_nums(_voxel_nums), voxel_lengths(_voxel_lengths), min_xyz(_min_xyz) {}

    __host__ __device__
    Point operator()(thrust::tuple<uint32_t, ui64RGB, uint32_t> t) const {
        uint32_t idx = thrust::get<0>(t);
        ui64RGB color = thrust::get<1>(t);
        uint32_t weight = thrust::get<2>(t);

        if (voxel_nums.x == 0 || voxel_nums.y == 0 || voxel_nums.z == 0) {
            printf("Error: Invalid voxel_nums (x=%u, y=%u, z=%u)\n", voxel_nums.x, voxel_nums.y, voxel_nums.z);
            return Point();
        }

        uint32_t idx_z = idx / (voxel_nums.x * voxel_nums.y);
        uint32_t idx_y = (idx - (idx_z * voxel_nums.x * voxel_nums.y)) / voxel_nums.x;
        uint32_t idx_x = idx % voxel_nums.x;

        Point p;
        p.x = ((float) idx_x) * voxel_lengths.x + min_xyz.x;
        p.y = ((float) idx_y) * voxel_lengths.y + min_xyz.y;
        p.z = ((float) idx_z) * voxel_lengths.z + min_xyz.z;

        // printf("I am here\n");
        
        p.r = (float) round(sqrt((float) (color.r / weight)));
        p.g = (float) round(sqrt((float) (color.g / weight)));
        p.b = (float) round(sqrt((float) (color.b / weight)));
        
        // if (weight > 0) {
        //     p.r = sqrtf((float)(color.r / weight));
        //     p.g = sqrtf((float)(color.g / weight));
        //     p.b = sqrtf((float)(color.b / weight));
        // } else {
        //     p.r = p.g = p.b = 0.0f;
        // }

        return p;
    }
};


struct is_point_invalid {
    __host__ __device__
    bool operator()(const Point p) const { return isnan(p.x); }
};

struct TFAndCropPoint {
    Eigen::Matrix4f tf;
    fXYZ min_xyz;
    fXYZ max_xyz;

    TFAndCropPoint(Eigen::Matrix4f _tf, fXYZ _min_xyz, fXYZ _max_xyz)
        : tf(_tf), min_xyz(_min_xyz), max_xyz(_max_xyz) {}

    __host__ __device__
    Point operator()(const Point p) const {

        Point np;

        np.x = tf(0, 0) * p.x + tf(0, 1) * p.y + tf(0, 2) * p.z + tf(0, 3); 
        np.y = tf(1, 0) * p.x + tf(1, 1) * p.y + tf(1, 2) * p.z + tf(1, 3); 
        np.z = tf(2, 0) * p.x + tf(2, 1) * p.y + tf(2, 2) * p.z + tf(2, 3); 
        np.r = p.r;
        np.g = p.g;
        np.b = p.b;

        if (np.x < min_xyz.x || np.x > max_xyz.x ||
            np.y < min_xyz.y || np.y > max_xyz.y ||
            np.z < min_xyz.z || np.z > max_xyz.z) {
            // printf("Invalid point: (%f, %f, %f)\n", p.x, p.y, p.z);
            return Point(NAN, NAN, NAN, 0, 0, 0);
        }
    
        return np;
    }
};

uint32_t voxel_grid(thrust::device_vector<Point>& d_points, float* out, const ui32XYZ& voxel_nums, const fXYZ& voxel_lengths, const fXYZ& min_xyz) {
    uint32_t num = d_points.size();

    // Step 1: Map points to colors and voxel indices
    // std::cout << "Step 1: Map points to colors and voxel indices" << std::endl;
    thrust::device_vector<ui64RGB> d_colors(num);
    thrust::device_vector<uint32_t> d_voxel_idxs(num);
    thrust::transform(d_points.begin(), d_points.end(), d_colors.begin(), PointToColor());
    thrust::transform(d_points.begin(), d_points.end(), d_voxel_idxs.begin(), PointToKey(voxel_nums, voxel_lengths, min_xyz));

    // Step 2: Sort points by voxel index
    // std::cout << "Step 2: Sort points by voxel index" << std::endl;
    thrust::device_vector<uint32_t> d_point_idxs(num);
    thrust::sequence(d_point_idxs.begin(), d_point_idxs.end());
    thrust::sort_by_key(d_voxel_idxs.begin(), d_voxel_idxs.end(), d_point_idxs.begin());

    // Step 3: Compute voxel histogram
    // std::cout << "Step 3: Compute voxel histogram" << std::endl;
    thrust::device_vector<uint32_t> d_weights(num);
    thrust::device_vector<uint32_t> d_idx_reduced(num);
    auto new_ends = thrust::reduce_by_key(d_voxel_idxs.begin(), d_voxel_idxs.end(),
                                          thrust::constant_iterator<uint32_t>(1),
                                          d_idx_reduced.begin(), d_weights.begin());
    uint32_t num_voxels = new_ends.first - d_idx_reduced.begin();
    d_weights.resize(num_voxels);
    d_idx_reduced.resize(num_voxels);

    // Step 4: Aggregate voxel colors
    // std::cout << "Step 4: Aggregate voxel colors" << std::endl;
    thrust::device_vector<ui64RGB> d_colors_out(num_voxels);
    thrust::reduce_by_key(d_voxel_idxs.begin(), d_voxel_idxs.end(),
                          thrust::make_permutation_iterator(d_colors.begin(), d_point_idxs.begin()),
                          d_idx_reduced.begin(), d_colors_out.begin());

    // Step 5: Compute voxel centroids
    // std::cout << "Step 5: Compute voxel centroids" << std::endl;
    thrust::device_vector<Point> d_point_cloud_out(num_voxels);
    auto zip_begin = thrust::make_zip_iterator(thrust::make_tuple(d_idx_reduced.begin(), d_colors_out.begin(), d_weights.begin()));
    auto zip_end = thrust::make_zip_iterator(thrust::make_tuple(d_idx_reduced.end(), d_colors_out.end(), d_weights.end()));
    
    // std::cout << "transforming" << std::endl;
    thrust::transform(zip_begin, zip_end, d_point_cloud_out.begin(), IdxColorWeightToPoint(voxel_nums, voxel_lengths, min_xyz));

    // Copy result to output
    // std::cout << "Copying result to output" << std::endl;
    thrust::copy(d_point_cloud_out.begin(), d_point_cloud_out.end(), reinterpret_cast<Point*>(out));

    // std::cout << "Number of voxels: " << num_voxels << std::endl;
    return num_voxels;
}

#define CUDA_CHECK(call)                                               \
    do {                                                               \
        cudaError_t err = call;                                        \
        if (err != cudaSuccess) {                                      \
            std::cerr << "CUDA Error: " << cudaGetErrorString(err)     \
                      << " at " << __FILE__ << ":" << __LINE__ << "\n";\
            exit(EXIT_FAILURE);                                        \
        }                                                              \
    } while (0)

// uint32_t transformCropAndVoxelizeCenter(std::vector<OBColorPoint>& points, float* point_cloud_out, Eigen::Matrix4f& T_camera_to_QR) {
//     size_t num_points = points.size();
//     if (num_points == 0) {
//         std::cerr << "Error: No input points\n";
//         return 0;
//     }

//     fXYZ min_xyz(-300.0f, -300.0f, -500.0f);
//     fXYZ max_xyz(+130.0f, +130.0f, 2000.0f);
//     fXYZ voxel_lengths( 1, 1, 1);
//     ui32XYZ voxel_nums(ceil((max_xyz.x - min_xyz.x) / voxel_lengths.x),
//                        ceil((max_xyz.y - min_xyz.y) / voxel_lengths.y),
//                        ceil((max_xyz.z - min_xyz.z) / voxel_lengths.z));

//     Eigen::Matrix4f tf;
//     //  = T_camera_to_QR;
//     // printf("T_camera_to_QR: \n");
//     // std::cout << T_camera_to_QR << std::endl;
//     tf << 1, 0, 0, 0,
//           0, 1, 0, 0,
//           0, 0, 1, 0,
//           0, 0, 0, 1;


//     // thrust::host_vector<Point> h_points(num_points);
//     // for (size_t i = 0; i < num_points; ++i) {
//     //     h_points[i] = toPoint(points[i]);
//     // }

//     // thrust::device_vector<Point> d_points = h_points;

//     thrust::device_vector<Point> d_points(points.size()); // Output
//     thrust::copy(
//         reinterpret_cast<const Point*>(&points[0]),
//         reinterpret_cast<const Point*>(&points[0] + points.size()),
//         d_points.begin());


//     // std
//     std::cout << "Number of points before filtering: " << d_points.size() << std::endl;

//     thrust::transform(d_points.begin(), d_points.end(), d_points.begin(), TFAndCropPoint(tf, min_xyz, max_xyz));
    
//     CUDA_CHECK(cudaDeviceSynchronize());


//     size_t new_size = thrust::remove_if(d_points.begin(), d_points.end(), is_point_invalid()) - d_points.begin();
//     if (new_size > d_points.size()) {
//         std::cerr << "Error: new_size exceeds original size\n";
//         exit(EXIT_FAILURE);
//     }
//     d_points.resize(new_size);

//     CUDA_CHECK(cudaDeviceSynchronize());

//     std::cout << "Number of points after filtering: " << new_size << std::endl;
//     return voxel_grid(d_points, point_cloud_out, voxel_nums, voxel_lengths, min_xyz);
// }


uint32_t transformCropAndVoxelizeCenter(OBColorPoint* points, size_t num_points, float* point_cloud_out, Eigen::Matrix4f& T_camera_to_QR) {
    if (num_points == 0) {
        std::cerr << "Error: No input points\n";
        return 0;
    }

    fXYZ min_xyz(-3000.0f, -3000.0f, -1000.0f);
    fXYZ max_xyz(+6000.0f, +6000.0f, 5000.0f);
    fXYZ voxel_lengths(1, 1, 1);
    ui32XYZ voxel_nums(
        ceil((max_xyz.x - min_xyz.x) / voxel_lengths.x),
        ceil((max_xyz.y - min_xyz.y) / voxel_lengths.y),
        ceil((max_xyz.z - min_xyz.z) / voxel_lengths.z));

    Eigen::Matrix4f tf = T_camera_to_QR;  // Use input transform
    // Eigen::Matrix4f tf;  // Use input transform
    // tf << 1, 0, 0, 0,
    //       0, 1, 0, 0,
    //       0, 0, 1, 0,
    //       0, 0, 0, 1;

    // CUDA THRUST - NO COPYING TO `std::vector`
    thrust::device_vector<Point> d_points(num_points);
    thrust::copy(
        reinterpret_cast<const Point*>(points),
        reinterpret_cast<const Point*>(points) + num_points,
        d_points.begin());

    // std::cout << "Number of points before filtering: " << d_points.size() << std::endl;

    thrust::transform(d_points.begin(), d_points.end(), d_points.begin(), TFAndCropPoint(tf, min_xyz, max_xyz));
    // CUDA_CHECK(cudaDeviceSynchronize());

    size_t new_size = thrust::remove_if(d_points.begin(), d_points.end(), is_point_invalid()) - d_points.begin();
    d_points.resize(new_size);

    // CUDA_CHECK(cudaDeviceSynchronize());

    std::cout << "Number of points after filtering: " << new_size << std::endl;
    return voxel_grid(d_points, point_cloud_out, voxel_nums, voxel_lengths, min_xyz);
}
