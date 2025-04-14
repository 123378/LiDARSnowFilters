// BETA
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <chrono>
#include <thread>
#include <mutex>

using namespace std;
using namespace std::chrono; 
void optimized_ROR(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& normal_ps,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& snow_ps,
    float radius = 0.3f,
    int min_neighbors = 3,
    unsigned int thread_num = std::thread::hardware_concurrency())
{

    const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree = [&](){
        auto tree = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ>>();
        tree->setInputCloud(input_cloud);
        return *tree;
    }();

    normal_ps->clear();
    snow_ps->clear();
    normal_ps->reserve(input_cloud->size() * 0.8); 
    snow_ps->reserve(input_cloud->size() * 0.8);

    std::mutex normal_mtx, snow_mtx;

    auto process_chunk = [&](size_t start, size_t end) {
        std::vector<int> indices;
        std::vector<float> dists;
        indices.reserve(100); 

        for (size_t i = start; i < end; ++i) {
            const int neighbors = kdtree.radiusSearch(
                input_cloud->points[i], 
                radius, 
                indices, 
                dists, 
            );

            std::lock_guard<std::mutex> lock(neighbors >= min_neighbors ? normal_mtx : snow_mtx);
            if (neighbors >= min_neighbors) {
                normal_ps->push_back(input_cloud->points[i]);
            } else {
                snow_ps->push_back(input_cloud->points[i]);
            }
        }
    };

    const size_t total_points = input_cloud->size();
    const size_t chunk_size = total_points / thread_num;
    std::vector<std::thread> threads;

    for (unsigned int t = 0; t < thread_num; ++t) {
        const size_t start = t * chunk_size;
        const size_t end = (t == thread_num - 1) ? total_points : start + chunk_size;
        threads.emplace_back(process_chunk, start, end);
    }

    for (auto& thread : threads) {
        thread.join();
    }
}
pcl::PointCloud<pcl::PointXYZ>::Ptr loadBinFile(const string& filename) 
{

    file.seekg(0, ios::end);
    size_t file_size = file.tellg();
    file.seekg(0, ios::beg);

    const size_t point_size = 4 * sizeof(float);
    size_t num_points = file_size / point_size;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = static_cast<uint32_t>(num_points);
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(num_points);

    vector<float> buffer(4 * num_points);
    file.read(reinterpret_cast<char*>(buffer.data()), file_size);

    for (size_t i = 0; i < num_points; ++i) {
        cloud->points[i].x = buffer[4 * i];
        cloud->points[i].y = buffer[4 * i + 1];
        cloud->points[i].z = buffer[4 * i + 2];
    }

    return cloud;
}

void d_ror(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
          pcl::PointCloud<pcl::PointXYZ>::Ptr& normal_ps,
          pcl::PointCloud<pcl::PointXYZ>::Ptr& snow_ps,
          float SRmin = 0.12f, float beta = 0.12f, float alpha = 0.2f)
{
    vector<float> radii(input_cloud->size());
    for (size_t i = 0; i < input_cloud->size(); ++i) {
        const auto& pt = input_cloud->points[i];
        float distance = sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
        radii[i] = (distance < SRmin) ? SRmin : beta * distance * alpha;
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(input_cloud);

    normal_ps->clear();
    snow_ps->clear();
    for (size_t i = 0; i < input_cloud->size(); ++i) {
        vector<int> indices;
        vector<float> dists;
        
        int neighbors = kdtree.radiusSearch(
            input_cloud->points[i], 
            radii[i], 
            indices, 
            dists
        );

        if (neighbors < 3) {
            snow_ps->push_back(input_cloud->points[i]);
        } else {
            normal_ps->push_back(input_cloud->points[i]);
        }
    }
}
void ROR(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& normal_ps,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& snow_ps,
    float radius = 0.5f,
    int min_neighbors = 3)
{
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(input_cloud);
    normal_ps->clear();
    snow_ps->clear();
    normal_ps->reserve(input_cloud->size());
    snow_ps->reserve(input_cloud->size());

    for (size_t i = 0; i < input_cloud->size(); ++i) {
        std::vector<int> indices;
        std::vector<float> dists;
        int neighbors = kdtree.radiusSearch(
            input_cloud->points[i],
            radius,
            indices,
            dists
        );

        if (neighbors >= min_neighbors) {
            normal_ps->push_back(input_cloud->points[i]);
        } else {
            snow_ps->push_back(input_cloud->points[i]);
        }
    }
}
