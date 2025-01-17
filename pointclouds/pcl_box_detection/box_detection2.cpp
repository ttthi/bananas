#include <cstddef>
#include <format>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <pcl/filters/extract_indices.h>
// TODO: ???
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/visualization/pcl_visualizer.h>

static pcl::PointCloud<pcl::Normal>::Ptr
compute_normals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                const pcl::search::Search<pcl::PointXYZ>::Ptr &tree) {
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(cloud);
  normal_estimator.setKSearch(50);
  normal_estimator.compute(*normals);

  return normals;
}

static std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
find_planes(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
            const pcl::search::Search<pcl::PointXYZ>::Ptr &tree,
            const pcl::PointCloud<pcl::Normal>::Ptr normals) {
  pcl::IndicesPtr indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*cloud, *indices);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize(20);
  reg.setMaxClusterSize(1000000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(50);
  reg.setInputCloud(cloud);
  reg.setIndices(indices);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold(0.5);

  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);

  pcl::ExtractIndices<pcl::PointXYZ> extract{};
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> res{};
  for (std::size_t i = 0; i < std::min(clusters.size(), std::size_t{4}); ++i) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane{
        new pcl::PointCloud<pcl::PointXYZ>{}};
    pcl::PointIndices::Ptr inliers(
        new pcl::PointIndices{std::move(clusters[i])});
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane);
    std::cout << "PointCloud representing the planar component: "
              << plane->width * plane->height << " data points.\n";
    res.push_back(std::move(plane));
  }

  return res;
}

int main() {
  std::ios_base::sync_with_stdio(false);

  auto viewer{
      std::make_unique<pcl::visualization::PCLVisualizer>("Cloud Viewer")};

  int v1{0};
  int v2{0};
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(0.5, 0.6, 0.7, v1);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->addCoordinateSystem(1.0);

  int frame{50};
  bool first{true};
  while (true) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{
        new pcl::PointCloud<pcl::PointXYZ>};
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(
            std::format("./basic-clouds/cloud{:04d}.pcd", frame), *cloud)) {
      PCL_ERROR("Couldn't read pointcloud\n");
      return EXIT_FAILURE;
    }

    pcl::search::Search<pcl::PointXYZ>::Ptr tree{
        new pcl::search::KdTree<pcl::PointXYZ>};
    auto normals{compute_normals(cloud, tree)};
    auto planes{find_planes(cloud, tree, normals)};
    std::cout << std::flush;

    int i = 0;
    for (const auto &plane : planes) {
      viewer->addPointCloud<pcl::PointXYZ>(plane, "plane" + std::to_string(i),
                                           v2);
      ++i;
    }
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "plane0");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "plane1");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "plane2");
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.0, 0.5, "plane3");

    if (first) {
      viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);
      viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
          cloud, normals, 4, 5.0f, "normals", v1);

      first = false;
    } else {
      viewer->updatePointCloud<pcl::PointXYZ>(cloud, "cloud");
      viewer->removePointCloud("normals", v1);
      viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(
          cloud, normals, 4, 5.0f, "normals", v1);
    }

    viewer->spinOnce(100000);

    if (viewer->wasStopped()) {
      break;
    }
    for (std::size_t i = 0; i < planes.size(); ++i) {
      viewer->removePointCloud("plane" + std::to_string(i), v2);
    }
    ++frame;
  }

  return 0;
}
