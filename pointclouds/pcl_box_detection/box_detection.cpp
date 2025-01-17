#include <format>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

static std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
find_planes(const pcl::PointCloud<pcl::PointXYZ> &cloud) {
  pcl::ModelCoefficients coefficients{};
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices{});

  pcl::SACSegmentation<pcl::PointXYZ> seg{};
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(500);
  seg.setDistanceThreshold(30.0);

  pcl::PointCloud<pcl::PointXYZ>::Ptr remaining{
      new pcl::PointCloud<pcl::PointXYZ>{cloud}};
  pcl::ExtractIndices<pcl::PointXYZ> extract{};
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> res{};
  for (int i = 0; i < 4; ++i) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane{
        new pcl::PointCloud<pcl::PointXYZ>{}};
    seg.setInputCloud(remaining);
    seg.segment(*inliers, coefficients);

    extract.setInputCloud(remaining);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*plane);
    std::cout << "PointCloud representing the planar component: "
              << plane->width * plane->height << " data points.\n";
    res.push_back(std::move(plane));

    extract.setNegative(true);
    extract.filter(*remaining);
    seg.setDistanceThreshold(5.0);
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
  bool first{true};

  long frame{50};
  while (true) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{
        new pcl::PointCloud<pcl::PointXYZ>};
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(
            std::format("./basic-clouds/cloud{:04d}.pcd", frame), *cloud)) {
      PCL_ERROR("Couldn't read pointcloud\n");
      return EXIT_FAILURE;
    }

    auto planes{find_planes(*cloud)};
    if (first) {
      viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud", v1);
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
      first = false;
    } else {
      viewer->updatePointCloud(cloud, "cloud");
      int i = 0;
      for (const auto &plane : planes) {
        viewer->updatePointCloud(plane, "plane" + std::to_string(i));
        ++i;
      }
    }

    viewer->spinOnce(10000000);

    if (viewer->wasStopped()) {
      break;
    }
    ++frame;
  }

  return 0;
}
