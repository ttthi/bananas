#include <format>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <depthai/depthai.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main() {
  std::ios_base::sync_with_stdio(false);

  auto pipeline = dai::Pipeline{};
  auto monoLeft = pipeline.create<dai::node::MonoCamera>();
  auto monoRight = pipeline.create<dai::node::MonoCamera>();
  auto depth = pipeline.create<dai::node::StereoDepth>();
  auto pointcloud = pipeline.create<dai::node::PointCloud>();
  auto xout = pipeline.create<dai::node::XLinkOut>();
  auto xoutDepth = pipeline.create<dai::node::XLinkOut>();

  monoLeft->setResolution(
      dai::MonoCameraProperties::SensorResolution::THE_720_P);
  monoLeft->setCamera("left");
  monoRight->setResolution(
      dai::MonoCameraProperties::SensorResolution::THE_720_P);
  monoRight->setCamera("right");

  // Create a node that will produce the depth map (using disparity output as
  // it's easier to visualize depth this way)
  depth->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::DEFAULT);
  // Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
  depth->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
  depth->setLeftRightCheck(true);
  depth->setExtendedDisparity(false);
  depth->setSubpixel(true);

  xout->setStreamName("out");
  xoutDepth->setStreamName("depth");

  monoLeft->out.link(depth->left);
  monoRight->out.link(depth->right);
  depth->depth.link(pointcloud->inputDepth);
  depth->disparity.link(xoutDepth->input);
  pointcloud->outputPointCloud.link(xout->input);
  pointcloud->initialConfig.setSparse(true);

  auto viewer{
      std::make_unique<pcl::visualization::PCLVisualizer>("Cloud Viewer")};

  bool first{true};

  dai::Device device{pipeline};
  device.setIrLaserDotProjectorIntensity(0.2);

  auto q{device.getOutputQueue("out", 8, false)};
  auto qDepth{device.getOutputQueue("depth", 8, false)};
  long counter{0};
  while (true) {
    std::cout << "Waiting for data\n";
    auto depthImg{qDepth->get<dai::ImgFrame>()};
    auto pclMsg{q->get<dai::PointCloudData>()};
    std::cout << "Got data\n";
    if (!pclMsg) {
      std::cout << "No data\n";
      continue;
    }

    auto frame{depthImg->getCvFrame()};
    frame.convertTo(frame, CV_8UC1,
                    255 / depth->initialConfig.getMaxDisparity());
    cv::imshow("depth", frame);
    cv::waitKey(1);

    if (pclMsg->getPoints().empty()) {
      std::cout << "Empty point cloud\n";
      continue;
    }
    std::cout << "Number of points: " << pclMsg->getPoints().size() << '\n';
    std::cout << "Min x: " << pclMsg->getMinX() << '\n';
    std::cout << "Min y: " << pclMsg->getMinY() << '\n';
    std::cout << "Min z: " << pclMsg->getMinZ() << '\n';
    std::cout << "Max x: " << pclMsg->getMaxX() << '\n';
    std::cout << "Max y: " << pclMsg->getMaxY() << '\n';
    std::cout << "Max z: " << pclMsg->getMaxZ() << '\n';
    std::cout << std::flush;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{pclMsg->getPclData()};
    // TODO: Why is this needed?
    cloud->width = cloud->size();
    cloud->height = 1;

    if (first) {
      viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
      first = false;
    } else {
      viewer->updatePointCloud(cloud, "cloud");
    }
    pcl::io::savePCDFileBinaryCompressed(std::format("/dev/shm/hires-clouds/cloud{:04d}.pcd", counter), *cloud);

    viewer->spinOnce(10);

    if (viewer->wasStopped()) {
      break;
    }
    ++counter;
  }

  return 0;
}
