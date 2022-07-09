#include <iostream>
#include <fstream>
#include <string>

#include <stdlib.h>
#include <time.h>

#include "eigen3/Eigen/Dense"
#include "opencv2/core/eigen.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/radius_outlier_removal.h>

// #include <tf/transform_listener.h>
// #include <tf/transform_datatypes.h>
// #include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc.hpp>

// #define M_PI 3.14f
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
using namespace std;
std::string repo_path = "../";

// static void getAffineTransform_(tf::StampedTransform& TFtransform, Eigen::Affine3f& Afftransform) {
//     Afftransform.translation() << (float)TFtransform.getOrigin()[0],
//                                   (float)TFtransform.getOrigin()[1],
//                                   (float)TFtransform.getOrigin()[2];

//     Afftransform.rotate (Eigen::Quaternionf((float)TFtransform.getRotation().getW(),
//                         (float)TFtransform.getRotation()[0],
//                         (float)TFtransform.getRotation()[1],
//                         (float)TFtransform.getRotation()[2]));
// }

int bins = 10;
int histSize[] = {bins, bins, bins};
float xranges[] = {-0.1, 0.1};
float yranges[] = {-0.1, 0.1};
float zranges[] = {-0.1, 0.1};
int channels[] = {0, 1, 2};
const float *ranges[] = {xranges, yranges, zranges};

static void getEMDDistance_(cv::Mat sig1, PointCloud::Ptr pcl_2, float &emd)
{
  cv::MatND hist_2;
  int num_rows = pcl_2->points.size();
  cv::Mat xyzPts_2(num_rows, 1, CV_32FC3);
  for (int ii = 0; ii < num_rows; ii++)
  {
    xyzPts_2.at<cv::Vec3f>(ii, 0)[0] = pcl_2->points[ii].x;
    xyzPts_2.at<cv::Vec3f>(ii, 0)[1] = pcl_2->points[ii].y;
    xyzPts_2.at<cv::Vec3f>(ii, 0)[2] = pcl_2->points[ii].z;
  }
  cv::calcHist(&xyzPts_2, 1, channels, cv::Mat(), hist_2, 3, histSize, ranges, true, false);

  // make signature
  int sigSize = bins * bins * bins;
  cv::Mat sig2(sigSize, 4, CV_32FC1);

  // //fill value into signature
  for (int x = 0; x < bins; x++)
  {
    for (int y = 0; y < bins; ++y)
    {
      for (int z = 0; z < bins; ++z)
      {
        float binval = hist_2.at<float>(x, y, z);
        sig2.at<float>(x * bins * bins + y * bins + z, 0) = binval;
        sig2.at<float>(x * bins * bins + y * bins + z, 1) = x;
        sig2.at<float>(x * bins * bins + y * bins + z, 2) = y;
        sig2.at<float>(x * bins * bins + y * bins + z, 3) = z;
      }
    }
  }

  emd = cv::EMD(sig1, sig2, cv::DIST_L2); // emd 0 is best matching.
}

cv::Mat eulerAnglesToRotationMatrix(cv::Vec3f theta)
{
  // Calculate rotation about x axis
  cv::Mat R_x = (cv::Mat_<float>(3, 3) << 1, 0, 0,
                 0, cosf(theta[0]), -sinf(theta[0]),
                 0, sinf(theta[0]), cosf(theta[0]));

  // Calculate rotation about y axis
  cv::Mat R_y = (cv::Mat_<float>(3, 3) << cosf(theta[1]), 0, sinf(theta[1]),
                 0, 1, 0,
                 -sin(theta[1]), 0, cosf(theta[1]));

  // Calculate rotation about z axis
  cv::Mat R_z = (cv::Mat_<float>(3, 3) << cosf(theta[2]), -sinf(theta[2]), 0,
                 sinf(theta[2]), cosf(theta[2]), 0,
                 0, 0, 1);

  // Combined rotation matrix
  cv::Mat R = R_z * R_y * R_x;

  return R;
}

#include <opencv2/core/affine.hpp>
int main(int ac, char *av[])
{
  srand(time(NULL));

  PointCloud::Ptr inputCloud(new PointCloud);

  double cent_x, cent_y, cent_z;
  double roll, pitch, yaw;

  pcl::io::loadPCDFile(std::string(av[1]), *inputCloud);

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(inputCloud);
  sor.setLeafSize(0.01f, 0.01f, 0.01f);
  sor.filter(*inputCloud);

  PointCloud::Ptr transformed_cloud(new PointCloud);

  int num_rows = inputCloud->points.size();
  std::cout << num_rows << std::endl;
  cv::Mat xyzPts_1(num_rows, 1, CV_32FC3);
  for (int ii = 0; ii < num_rows; ii++)
  {
    xyzPts_1.at<cv::Vec3f>(ii, 0)[0] = inputCloud->points[ii].x;
    xyzPts_1.at<cv::Vec3f>(ii, 0)[1] = inputCloud->points[ii].y;
    xyzPts_1.at<cv::Vec3f>(ii, 0)[2] = inputCloud->points[ii].z;
  }
  cv::MatND hist_1;
  cv::calcHist(&xyzPts_1, 1, channels, cv::Mat(), hist_1, 3, histSize, ranges, true, false);
  int sigSize = bins * bins * bins;
  cv::Mat sig1(sigSize, 4, CV_32FC1);
  for (int x = 0; x < bins; x++)
  {
    for (int y = 0; y < bins; ++y)
    {
      for (int z = 0; z < bins; ++z)
      {
        float binval = hist_1.at<float>(x, y, z);
        sig1.at<float>(x * bins * bins + y * bins + z, 0) = binval;
        sig1.at<float>(x * bins * bins + y * bins + z, 1) = x;
        sig1.at<float>(x * bins * bins + y * bins + z, 2) = y;
        sig1.at<float>(x * bins * bins + y * bins + z, 3) = z;
      }
    }
  }
  float emd;
  pcl::transformPointCloud(*inputCloud, *transformed_cloud, Eigen::Matrix4f::Identity());

  for (cent_x = -0.02; cent_x <= 0.02; cent_x = cent_x + 0.01)
  {
    for (cent_y = -0.02; cent_y <= 0.02; cent_y = cent_y + 0.01)
    {
      for (cent_z = -0.02; cent_z <= 0.02; cent_x = cent_z + 0.01)
      {
        clock_t begin_time = clock();
        for (roll = 0; roll < M_PI; roll = roll + 0.1)
        {
          for (pitch = 0; pitch < M_PI; pitch = pitch + 0.1)
          {
            for (yaw = 0; yaw < M_PI; yaw = yaw + 0.1)
            {

              getEMDDistance_(sig1, transformed_cloud, emd);
              std::cout << emd << " " << std::endl; //<< std::endl;
              cv::Mat rtma = eulerAnglesToRotationMatrix(cv::Vec3f(yaw, pitch, roll));
              cv::Vec3f tfsad(cent_x, cent_y, cent_z);
              cv::Affine3f cvpose(rtma, tfsad); //=cv::Affine3f::Identity();//=cv::Affine3d::Identity();
              Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
              cv::Mat aa(cvpose.matrix);
              cv::cv2eigen(aa, transform_1);
              pcl::transformPointCloud(*inputCloud, *transformed_cloud, transform_1);
            }
          }
        }
        std::cout << "time: " << float(clock() - begin_time) / CLOCKS_PER_SEC << std::endl;
      }
    }
  }

  return 0;
}
