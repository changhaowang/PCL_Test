#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <fstream>
#include <iosfwd>
#include <math.h>
#include <iomanip>
//#include "direct.h"
#include"stdlib.h"
#include "iostream"
#include "string"
using namespace std;



int main(int argc, char** argv)
{
  // 初始化点云对象
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);  //存储源点云
  //pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);   //存储提取的局内点

  // 填充点云数据
  cloud->width    = 4000;                 //填充点云数目
   cloud->height   = 1;                     //无序点云
  cloud->is_dense = false;
  cloud->points.resize (cloud->width * cloud->height);
  //设置顶面
  for (size_t i = 0;i < cloud->points.size()/4;i++)
  {
    cloud->points[i].x = pow(-1.0,rand())*3.0* rand () / (RAND_MAX);
    cloud->points[i].y = pow(-1.0,rand())*0.5* rand () / (RAND_MAX);
    cloud->points[i].z = 3;
  }
  for(size_t i = cloud->points.size()/4;i < 2*cloud->points.size()/4;i++)
  {
    cloud->points[i].x = pow(-1.0,rand())*3.0* rand () / (RAND_MAX);
    cloud->points[i].y = pow(-1.0,rand())*0.5* rand () / (RAND_MAX);
    cloud->points[i].z = -3;
  }
  for(size_t i = 2*cloud->points.size()/4;i < 3*cloud->points.size()/4;i++)
  {
    cloud->points[i].x = pow(-1.0,rand())*3.0* rand () / (RAND_MAX);
    cloud->points[i].y = 0.5;
    cloud->points[i].z = pow(-1.0,rand()) * 3.0 * rand () / (RAND_MAX);
  }
  for(size_t i = 3*cloud->points.size()/4;i < 4*cloud->points.size()/4;i++)
  {
    cloud->points[i].x = pow(-1.0,rand())*3.0* rand () / (RAND_MAX);
    cloud->points[i].y = -0.5;
    cloud->points[i].z = pow(-1.0,rand()) * 3.0 * rand () / (RAND_MAX);
  }

  //const char* filepath = "/home/wang/Desktop/pcl_test/indoor_segmentation/build";
  pcl::io::savePCDFileASCII("IndoorScene111.pcd", *cloud);
}
