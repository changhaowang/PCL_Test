//#include "ros/ros.h"
//#include "std_msgs/String.h"
//#include <sensor_msgs/PointCloud2.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/conversions.h>
//#include <pcl/common/impl/io.hpp>
#include <math.h>
#include <stdio.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include "boost/bind.hpp"
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include<iostream>

class PN
{
public:
  pcl::PointXYZ point;
  float normal1[3] ;
  float normal2[3] ;
};

int main(int argc, char **argv)
{
//  Parser parser;
//  parser.addGroup(LocalConfig());
//  parser.addGroup(GeneralConfig());
//  parser.read(argc, argv);
  //ros::init(argc, argv, "recognition");
  //ros::NodeHandle nh;
  //Segmentation segment(nh);
  //segment.init(nh);
  int counter = 0;
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  reader.read("IndoorScene.pcd", *cloud);
  //std::cout << "PointCloud before filtering has: " << cloud->points.size() << " data points." << std::endl;
  // Create the filtering object: downsample the dataset using a leaf size of 10cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.1f, 0.1f, 0.1f);
  vg.filter (*cloud_filtered);
  //normal computation
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
  //tree->setInputCloud (cloud_filtered);
  ne.setInputCloud (cloud_filtered);
  ne.setSearchMethod (tree);
  ne.setRadiusSearch (0.2) ;
  //开始进行法向计算
  ne.compute (*normals);
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud_filtered, *normals, *cloud_with_normals);
  //make a new point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  cloud_f = (new pcl::PointCloud<pcl::PointXYZ>)->makeShared();
  for(size_t i_point = 0; i_point <cloud_with_normals->points.size();i_point++)
  {
    if((pow(cloud_with_normals->points[i_point].normal[0],2)+pow(cloud_with_normals->points[i_point].normal[1],2)+pow(cloud_with_normals->points[i_point].normal[2]-1,2)) > 1 && (pow(cloud_with_normals->points[i_point].normal[0],2)+pow(cloud_with_normals->points[i_point].normal[1],2)+pow(cloud_with_normals->points[i_point].normal[2]+1,2)) > 1)
    {
      //cloud_filtered->erase(i_point);
      pcl::PointXYZ point;
      point.x = (cloud_with_normals->points[i_point].x);
      point.y = (cloud_with_normals->points[i_point].y);
      point.z = (cloud_with_normals->points[i_point].z);
      cloud_f->points.push_back(point);
      counter++;
    }
  }
  cloud_f->width = counter;
  cloud_f->height = 1;
  cloud_f->is_dense = false;
  pcl::io::savePCDFileASCII("IndoorScene2.pcd", *cloud_f);
  //Euclidean segmentation
  //tree->setInputCloud (cloud_f);//创建点云索引向量，用于存储实际的点云信息
  std::vector<pcl::PointIndices> cluster_e;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.5); //设置近邻搜索的搜索半径为50cm
  ec.setMinClusterSize (100);//设置一个聚类需要的最少点数目为100
  ec.setMaxClusterSize (250000); //设置一个聚类需要的最大点数目为25000
  ec.setSearchMethod (tree);//设置点云的搜索机制
  ec.setInputCloud (cloud_f);
  ec.extract (cluster_e);//从点云中提取聚类，并将点云索引保存在cluster_indices中
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud_e[cluster_e.size()] ;
  //colored_cloud_e = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
  std::vector<unsigned int> colors(3,1);
  //colored_cloud_e->is_dense = cloud_f->is_dense;
  //counter = 0;
  for(size_t i_cluster = 0; i_cluster < cluster_e.size(); i_cluster++)
  {
    colored_cloud_e[i_cluster] = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared();
    counter = 0;
    colors[0] = rand() % 256;
    colors[1] = rand() % 256;
    colors[2] = rand() % 256;
    colored_cloud_e[i_cluster]->is_dense = cloud_f->is_dense;
    for(size_t i_point = 0; i_point <cluster_e[i_cluster].indices.size();i_point++)
    {
      pcl::PointXYZRGB point;
      point.x = (cloud_f->points[cluster_e[i_cluster].indices[i_point]].x);
      point.y = (cloud_f->points[cluster_e[i_cluster].indices[i_point]].y);
      point.z = (cloud_f->points[cluster_e[i_cluster].indices[i_point]].z);
      point.r = colors[0];
      point.g = colors[1];
      point.b = colors[2];
      colored_cloud_e[i_cluster]->points.push_back(point);
      //std::cerr<<"point.z"<<point.z<<std::endl;
      counter++;
    }
    colored_cloud_e[i_cluster]->width = counter;
    colored_cloud_e[i_cluster]->height = 1;
    std::stringstream ss;
    ss << "cloud_cluster_e_" << i_cluster << ".pcd";
    pcl::io::savePCDFileASCII(ss.str(), *colored_cloud_e[i_cluster]);
  }

  //
  std::vector <pcl::PointIndices> cluster_r[cluster_e.size()];
  pcl::search::Search<pcl::PointXYZRGB>::Ptr tree2 = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> >(new pcl::search::KdTree<pcl::PointXYZRGB>);


  float sum_x[cluster_e.size()];
  float average_x[cluster_e.size()];
  float sum_y[cluster_e.size()];
  float average_y[cluster_e.size()];
  float sum_z[cluster_e.size()];
  float average_z[cluster_e.size()];
  for(size_t i_cluster = 0; i_cluster < cluster_e.size(); i_cluster++)
  {
    colors[0] = rand() % 256;
    colors[1] = rand() % 256;
    colors[2] = rand() % 256;
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne2;
    ne2.setInputCloud (colored_cloud_e[i_cluster]);
    ne2.setSearchMethod (tree2);
    ne2.setRadiusSearch (0.5) ;
    ne2.compute (*normals);
    //float sum_x = 0.0;
    //float average_x = 0.0;
    //float sum_y = 0.0;
    //float average_y = 0.0;
    //float sum_z = 0.0;
    //float average_z = 0.0;
    int counter = 0;
    sum_x[i_cluster] = 0.0;
    sum_y[i_cluster] = 0.0;
    sum_z[i_cluster] = 0.0;
    for(size_t i_point = 0; i_point <cluster_e[i_cluster].indices.size();i_point++)
    {
      //std::cerr<<normals->points[i_point].curvature<<endl;
      if (normals->points[i_point].curvature > 0.05)
      {
        counter++;
        sum_x[i_cluster] = sum_x[i_cluster] + colored_cloud_e[i_cluster]->points[i_point].x;
        sum_y[i_cluster] = sum_y[i_cluster] + colored_cloud_e[i_cluster]->points[i_point].y;
        sum_z[i_cluster] = sum_z[i_cluster] + colored_cloud_e[i_cluster]->points[i_point].z;
      }
     }
    average_x[i_cluster] = sum_x[i_cluster]/(counter);
    average_y[i_cluster] = sum_y[i_cluster]/(counter);
    average_z[i_cluster] = sum_z[i_cluster]/(counter);
    //std::cerr<<normals->points[1].curvature<<std::endl;
    //std:cerr<<"sum_x"<<sum_x[i_cluster]<<std::endl;
    //std::cerr<<"nodex: "<<average_x[i_cluster]<<std::endl;
    //std::cerr<<"nodey: "<<average_y[i_cluster]<<std::endl;
    //std::cerr<<"nodez: "<<average_z[i_cluster]<<std::endl;
    //std::cerr<<""<<std::endl;
    pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
    reg.setSearchMethod (tree2);
    reg.setMinClusterSize(50);
    reg.setMaxClusterSize(100000);
    reg.setNumberOfNeighbours (8);
    reg.setInputCloud (colored_cloud_e[i_cluster]);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (1.0);
    reg.extract (cluster_r[i_cluster]);
  }
  //comupte directions
  std::vector<float> directions;
  PN point_normal[cluster_e.size()];
  for(size_t i_cluster = 0; i_cluster < cluster_e.size(); i_cluster++)
  {
    counter = 0;
    for(size_t i_cluster_r = 0; i_cluster_r < cluster_r[i_cluster].size(); i_cluster_r++)
    {

      float dx,dy;
      counter++;
      dx = average_x[i_cluster] - colored_cloud_e[i_cluster]->points[cluster_r[i_cluster][i_cluster_r].indices[50]].x;
      dy = average_y[i_cluster] - colored_cloud_e[i_cluster]->points[cluster_r[i_cluster][i_cluster_r].indices[50]].y;
      directions.push_back(dx/sqrt(pow(dx,2)+pow(dy,2)));
      directions.push_back(dy/sqrt(pow(dx,2)+pow(dy,2)));
      //directions.push_back(average_z[i_cluster] - colored_cloud_e[i_cluster]->points[cluster_r[i_cluster][0].indices[50]].z);
      //std::cerr<<"directions_x: "<<directions[2*counter-2]<<std::endl;
      //std::cerr<<"directions_y: "<<directions[2*counter-1]<<std::endl;
      //std::cerr<<"directions_z: "<<directions[3*counter-1]<<std::endl;
      //std::cerr<<""<<std::endl;


    }
    if(isnan(average_x[i_cluster])  && isnan(average_y[i_cluster]) && isnan(average_z[i_cluster]))
    {
      std::cerr<<"There is no node in cluster: "<<i_cluster<<std::endl;
      continue;
    }
    else
    {
      point_normal[i_cluster].point.x = average_x[i_cluster];
      point_normal[i_cluster].point.y = average_y[i_cluster];
      point_normal[i_cluster].point.z = average_z[i_cluster];
      point_normal[i_cluster].normal1[0] = directions[4*(i_cluster+1)-4];
      point_normal[i_cluster].normal1[1] = directions[4*(i_cluster+1)-3];
      point_normal[i_cluster].normal1[2] = 0;
      point_normal[i_cluster].normal2[0] = directions[4*(i_cluster+2)-2];
      point_normal[i_cluster].normal2[1] = directions[4*(i_cluster)-1];
      point_normal[i_cluster].normal2[2] = 0;
      std::cerr<<"point_normal ["<<i_cluster<<"] point_x: "<<point_normal[i_cluster].point.x<<std::endl;
      std::cerr<<"point_normal ["<<i_cluster<<"] point_y: "<<point_normal[i_cluster].point.y<<std::endl;
      std::cerr<<"point_normal ["<<i_cluster<<"] point_z: "<<point_normal[i_cluster].point.z<<std::endl;
      std::cerr<<"point_normal ["<<i_cluster<<"] normal1_x:"<<point_normal[i_cluster].normal1[0]<<std::endl;
      std::cerr<<"point_normal ["<<i_cluster<<"] normal1_y:"<<point_normal[i_cluster].normal1[1]<<std::endl;
      std::cerr<<"point_normal ["<<i_cluster<<"] normal1_z:"<<point_normal[i_cluster].normal1[2]<<std::endl;
      std::cerr<<"point_normal ["<<i_cluster<<"] normal2_x:"<<point_normal[i_cluster].normal2[0]<<std::endl;
      std::cerr<<"point_normal ["<<i_cluster<<"] normal2_y:"<<point_normal[i_cluster].normal2[1]<<std::endl;
      std::cerr<<"point_normal ["<<i_cluster<<"] normal2_z:"<<point_normal[i_cluster].normal2[2]<<std::endl;
      std::cerr<<""<<std::endl;
    }
  }

  //for(size_t i_cluster_e = 0; i_cluster_e < cluster_e.size(); i_cluster_e++)
  //{
    //std::cerr<<"number"<<cluster_r[i_cluster_e].size()<<std::endl;
  //}
  //node recognition

  //for(size_t i_cluster_e = 0; i_cluster_e < cluster_e.size(); i_cluster_e++)
  //{
    //for(size_t i_cluster_r = 0; i_cluster_r < cluster_r[i_cluster_e].size(); i_cluster_r++)
    //{
      //for(size_t i_point = 0; i_point <cluster_r[i_cluster_e][i_cluster_r].indices.size();i_point++)
      //{
        //pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne2;
        //ne2.setInputCloud (colored_cloud_e[i_cluster_e]);
        //ne2.setSearchMethod (tree2);
        //ne2.setRadiusSearch (0.5) ;
        //ne2.compute (*normals);
        //cloud_f->points[cluster_e[i_cluster].indices[i_point]].x
        //if(normals->points[cluster_r[i_cluster_e][i_cluster_r].indices[i_point]].curvature > 0.5)
        //{
          //counter++;
          //sum_x = sum_x + colored_cloud_e[i_cluster_e]->points[cluster_r[i_cluster_e][i_cluster_r].indices[i_point]].x;
          //sum_y = sum_y + colored_cloud_e[i_cluster_e]->points[cluster_r[i_cluster_e][i_cluster_r].indices[i_point]].y;
          //sum_z = sum_z + colored_cloud_e[i_cluster_e]->points[cluster_r[i_cluster_e][i_cluster_r].indices[i_point]].z;
          //average_x = sum_x/counter;
          //average_y = sum_y/counter;
          //average_z = sum_z/counter;
        //}

          //pcl::PointXYZRGB crosspoint;
      //}
      //std::cerr<<"nodex: "<<average_x<<std::endl;
      //std::cerr<<"nodey: "<<average_y<<std::endl;
      //std::cerr<<"nodez: "<<average_z<<std::endl;
      //std::cerr<<""<<std::endl;
    //}

  //}
  //ros::spin();
  return 0;
}
