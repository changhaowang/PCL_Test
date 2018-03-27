#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/assign/list_of.hpp>
#include <iostream>
#include <vector>
#include <sstream>
#include <boost/bind.hpp>
//#include "scene_recognition/config.h"


//struct LocalConfig :Config
//{
//  static int DistanceThreshold;
//  static float p_inliers;
//  LocalConfig() :
//  Config(){
//    params.push_back(new Parameter<int> ("DistanceThreshold", &DistanceThreshold, ""));
//    params.push_back(new Parameter<float> ("p_inliers", &p_inliers, ""));
//  }
//};
//int LocalConfig::DistanceThreshold = 1;
//float LocalConfig::p_inliers = 0.3;
//void chatterCallback(const std_msgs::String::ConstPtr& msg)
//{
//  ROS_INFO("I heard: [%s]", msg->data.c_str());
//}
float DistanceThreshold = 0.5;
float p_inliers = 0.3;

ros::Publisher pcl_pub;

void cloudCallback(sensor_msgs::PointCloud2ConstPtr input,ros::NodeHandle& nh)
{
  //ros::Publisher seg_pub;
  ros::Publisher seg_pub = nh.advertise<sensor_msgs::PointCloud2> ("seg_output", 1);
  sensor_msgs::PointCloud2 output;


  //pcl::visualization::CloudViewer viewer("pcd viewer");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg (*input, *cloud);
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  //std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  //seg.setMaxIterations (100);
  //seg.setDistanceThreshold (LocalConfig::DistanceThreshold);
  seg.setDistanceThreshold (DistanceThreshold);
  //std::cerr << "LocalConfig::DistanceThreshold:"<<LocalConfig::DistanceThreshold<<std::endl;
  //segmentation iterativly
  int i=0, nr_points = (int) cloud_filtered->points.size ();
  while (cloud_filtered->points.size () > p_inliers * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    std::cerr << "inliers:"<<inliers->indices.size()<<std::endl;
    std::cerr << "totalpoints"<<nr_points<<std::endl;
    std::cerr << "Model coefficients: " <<i<<" "<< coefficients->values[0] << " "
                                        << coefficients->values[1] << " "
                                        << coefficients->values[2] << " "
                                        << coefficients->values[3] << std::endl;
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
    i++;
    //viewer.showCloud(cloud_filtered);
    //system("pause");
    pcl::toROSMsg(*cloud_filtered, output);
    output.header.frame_id = "odom";
    seg_pub.publish(output);
  }
  //seg.setInputCloud (cloud.makeShared ());   //设置输入的点云
  //seg.segment (inliers, coefficients);       //cloud.makeShared() 创建一个 boost shared_ptr
  //pcl_msgs::ModelCoefficients ros_coefficients;
  //pcl_conversions::fromPCL(coefficients, ros_coefficients);
  //pub.publish (ros_coefficients);
  std::cout << "Finished" <<std::endl;
}

int main(int argc, char **argv)
{
//  Parser parser;
//  parser.addGroup(LocalConfig());
//  parser.addGroup(GeneralConfig());
//  parser.read(argc, argv);


  ros::init(argc, argv, "recognition");

  ros::NodeHandle nh;

  std::vector<ros::Subscriber> sub = nh.subscribe("ring_buffer/cloud2", 1, boost::bind(cloudCallback, _1, nh));
  //ros::Subscriber sub;
  //sub.push_back(nh)
  ros::spin();

  return 0;
}
