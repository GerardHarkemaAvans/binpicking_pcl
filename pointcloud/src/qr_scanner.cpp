//Standaard Headers
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include <std_msgs/String.h>
#include <iostream>
#include <algorithm>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp>
#include <string>
#include <vector>
//Intel Realsense Headers
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
// PCL Headers
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/filter.h>
#include <pcl/common/common_headers.h>
#include "pcl/common/common.h"
#include <pcl/common/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
//Header voor ROI
#include <pcl/filters/statistical_outlier_removal.h>
//Header voor filter voxel grid
#include <pcl/filters/voxel_grid.h>
//Header voor region growing
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;

typedef pcl::PointXYZRGB RGB_Cloud;
typedef pcl::PointCloud<RGB_Cloud> point_cloud;
typedef point_cloud::Ptr cloud_pointer;
typedef point_cloud::Ptr prevCloud;
typedef pcl::PointXYZI PointT;

// Prototypes
void Load_PCDFile(void);
void Load_PCDFileNormal(string filename);
void Load_PCDFile_ROI(string filename);

// Global Variables
string cloudFile; // .pcd file name
string prevCloudFile; // .pcd file name (Old cloud)
int i = 1; // Index for incremental file name

//waarde voor ROI
#define X_LEFT  -79     // is right (getal hoger = verder naar rechts)    ///// aannpassen
#define X_RIGHT  123 // is left  (getal lager is verder naar rechts)
#define Y_TOP  -73 // is bottom(getal lager is verder naar boven)
#define Y_BOTTOM 175 // is top (getal hoger is naar beneden)
#define Z_TOP  0.591 // alles boven deze waarde wordt weggegooid
#define Z_BOTTOM 0.4  // alles onder deze waarde wordt weggegooid

//======================================================
// RGB Texture
// - Function is utilized to extract the RGB data from
// a single point return R, G, and B values.
// Normals are stored as RGB components and
// correspond to the specific depth (XYZ) coordinate.
// By taking these normals and converting them to
// texture coordinates, the RGB components can be
// "mapped" to each individual point (XYZ).
//======================================================
std::tuple<int, int, int> RGB_Texture(rs2::video_frame texture, rs2::texture_coordinate Texture_XY)
{
 // Get Width and Height coordinates of texture
 int width = texture.get_width();  // Frame width in pixels
 int height = texture.get_height(); // Frame height in pixels
            // Normals to Texture Coordinates conversion
 int x_value = min(max(int(Texture_XY.u * width + .5f), 0), width - 1);
 int y_value = min(max(int(Texture_XY.v * height + .5f), 0), height - 1);
 int bytes = x_value * texture.get_bytes_per_pixel();   // Get # of bytes per pixel
 int strides = y_value * texture.get_stride_in_bytes(); // Get line width in bytes
 int Text_Index = (bytes + strides);
 const auto New_Texture = reinterpret_cast<const uint8_t*>(texture.get_data());
 // RGB components to save in tuple
 int NT1 = New_Texture[Text_Index];
 int NT2 = New_Texture[Text_Index + 1];
 int NT3 = New_Texture[Text_Index + 2];
 return std::tuple<int, int, int>(NT1, NT2, NT3);
}

//===================================================
//  PCL_Conversion
// - Function is utilized to fill a point cloud
//  object with depth and RGB data from a single
//  frame captured using the Realsense.
//===================================================
cloud_pointer PCL_Conversion(const rs2::points& points, const rs2::video_frame& color) {
 // Object Declaration (Point Cloud)
 cloud_pointer cloud(new point_cloud);
 // Declare Tuple for RGB value Storage (<t0>, <t1>, <t2>)
 std::tuple<uint8_t, uint8_t, uint8_t> RGB_Color;
 //================================
 // PCL Cloud Object Configuration
 //================================
 // Convert data captured from Realsense camera to Point Cloud
 auto sp = points.get_profile().as<rs2::video_stream_profile>();
 cloud->width = static_cast<uint32_t>(sp.width());
 cloud->height = static_cast<uint32_t>(sp.height());
 cloud->is_dense = false;
 cloud->points.resize(points.size());
 auto Texture_Coord = points.get_texture_coordinates();
 auto Vertex = points.get_vertices();
 // Iterating through all points and setting XYZ coordinates
 // and RGB values
 for (int i = 0; i < points.size(); i++)
 {
  //===================================
  // Mapping Depth Coordinates
  // - Depth data stored as XYZ values
  //===================================
  cloud->points[i].x = Vertex[i].x;
  cloud->points[i].y = Vertex[i].y;
  cloud->points[i].z = Vertex[i].z;
  // Obtain color texture for specific point
  RGB_Color = RGB_Texture(color, Texture_Coord[i]);
  // Mapping Color (BGR due to Camera Model)
  cloud->points[i].r = get<2>(RGB_Color); // Reference tuple<2>
  cloud->points[i].g = get<1>(RGB_Color); // Reference tuple<1>
  cloud->points[i].b = get<0>(RGB_Color); // Reference tuple<0>
 }
 return cloud; // PCL RGB Point Cloud generated
}

/*================================ Class voor pub en sub ========================================*/

  void StartCallback(const std_msgs::String::ConstPtr& msg);
  ros::Publisher pub;
/*========================================= Main =========================================*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;

  ros::Subscriber sub;
  pub = nh.advertise<std_msgs::Float64MultiArray>("LocationArray", 100);
  sub = nh.subscribe("StartVision", 100, StartCallback);
///std_msgs::String::ConstPtr msg;
///StartCallback(msg);
  ros::spin();
}

/*================================ callback start ========================================*/
// Functie voor het subscriben van de start van de vision applicatie
void StartCallback(const std_msgs::String::ConstPtr& msg)
{
    cout << "TRUE" << endl;

  string test;
  test = "TRUE";
  if (msg->data.c_str() == "TRUE") {
  //if (test == "TRUE") {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    boost::shared_ptr<pcl::visualization::PCLVisualizer> openCloud;
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    //======================
    // Stream configuration
    //======================
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 1280, 720, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    rs2::pipeline_profile selection = pipe.start(cfg);
    rs2::device selected_device = selection.get_device();
    auto depth_sensor = selected_device.first<rs2::depth_sensor>();
    // Begin Stream with default configs
     /*===================================================================== Opvragen beeld realsense en omzetten naar pcd file ======================================================================================*/
       // Wait for frames from the camera to settle
     for (int i = 0; i < 30; i++) {
      auto frames = pipe.wait_for_frames(); //Drop several frames for auto-exposure
     }
     // Capture a single frame and obtain depth + RGB values from it
     auto frames = pipe.wait_for_frames();
     auto depth = frames.get_depth_frame();
     auto RGB = frames.get_color_frame();
     // Map Color texture to each point
     pc.map_to(RGB);
     // Generate Point Cloud
     auto points = pc.calculate(depth);
     // Convert generated Point Cloud to PCL Formatting
     cloud_pointer cloud = PCL_Conversion(points, RGB);

     // Filter PointCloud (PassThrough Method)
     pcl::PassThrough<pcl::PointXYZRGB> Cloud_Filter; // Create the filtering object
     Cloud_Filter.setInputCloud(cloud);     // Input generated cloud to filter
     Cloud_Filter.setFilterFieldName("z");    // Set field name to Z-coordinate
     Cloud_Filter.setFilterLimits(0.0, 1.0);    // Set accepted interval values
     Cloud_Filter.filter(*newCloud);      // Filtered Cloud Outputted
     cloudFile = "Captured_Frame" + to_string(i) + ".pcd";

     // Take Cloud Data and write to .PCD File Format
     pcl::io::savePCDFileASCII(cloudFile, *cloud); // Input cloud to be saved to .pcd
     /*===================================================================== Downsampeling filter ======================================================================================*/
     // load file en sla ROI op
     Load_PCDFile_ROI(cloudFile);
     // filter toepassen op ROI om aantal punten te verminderen
     pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2());
     pcl::PCLPointCloud2::Ptr cloud_ROI2(new pcl::PCLPointCloud2());
     pcl::PCDReader reader;
     reader.read("ROI_PointCloud.pcd", *cloud_ROI2);
     // Create the filtering object
     pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
     sor.setInputCloud(cloud_ROI2);
     // leaf size van 0.01 betekend 1 cm.
     sor.setLeafSize(0.0005f, 0.0005f, 0.005f);
     sor.filter(*cloud_filtered);
     pcl::PCDWriter writer;
     writer.write("ROI_Filtered.pcd", *cloud_filtered,
     Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);
     /*===================================================================== Region Growing Algrotime ======================================================================================*/
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRG(new pcl::PointCloud<pcl::PointXYZ>);
     if (pcl::io::loadPCDFile <pcl::PointXYZ>("ROI_Filtered.pcd", *cloudRG) == -1)
     {
      std::cout << "Cloud reading failed." << std::endl;
     }
     pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
     pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
     normal_estimator.setSearchMethod(tree);
     normal_estimator.setInputCloud(cloudRG);
     normal_estimator.setKSearch(50);
     normal_estimator.compute(*normals);
     pcl::IndicesPtr indices(new std::vector <int>);
     pcl::PassThrough<pcl::PointXYZ> pass;
     pass.setInputCloud(cloudRG);
     pass.setFilterFieldName("z");
     pass.setFilterLimits(0.0, 1.0);
     pass.filter(*indices);
     pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
     reg.setMinClusterSize(500);
     reg.setMaxClusterSize(100000);
     reg.setSearchMethod(tree);
     reg.setNumberOfNeighbours(50);
     reg.setInputCloud(cloudRG);
     //reg.setIndices (indices);
     reg.setInputNormals(normals);
     reg.setSmoothnessThreshold(7.5 / 180.0 * M_PI);
     reg.setCurvatureThreshold(3.0);
     std::vector <pcl::PointIndices> clusters;
     reg.extract(clusters);
     pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
     /*===================================================================== clusters omzetten naar pcd file ======================================================================================*/
     pcl::PCDReader reader3;
     pcl::PCDWriter writer3;
     int j = 0;
     for (std::vector<pcl::PointIndices>::const_iterator it = clusters.begin(); it != clusters.end(); ++it)
     {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
       cloud_cluster->points.push_back(cloudRG->points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      std::stringstream ss;
      ss << "cloud_cluster_" << j << ".pcd";
      writer3.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
      j++;
     }
     /*===================================================================== zwaartepunt bepalen ======================================================================================*/
        //cluster 0 inlezen
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_1(new pcl::PointCloud<pcl::PointXYZ>);
     pcl::PCDReader readerCluster;
     readerCluster.read("cloud_cluster_0.pcd", *cloud_cluster_1);
     Eigen::Vector4f centroid;
     pcl::compute3DCentroid(*cloud_cluster_1, centroid);
     //bepalen van Z (hoogste z van de cloud cluster)
     pcl::PointXYZ minPt, maxPt;
     pcl::getMinMax3D(*cloud_cluster_1, minPt, maxPt);
     pcl::PointXYZ CenterPoint;
     CenterPoint.x = centroid[0];
     CenterPoint.y = centroid[1];
     CenterPoint.z = minPt.z;
     cout << "Het middelpunt (x, y, z) van dit object is: " << CenterPoint << endl;
     i++; // Increment File Name

     //Publish waarden positie
     double x, y, z;
     x = CenterPoint.x;
     y = CenterPoint.y;
     z = CenterPoint.z;

     std_msgs::Float64MultiArray Position;
     Position.data.push_back(x);
     Position.data.push_back(y);
     Position.data.push_back(z);

     pub.publish(Position);

    }//einde if true


}// einde callback functie


/*=================================== Functies ==========================================*/
void Load_PCDFile(void)
{
 string openFileName;
 // Generate object to store cloud in .pcd file
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudView(new pcl::PointCloud<pcl::PointXYZRGB>);
 openFileName = "Captured_Frame" + to_string(i) + ".pcd";
 pcl::io::loadPCDFile(openFileName, *cloudView); // Load .pcd File
             //==========================
             // Pointcloud Visualization
             //==========================
             // Create viewer object titled "Captured Frame"
 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Captured Frame"));
 // Set background of viewer to black
 viewer->setBackgroundColor(0, 0, 0);
 // Add generated point cloud and identify with string "Cloud"
 viewer->addPointCloud<pcl::PointXYZRGB>(cloudView, "Cloud");
 // Default size for rendered points
 viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud");
 // Viewer Properties
 viewer->initCameraParameters();  // Camera Parameters for ease of viewing
 cout << endl;
 cout << "Press [Q] in viewer to continue. " << endl;
 viewer->spin(); // Allow user to rotate point cloud and view it
     // Note: No method to close PC visualizer, pressing Q to continue software flow only solution.

}

void Load_PCDFile_ROI(string filename)
{
 string openFileName = filename;
 // Generate object to store cloud in .pcd file
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered_x(new pcl::PointCloud<pcl::PointXYZRGB>);
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered_xy(new pcl::PointCloud<pcl::PointXYZRGB>);
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered_z(new pcl::PointCloud<pcl::PointXYZRGB>);
 pcl::io::loadPCDFile(openFileName, *cloud); // Load .pcd File

   // Create the filtering object for x direction
 pcl::PassThrough<pcl::PointXYZRGB> pass_x;
 pass_x.setInputCloud(cloud);
 pass_x.setFilterFieldName("x");
 pass_x.setFilterLimits(X_LEFT / 1000.0, X_RIGHT / 1000.0);
 // pass_x.setFilterLimitsNegative (true);
 pass_x.filter(*cloudFiltered_x);

 // Create the filtering object for y direction
 pcl::PassThrough<pcl::PointXYZRGB> pass_y;
 pass_y.setInputCloud(cloudFiltered_x);
 pass_y.setFilterFieldName("y");
 pass_y.setFilterLimits(Y_TOP / 1000.0, Y_BOTTOM / 1000.0);
 // pass_x.setFilterLimitsNegative (true);
 pass_y.filter(*cloudFiltered_xy);
 // Create the filtering object for z direction
 pcl::PassThrough<pcl::PointXYZRGB> pass_z;
 pass_y.setInputCloud(cloudFiltered_xy);
 pass_y.setFilterFieldName("z");
 pass_y.setFilterLimits(Z_BOTTOM, Z_TOP);
 pass_x.setFilterLimitsNegative(true);
 pass_y.filter(*cloudFiltered_z);
 pcl::io::savePCDFileASCII("ROI_PointCloud.pcd", *cloudFiltered_z); // ROI point cloud saved to .pcd
}

void Load_PCDFileNormal(string filename)
{
 string openFileName = filename;
 std::cout << "Reading " << openFileName << std::endl;
 // Generate object to store cloud in .pcd file
 pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
 pcl::io::loadPCDFile(openFileName, *cloud); // Load .pcd File
 //==========================
 // Pointcloud Visualization
 //==========================
 // Create viewer object titled "Captured Frame"
 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer(openFileName));
 // Set background of viewer to black
 viewer->setBackgroundColor(0, 0, 0);
 // Add generated point cloud and identify with string "Cloud"
 viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "Cloud RIO");
 // Default size for rendered points
 viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Cloud RIO");
 // Viewer Properties
 viewer->initCameraParameters();  // Camera Parameters for ease of viewing
 cout << endl;
 cout << "Press [Q] in viewer to continue. " << endl;
 viewer->spin(); // Allow user to rotate point cloud and view it
     // Note: No method to close PC visualizer, pressing Q to continue software flow only solution.

}















