/*
* PCL Example using ROS and CPP
*/

// Include the ROS library
#include <ros/ros.h>

// Include pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

// Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>

/********************************************** PRUEBA1 ********************************************************************/
#include <pcl/features/normal_3d.h>
/********************************************** FIN PRUEBA1 ****************************************************************/

/********************************************** PRUEBA2 ********************************************************************/
#include <pcl/visualization/pcl_visualizer.h>

/********************************************** FIN PRUEBA2 ****************************************************************/

// Topics
static const std::string IMAGE_TOPIC = "/velodyne_points";
static const std::string PUBLISH_TOPIC = "/pcl/points";

// ROS Publisher
ros::Publisher pub;

/********************************************** PRUEBA2 ********************************************************************/
pcl::visualization::PCLVisualizer::Ptr normalsVis (
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");  // Edita la forma de ver los puntos
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 0.05, "normals");  //Muestra las lineas
  viewer->addCoordinateSystem (1.0); //Muestra los ejes
  viewer->initCameraParameters ();
  return (viewer);
}
/********************************************** FIN PRUEBA2 ****************************************************************/

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	// Container for original & filtered data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	// Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	// Perform the actual filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize (0.1, 0.1, 0.1);
	sor.filter (cloud_filtered);



	/********************************************** PRUEBA1 ********************************************************************/
	//Prueba Superficies Normales
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nueva (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2 (cloud_filtered, *cloud_nueva);

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud_nueva);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.5);

	// Compute the features
	ne.compute (*cloud_normals);
	/********************************************** FIN PRUEBA1 ****************************************************************/


	/********************************************** PRUEBA2 ********************************************************************/
	pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer = normalsVis(cloud_nueva, cloud_normals);
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
	}
	/********************************************** FIN PRUEBA2 ****************************************************************/

	// Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	//pcl_conversions::fromPCL(cloud_filtered, output);   //Conversion antigua

	/********************************************** PRUEBA1 ********************************************************************/
	pcl::toROSMsg(*cloud_normals, output);    //Conversion nueva
	//No da error e imprime el mensaje, pero no lo visualizo en el rviz
	/********************************************** FIN PRUEBA1 ****************************************************************/

	// Publish the data
	pub.publish (output);
}

int main (int argc, char** argv)
{
	// Initialize the ROS Node "roscpp_pcl_example"
	ros::init (argc, argv, "roscpp_pcl_example");
	ros::NodeHandle nh;

	// Print "Hello" message with node name to the terminal and ROS log file
	ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());

	// Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function to cloud_cb
	ros::Subscriber sub = nh.subscribe(IMAGE_TOPIC, 1, cloud_cb);

	// Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
	pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);

	// Spin
	ros::spin();

	// Success
	return 0;
}
