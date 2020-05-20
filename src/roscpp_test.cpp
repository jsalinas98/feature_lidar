/**************************LIBRARIES**************************/
// Include the ROS library
#include <ros/ros.h>

// Include pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> //toPCL
#include <pcl/common/common.h> //getMinMax3D
#include <pcl/filters/voxel_grid.h> //VoxelGrid
//#include <pcl/filters/passthrough.h> //PassThrough

// Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>

/**************************DECLARATIONS***********************/
// Topics
static const std::string IMAGE_TOPIC = "/velodyne_points";
static const std::string PUBLISH_TOPIC = "/pcl/points";

// ROS Publisher
ros::Publisher pub;
// ROS Subscriber
ros::Subscriber sub;

/**************************MSGS ANSWER*************************/
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	// Container for original & filtered data
	pcl::PCLPointCloud2* cloud_PCL2 = new pcl::PCLPointCloud2; //Tipo de dato para que lea del mensaje
	pcl::PCLPointCloud2ConstPtr cloud_PLC2_Ptr(cloud_PCL2); //Tipo de dato que admite el filtro
	pcl::PCLPointCloud2 cloud_f_PCL2;

	// Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud_PCL2);


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_XYZI (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2 (*cloud_PLC2_Ptr, *cloud_XYZI);

	pcl::PointXYZ minPt, maxPt;
  	pcl::getMinMax3D (*cloud_XYZI, minPt, maxPt);
	std::cout << "Min z: " << minPt.z+1 << std::endl;
	std::cout << "Max z: " << maxPt.z << std::endl;

	// Perform the actual filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloud_PLC2_Ptr);
	sor.setLeafSize (0.1, 0.1, 0.1);
	sor.filter (cloud_f_PCL2);

	// Paso la cloud filtrada 1 a tipo PointCloud...PointXYZI
	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f_XYZI (new pcl::PointCloud<pcl::PointXYZI>);
	//pcl::fromPCLPointCloud2 (cloud_f_PCL2, *cloud_f_XYZI);

/*
	// cascade the floor removal filter and define a container for floorRemoved	
	pcl::PCLPointCloud2::Ptr cloud_f2_PCL2 (new pcl::PCLPointCloud2 ());
	pcl::PCLPointCloud2ConstPtr cloud_f2_Ptr (&cloud_f_PCL2);
		
	// define a PassThrough filter
	pcl::PassThrough<pcl::PCLPointCloud2> pass;
	pass.setInputCloud(cloud_f2_Ptr);
	// filter along z-axis
	pass.setFilterFieldName("z");
	// set z-limits
	pass.setFilterLimits(minPt.z+2, maxPt.z);
	pass.filter(*cloud_f2_PCL2);

	// Paso la cloud filtrada 2 a tipo PointCloud...PointXYZI
	//pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f2_XYZI (new pcl::PointCloud<pcl::PointXYZI>);
	//pcl::fromPCLPointCloud2 (*cloud_f2_PCL2, *cloud_f2_XYZI);
*/

   	sensor_msgs::PointCloud2 output;
   	//pcl_conversions::fromPCL(cloud_PC, output); // Si la salida es tipo plc::PLCPointCloud2
   	//pcl::toROSMsg(*cloud_XYZI, output); // Si la salida es tipo plc::PintCloud
	output.header.frame_id = "/velodyne";

	// Publish the data
	pub.publish (output);
}

/******************************MAIN*****************************/
int main (int argc, char** argv)
{
	// Initialize the ROS Node
	ros::init (argc, argv, "roscpp_pcl_example");
	ros::NodeHandle nh;

	// Print "Hello" message with node name to the terminal and ROS log file
	ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());

	// Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function to cloud_cb
	sub = nh.subscribe(IMAGE_TOPIC, 1, cloud_cb);

	// Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
	pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);

	// Spin
	ros::spin();

	// Success
	return 0;
}