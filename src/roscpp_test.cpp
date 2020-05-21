/**************************LIBRARIES**************************/
// Include the ROS library
#include <ros/ros.h>

// Include pcl types and conversions
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h> //toPCL

// Inlcude pcl filters
#include <pcl/common/common.h> //getMinMax3D
#include <pcl/filters/voxel_grid.h> //VoxelGrid
#include <pcl/filters/passthrough.h> //PassThrough

// Include pcl for keypoints
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/harris_3d.h>

// Include pcl features
#include <pcl/features/normal_3d.h>

// Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>

/**************************DECLARATIONS***********************/
// Topics
static const std::string IMAGE_TOPIC = "/velodyne_points";
static const std::string PUBLISH_TOPIC = "/pcl/points";
static const std::string PUBLISH_TOPIC_2 = "/pcl/points2";

// ROS Publisher
ros::Publisher pubF;
ros::Publisher pubKP;
// ROS Subscriber
ros::Subscriber sub;


/*****************************KEYPOINTS FUNCTIONS************************************************/
// Incluir al usar SIFTKeyPointsFieldSelector para que seleccione segun la Z.
/*namespace pcl
{
  template<>
    struct SIFTKeypointFieldSelector<PointXYZI>
    {
      inline float
      operator () (const PointXYZI &p) const
      {
		return p.z;
      }
    };
}
*/
pcl::PointCloud<pcl::PointXYZI>::Ptr KeyPointsSiftZ(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{  
	// Parameters for sift computation
	const float min_scale = 0.05f;
	const int n_octaves = 10;
	const int n_scales_per_octave = 14;
	const float min_contrast = 0.05f;

	// Estimate the sift interest points using z values from xyz as the Intensity variants
	pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale> result;
	pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI> ());
	sift.setSearchMethod(kdtree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(cloud);
	sift.compute(result);

	std::cout << "No of SIFT Z Keypoints in the result are " << result.points.size () << std::endl;

	// Copying the pointwithscale to pointxyz so as visualize the cloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZI>);
	copyPointCloud(result, *cloud_temp);

	return cloud_temp;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr KeyPointsSiftNE(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	// Parameters for sift computation
	const float min_scale = 0.01f;
	const int n_octaves = 3;
	const int n_scales_per_octave = 4;
	const float min_contrast = 0.01f;

	// Estimate the normals of the cloud_xyz
	pcl::NormalEstimation<pcl::PointXYZI, pcl::PointNormal> ne;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>());
	ne.setInputCloud(cloud);
	ne.setSearchMethod(kdtree);
	ne.setRadiusSearch(0.2);
	ne.compute(*cloud_normals);


	// Copy the xyz info from cloud_xyz and add it to cloud_normals as the xyz field in PointNormals estimation is zero
	for(std::size_t i = 0; i<cloud_normals->points.size(); ++i)
	{
		cloud_normals->points[i].x = cloud->points[i].x;
		cloud_normals->points[i].y = cloud->points[i].y;
		cloud_normals->points[i].z = cloud->points[i].z;
	}

	// Estimate the sift interest points using normals values from xyz as the Intensity variants
	pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale> result;
	pcl::search::KdTree<pcl::PointNormal>::Ptr kdtree_n(new pcl::search::KdTree<pcl::PointNormal> ());
	sift.setSearchMethod(kdtree_n);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(cloud_normals);
	sift.compute(result);

	std::cout << "No of SIFT NE Keypoints in the result are " << result.points.size () << std::endl;

	// Copying the pointwithscale to pointxyz so as visualize the cloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZI>);
	copyPointCloud(result, *cloud_temp);

	return cloud_temp;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr KeyPointsISS(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{

    pcl::ISSKeypoint3D<pcl::PointXYZI, pcl::PointXYZI> iss_detector;
    pcl::PointCloud<pcl::PointXYZI>::Ptr result (new pcl::PointCloud<pcl::PointXYZI>());
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZI>());

    iss_detector.setSearchMethod(kdtree);
    iss_detector.setSalientRadius(10 * 0.05);
    iss_detector.setNonMaxRadius(8 * 0.05);
    iss_detector.setThreshold21(0.2);
    iss_detector.setThreshold32(0.2);
    iss_detector.setMinNeighbors(10);
    iss_detector.setNumberOfThreads(10);
    iss_detector.setInputCloud(cloud);
    iss_detector.compute(*result);

	std::cout << "No of ISS Keypoints in the result are " << result->points.size () << std::endl;

	// Copying the pointwithscale to pointxyz so as visualize the cloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZI>);
	copyPointCloud(*result, *cloud_temp);

	return cloud_temp;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr KeyPointsHarris(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	pcl::HarrisKeypoint3D <pcl::PointXYZI, pcl::PointXYZI> detector;
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZI>);

	detector.setNonMaxSupression (true);
	detector.setInputCloud (cloud);
	detector.setThreshold (1e-11);
	detector.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZI,pcl::PointXYZI>::HARRIS); 
	detector.setRefine(false);
	detector.setRadius(0.5);
	detector.compute (*keypoints);

	std::cout << "No of Harris Keypoints in the result are " << keypoints->points.size () << std::endl;

	// Copying the pointwithscale to pointxyz so as visualize the cloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZI>);
	copyPointCloud(*keypoints, *cloud_temp);

	return cloud_temp;
}


/**************************MSGS ANSWER*************************/
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	// Container for original & filtered data
	pcl::PCLPointCloud2* cloud_PCL2_Ptr = new pcl::PCLPointCloud2; 		//Cloud pointer input filter
	pcl::PCLPointCloud2ConstPtr cloud_PCL2_ConstPtr(cloud_PCL2_Ptr); 	//Filter Input (Pointer to Cloud pointer input filter)
	pcl::PCLPointCloud2 cloud_f_PCL2;									//VoxelGrid Filter Output

	// Convert MSGS to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud_PCL2_Ptr);
	// Convert to PointCloud (PointXYZI) data type
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_XYZI (new pcl::PointCloud<pcl::PointXYZI>);		//Cloud XYZI original
	pcl::fromPCLPointCloud2 (*cloud_PCL2_ConstPtr, *cloud_XYZI);

	// VoxedGrid Filter
	pcl::VoxelGrid<pcl::PCLPointCloud2> FilterVoxelGrid;
	FilterVoxelGrid.setInputCloud (cloud_PCL2_ConstPtr);
	FilterVoxelGrid.setLeafSize (1.0,1.0,1.0);
	FilterVoxelGrid.filter (cloud_f_PCL2);

	// Paso la cloud filtrada a tipo PointCloud (PointXYZI)
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f_XYZI (new pcl::PointCloud<pcl::PointXYZI>);	//VoxelGrid Filter Output in Cloud XYZI
	pcl::fromPCLPointCloud2 (cloud_f_PCL2, *cloud_f_XYZI);

	// Prepare data clouds for the next filter in cascade
	pcl::PCLPointCloud2::Ptr cloud_f2_PCL2 (new pcl::PCLPointCloud2 ());	//PassThrough Filter Output in X
	*cloud_PCL2_Ptr = cloud_f_PCL2;											//Copy VoxelGrid Output Filter to Filter Input

	// PassThrough Filter in X
	pcl::PassThrough<pcl::PCLPointCloud2> pass;
	pass.setInputCloud(cloud_PCL2_ConstPtr);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-10, 10);
	pass.filter(*cloud_f2_PCL2);

	// Prepare data clouds for the next filter in cascade
	pcl::PCLPointCloud2::Ptr cloud_f3_PCL2 (new pcl::PCLPointCloud2 ());	//Output PassThrough Filter in Y
	*cloud_PCL2_Ptr = *cloud_f2_PCL2;										//Copy PassThroughX Filter Output to Filter Input

	// PassThrough Filter in Y
	pass.setInputCloud(cloud_PCL2_ConstPtr);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-10, 16);
	pass.filter(*cloud_f3_PCL2);

	// Paso la cloud filtrada 2 a tipo PointCloud (PointXYZI)
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_f3_XYZI (new pcl::PointCloud<pcl::PointXYZI>);	//Cascade Filter Output in Cloud XYZI
	pcl::fromPCLPointCloud2 (*cloud_f3_PCL2, *cloud_f3_XYZI);

	// Publish the data filtered in pubF
   	sensor_msgs::PointCloud2 outputF;
   	pcl::toROSMsg(*cloud_f3_XYZI, outputF); // Si la salida es tipo plc::PintCloud
	outputF.header.frame_id = "/velodyne";
	pubF.publish (outputF);

	// Obtengo KeyPoints segun SIFT Z
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_KPSiftZ_XYZI (new pcl::PointCloud<pcl::PointXYZI>);
	cloud_KPSiftZ_XYZI=KeyPointsSiftZ(cloud_f3_XYZI);

	// Obtengo KeyPoints segun SIFT NE
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_KPSiftNE_XYZI (new pcl::PointCloud<pcl::PointXYZI>);
	cloud_KPSiftNE_XYZI=KeyPointsSiftNE(cloud_f3_XYZI);

	// Obtengo KeyPoints segun ISS
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_KPISS_XYZI (new pcl::PointCloud<pcl::PointXYZI>);
	cloud_KPISS_XYZI=KeyPointsISS(cloud_f3_XYZI);

	// Obtengo KeyPoints segun HARRIS
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_KPHARRIS_XYZI (new pcl::PointCloud<pcl::PointXYZI>);
	cloud_KPHARRIS_XYZI=KeyPointsHarris(cloud_f3_XYZI);

	// Prubish the data KP
   	sensor_msgs::PointCloud2 outputKP;
   	pcl::toROSMsg(*cloud_KPSiftZ_XYZI, outputKP); // Si la salida es tipo plc::PintCloud
	outputKP.header.frame_id = "/velodyne";
	pubKP.publish (outputKP);
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
	pubF = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);
	pubKP = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC_2, 1);

	// Spin
	ros::spin();

	// Success
	return 0;
}