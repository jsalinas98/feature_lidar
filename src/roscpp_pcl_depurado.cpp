/**************************LIBRARIES**************************/

// Include the ROS library
#include <ros/ros.h>

// Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>

// Include pcl types
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Include pcl filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

//Include features
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/vfh.h>

//Include Visualizer and Plotter
//#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/visualization/histogram_visualizer.h>
//#include <pcl/visualization/pcl_plotter.h>

//Include keypoints
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>

/**************************DECLARATIONS***********************/
// Topics
static const std::string IMAGE_TOPIC = "/velodyne_points";
static const std::string PUBLISH_TOPIC = "/pcl/points";
static const std::string PUBLISH_TOPIC2 = "/pcl/points2";

//Variables globales
pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_anterior (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptorPFH_anterior(new pcl::PointCloud<pcl::PFHSignature125>());
pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptorFPHF_anterior(new pcl::PointCloud<pcl::FPFHSignature33>());
pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptorVFH_anterior(new pcl::PointCloud<pcl::VFHSignature308>);


// ROS Publisher
ros::Publisher pubF;
ros::Publisher pubKP;

// ROS Subscriber
ros::Subscriber sub;

pcl::PointCloud<pcl::Normal>::Ptr Normals(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.03);

	// Compute the features
	ne.compute (*cloud_normals);

	return cloud_normals;
}

pcl::PointCloud<pcl::PFHSignature125>::Ptr PFH(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
	// PFH estimation object.
	pcl::PFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud (cloud);
	pfh.setInputNormals (cloud_normals);
	
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
	pfh.setSearchMethod (tree);

	// Object for storing the PFH descriptors for each point.
	pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptor(new pcl::PointCloud<pcl::PFHSignature125>());

	// Use all neighbors in a sphere of radius 5cm (radius goes in meters)
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	pfh.setRadiusSearch (0.5);

	// Compute the features
	pfh.compute (*descriptor);
/*
	// Plotter object.
	pcl::visualization::PCLHistogramVisualizer Hviewer;
	// We need to set the size of the descriptor beforehand.
	Hviewer.addFeatureHistogram(*descriptor, 125);

	Hviewer.spin();
*/
	return descriptor;
}


pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFH(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
	// FPFH estimation object.
	pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud (cloud);
	fpfh.setInputNormals (cloud_normals);
	
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI> ());
	fpfh.setSearchMethod (tree);

	// Object for storing the FPFH descriptors for each point.
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptor(new pcl::PointCloud<pcl::FPFHSignature33>());
	
	// Use all neighbors in a sphere of radius 5cm (radius goes in meters)
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	fpfh.setRadiusSearch (0.5);

	// Compute the features
	fpfh.compute (*descriptor);

/*
	// Plotter object.
	pcl::visualization::PCLPlotter plotter;
	// We need to set the size of the descriptor beforehand.
	plotter.addFeatureHistogram(*descriptor, 33);

	plotter.plot();
*/
	return descriptor;
}

pcl::PointCloud<pcl::VFHSignature308>::Ptr VFH(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
	// Object for storing the VFH descriptor.
	pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);

	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);

	// VFH estimation object.
	pcl::VFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud(cloud);
	vfh.setInputNormals(cloud_normals);
	vfh.setSearchMethod(tree);
	// Optionally, we can normalize the bins of the resulting histogram,
	// using the total number of points.
	vfh.setNormalizeBins(true);
	// Also, we can normalize the SDC with the maximum size found between
	// the centroid and any of the cluster's points.
	vfh.setNormalizeDistance(false);

	vfh.compute(*descriptor);
/*
	// Plotter object.
	pcl::visualization::PCLPlotter plotter;
	// We need to set the size of the descriptor beforehand.
	plotter.addFeatureHistogram(*descriptor, 308);

	plotter.plot();
*/
	return descriptor;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr KeyPointsHarris(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZI>);

	pcl::HarrisKeypoint3D <pcl::PointXYZI, pcl::PointXYZI> detector;
	detector.setNonMaxSupression (true);
	detector.setInputCloud (cloud);
	detector.setThreshold (1e-11);
	detector.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZI,pcl::PointXYZI>::HARRIS); 
	detector.setRefine(false);
	detector.setRadius(0.5); 
	detector.compute (*keypoints);

	std::cout << "No of Harris Keypoints in the result are " << keypoints->points.size () << std::endl;

	return keypoints;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr KeyPointsSiftNE(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	// Parameters for sift computation
	const float min_scale = 0.01f;
	const int n_octaves = 3;
	const int n_scales_per_octave = 4;
	const float min_contrast = 0.001f;

	// Estimate the normals of the cloud_xyz
	pcl::NormalEstimation<pcl::PointXYZI, pcl::PointNormal> ne;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());

	ne.setInputCloud(cloud);
	ne.setSearchMethod(tree);
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
	pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointXYZI> sift;
	pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal> ());
	sift.setSearchMethod(tree2);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(cloud_normals);
	sift.compute(*result);

	std::cout << "No of SIFT NE Keypoints in the result are " << result->points.size () << std::endl;

	return result;
}

// Incluir al usar SIFTKeyPointsFieldSelector para que seleccione segun la Z.
namespace pcl
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

pcl::PointCloud<pcl::PointXYZI>::Ptr KeyPointsSiftZ(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{  
	// Parameters for sift computation
	const float min_scale = 0.005f;
	const int n_octaves = 6;
	const int n_scales_per_octave = 4;
	const float min_contrast = 0.005f;

	// Estimate the sift interest points using z values from xyz as the Intensity variants
	pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointXYZI> sift;
	pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI> ());
	sift.setSearchMethod(tree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(cloud);
	sift.compute(*result);

	std::cout << "No of SIFT Z Keypoints in the result are " << result->points.size () << std::endl;

	return result;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr KeyPointsISS(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{

	pcl::ISSKeypoint3D<pcl::PointXYZI, pcl::PointXYZI> iss_detector;
	pcl::PointCloud<pcl::PointXYZI>::Ptr result (new pcl::PointCloud<pcl::PointXYZI>());
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>());

	iss_detector.setSearchMethod(tree);
	iss_detector.setSalientRadius(10 * 0.05);
	iss_detector.setNonMaxRadius(8 * 0.05);
	iss_detector.setThreshold21(0.2);
	iss_detector.setThreshold32(0.2);
	iss_detector.setMinNeighbors(10);
	iss_detector.setNumberOfThreads(10);
	iss_detector.setInputCloud(cloud);
	iss_detector.compute(*result);

	std::cout << "No of ISS Keypoints in the result are " << result->points.size () << std::endl;

	return result;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	// Container for original data
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(*cloud_msg, *cloud);

	// VoxedGrid Filter
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

	pcl::VoxelGrid<pcl::PointXYZI> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.1, 0.1, 0.1);
	sor.filter (*cloud_filtered);

	// PassThrough Filter in X
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZI>);

	pcl::PassThrough<pcl::PointXYZI> pass;
	pass.setInputCloud(cloud_filtered);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-10, 10);
	pass.filter(*cloud_filtered2);

	// PassThrough Filter in Y
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered3(new pcl::PointCloud<pcl::PointXYZI>);

	pass.setInputCloud(cloud_filtered2);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-10, 16);
	pass.filter(*cloud_filtered3);

	// Obtengo KeyPoints por distintos métodos
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_KPHarris(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_KPSiftNE(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_KPSiftZ(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_KPISS(new pcl::PointCloud<pcl::PointXYZI>);

	cloud_KPHarris=KeyPointsHarris(cloud_filtered3);
	cloud_KPSiftNE=KeyPointsSiftNE(cloud_filtered3);
	cloud_KPSiftZ=KeyPointsSiftZ(cloud_filtered3);
	cloud_KPISS=KeyPointsISS(cloud_filtered3);

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);

	cloud_normal = Normals(cloud_filtered3);

	pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptorPFH_actual(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptorFPFH_actual(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptorVFH_actual(new pcl::PointCloud<pcl::VFHSignature308>());

	descriptorPFH_actual = PFH(cloud_filtered3, cloud_normal);
	descriptorFPFH_actual = FPFH(cloud_filtered3, cloud_normal);
	descriptorVFH_actual = VFH(cloud_filtered3, cloud_normal);

	if (keypoints_anterior != NULL){
		std::cout << "Puntos nube anterior: " << keypoints_anterior->points.size() << "  -  Puntos nube actual: " << cloud_KPSiftZ->points.size() << std::endl;
	}

	keypoints_anterior = cloud_KPSiftZ;

	descriptorPFH_anterior = descriptorPFH_actual;
	descriptorFPHF_anterior = descriptorFPFH_actual;
	descriptorVFH_anterior = descriptorVFH_actual;

	// Preparo como mensajes la nube filtrada y los KeyPoints que quiero visualizar
   	sensor_msgs::PointCloud2 outputF;
   	pcl::toROSMsg(*cloud_filtered3, outputF);
	outputF.header.frame_id = "/velodyne";

   	sensor_msgs::PointCloud2 outputKP;
   	pcl::toROSMsg(*cloud_KPHarris, outputKP);
	outputKP.header.frame_id = "/velodyne";

	// Publish the data
	pubF.publish (outputF);
	pubKP.publish (outputKP);
}

int main (int argc, char** argv)
{
	// Initialize the ROS Node "roscpp_pcl_example"
	ros::init (argc, argv, "roscpp_pcl_depurado");
	ros::NodeHandle nh;

	// Print "Hello" message with node name to the terminal and ROS log file
	ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());

	// Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function to cloud_cb
	sub = nh.subscribe(IMAGE_TOPIC, 1, cloud_cb);

	// Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
	pubF = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);
	pubKP = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC2, 1);

	// Spin
	ros::spin();

	// Success
	return 0;
}
