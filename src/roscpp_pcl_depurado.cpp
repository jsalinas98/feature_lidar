/**************************LIBRARIES**************************/

// Include the ROS library
#include <ros/ros.h>

// Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>

// Include pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

//Include features
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/shot.h>
#include <pcl/features/vfh.h>

//Include Visualizer and Plotter
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>

//Include keypoints
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>

#include <pcl/common/common.h>

/**************************DECLARATIONS***********************/
// Topics
static const std::string IMAGE_TOPIC = "/velodyne_points";
static const std::string PUBLISH_TOPIC = "/pcl/points";
static const std::string PUBLISH_TOPIC2 = "/pcl/points2";

// ROS Publisher
ros::Publisher pubF;
ros::Publisher pubKP;

// ROS Subscriber
ros::Subscriber sub;

pcl::visualization::PCLVisualizer::Ptr normalsVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");  // Edita la forma de ver los puntos
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 2, "normals");  //Muestra las lineas, (cloud, mormales, por cada x lineas muestra 1, long lineas, "")
	viewer->addCoordinateSystem (0.5); //Muestra los ejes
	viewer->initCameraParameters ();  //Inicia la vista en el origen
	return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr visualizer (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");  // Edita la forma de ver los puntos
	viewer->addCoordinateSystem (0.5); //Muestra los ejes
	viewer->initCameraParameters ();  //Inicia la vista en el origen
	return (viewer);
}

pcl::PointCloud<pcl::Normal>::Ptr Normals(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.03);

	// Compute the features
	ne.compute (*cloud_normals);

	return cloud_normals;
}

pcl::PointCloud<pcl::PFHSignature125>::Ptr PFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
	// PFH estimation object.
	pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud (cloud);
	pfh.setInputNormals (cloud_normals);
	
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
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


pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
	// FPFH estimation object.
	pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud (cloud);
	fpfh.setInputNormals (cloud_normals);
	
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
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

pcl::PointCloud<pcl::VFHSignature308>::Ptr VFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
	// Object for storing the VFH descriptor.
	pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

	// VFH estimation object.
	pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
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

pcl::PointCloud<pcl::PointXYZI>::Ptr KeyPointsInd(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::HarrisKeypoint3D <pcl::PointXYZ, pcl::PointXYZI> detector;
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZI>);
	detector.setNonMaxSupression (true);
	detector.setInputCloud (cloud);
	detector.setThreshold (1e-11);

	detector.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZ,pcl::PointXYZI>::HARRIS); 
	detector.setRefine(false);
	detector.setRadius(0.5); 

	detector.compute (*keypoints);
	pcl::console::print_highlight ("Detected %zd points in algunos s\n", keypoints->size ());

	return keypoints;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr KeyPointsSiftNE(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// Parameters for sift computation
	const float min_scale = 0.01f;
	const int n_octaves = 3;
	const int n_scales_per_octave = 4;
	const float min_contrast = 0.001f;

	// Estimate the normals of the cloud_xyz
	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

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
	pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale> result;
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal> ());
	sift.setSearchMethod(tree2);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(cloud_normals);
	sift.compute(result);

	std::cout << "No of SIFT points in the result are " << result.points.size () << std::endl;

	// Copying the pointwithscale to pointxyz so as visualize the cloud
	//Probar a cambiar el tipo de salida para no hacer esto
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZI>);
	copyPointCloud(result, *cloud_temp);

	return cloud_temp;
}

// Incluir al usar SIFTKeyPointsFieldSelector para que seleccione segun la Z.
namespace pcl
{
  template<>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
      inline float
      operator () (const PointXYZ &p) const
      {
	return p.z;
      }
    };
}

pcl::PointCloud<pcl::PointXYZI>::Ptr KeyPointsSiftZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{  
	// Parameters for sift computation
	const float min_scale = 0.005f;
	const int n_octaves = 6;
	const int n_scales_per_octave = 4;
	const float min_contrast = 0.005f;

	// Estimate the sift interest points using z values from xyz as the Intensity variants
	pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale> result;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
	sift.setSearchMethod(tree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(cloud);
	sift.compute(result);

	std::cout << "No of SIFT points in the result are " << result.points.size () << std::endl;

	// Copying the pointwithscale to pointxyz so as visualize the cloud
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZI>);
	copyPointCloud(result, *cloud_temp);

/*
	// Visualization of keypoints along with the original cloud
	pcl::visualization::PCLVisualizer viewer("PCL Viewer");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler (cloud_temp, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud, 255, 0, 0);
	viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
	viewer.addPointCloud(cloud, cloud_color_handler, "cloud");
	viewer.addPointCloud(cloud_temp, keypoints_color_handler, "keypoints");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

	while(!viewer.wasStopped ())
	{
		viewer.spinOnce ();
	}
*/
	return cloud_temp;

}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	// Container for original data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromROSMsg(*cloud_msg, *cloud);

	// VoxedGrid Filter
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.1, 0.1, 0.1);
	sor.filter (*cloud_filtered);

	// PassThrough Filter in X
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(cloud_filtered);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-10, 10);
	pass.filter(*cloud_filtered2);

	// PassThrough Filter in Y
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered3(new pcl::PointCloud<pcl::PointXYZ>);

	pass.setInputCloud(cloud_filtered2);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-10, 16);
	pass.filter(*cloud_filtered3);

	//Obtengo KeyPoints por distintos métodos
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_KPHarris(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_KPSiftNE(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_KPSiftZ(new pcl::PointCloud<pcl::PointXYZI>);

	cloud_KPHarris=KeyPointsInd(cloud_filtered3);
	cloud_KPSiftNE=KeyPointsSiftNE(cloud_filtered3);
	cloud_KPSiftZ=KeyPointsSiftZ(cloud_filtered3);

	//Preparo como mensajes la nube filtrada y los KeyPoints que quiero visualizar
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
	ros::init (argc, argv, "roscpp_pcl_example");
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
