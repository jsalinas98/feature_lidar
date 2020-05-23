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

/*******************Include del gilipollas de salinas*********************************************/
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>

/**************************************************************************************************/
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

pcl::PointCloud<pcl::PFHSignature125>::Ptr PFH(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, const pcl::PointIndicesConstPtr keypoints_indices)
{
	// PFH estimation object.
	pcl::PFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::PFHSignature125> pfh;
	pfh.setInputCloud (cloud);
	pfh.setInputNormals (cloud_normals);
	if(keypoints_indices != NULL)
		pfh.setIndices(keypoints_indices);
	
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


pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFH(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, const pcl::PointIndicesConstPtr keypoints_indices)
{
	// FPFH estimation object.
	pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud (cloud);
	fpfh.setInputNormals (cloud_normals);
	if(keypoints_indices != NULL)
		fpfh.setIndices(keypoints_indices);
	
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

pcl::PointCloud<pcl::VFHSignature308>::Ptr VFH(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, const pcl::PointIndicesConstPtr keypoints_indices)
{
	// Object for storing the VFH descriptor.
	pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);

	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);

	// VFH estimation object.
	pcl::VFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud(cloud);
	vfh.setInputNormals(cloud_normals);
	if(keypoints_indices != NULL)
		vfh.setIndices(keypoints_indices);
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

pcl::PointCloud<pcl::PointXYZI> KeyPointsHarris(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointIndicesConstPtr* keypoints_indices)
{
	pcl::PointCloud<pcl::PointXYZI> keypoints;

	pcl::HarrisKeypoint3D <pcl::PointXYZI, pcl::PointXYZI> detector;
	detector.setNonMaxSupression (true);
	detector.setInputCloud (cloud);
	detector.setThreshold (1e-11);
	detector.setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZI,pcl::PointXYZI>::HARRIS); 
	detector.setRefine(false);
	detector.setRadius(0.5);
	detector.compute (keypoints);
	*keypoints_indices = detector.getKeypointsIndices();

	std::cout << "No of Harris Keypoints in the result are " << keypoints.points.size () << std::endl;

	return keypoints;
}

pcl::PointCloud<pcl::PointXYZI> KeyPointsSiftNE(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointIndicesConstPtr* keypoints_indices)
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
	pcl::PointCloud<pcl::PointXYZI> keypoints;
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal> ());

	pcl::SIFTKeypoint<pcl::PointNormal, pcl::PointXYZI> sift;
	sift.setSearchMethod(tree2);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(cloud_normals);
	sift.compute(keypoints);
	*keypoints_indices = sift.getKeypointsIndices();

	std::cout << "No of SIFT NE Keypoints in the result are " << keypoints.points.size () << std::endl;

	return keypoints;
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

pcl::PointCloud<pcl::PointXYZI> KeyPointsSiftZ(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointIndicesConstPtr* keypoints_indices)
{  
	// Parameters for sift computation
	const float min_scale = 0.005f;
	const int n_octaves = 6;
	const int n_scales_per_octave = 4;
	const float min_contrast = 0.005f;

	// Estimate the sift interest points using z values from xyz as the Intensity variants
	pcl::PointCloud<pcl::PointXYZI> keypoints;
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI> ());

	pcl::SIFTKeypoint<pcl::PointXYZI, pcl::PointXYZI> sift;
	sift.setSearchMethod(tree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(cloud);
	sift.compute(keypoints);
	*keypoints_indices = sift.getKeypointsIndices();

	std::cout << "No of SIFT Z Keypoints in the result are " << keypoints.points.size () << std::endl;

	return keypoints;
}

pcl::PointCloud<pcl::PointXYZI> KeyPointsISS(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointIndicesConstPtr* keypoints_indices)
{

	pcl::ISSKeypoint3D<pcl::PointXYZI, pcl::PointXYZI> iss_detector;
	pcl::PointCloud<pcl::PointXYZI> keypoints;
	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>());

	iss_detector.setSearchMethod(tree);
	iss_detector.setSalientRadius(10 * 0.05);
	iss_detector.setNonMaxRadius(8 * 0.05);
	iss_detector.setThreshold21(0.2);
	iss_detector.setThreshold32(0.2);
	iss_detector.setMinNeighbors(10);
	iss_detector.setNumberOfThreads(10);
	iss_detector.setInputCloud(cloud);
	iss_detector.compute(keypoints);
	*keypoints_indices = iss_detector.getKeypointsIndices();

	std::cout << "No of ISS Keypoints in the result are " << keypoints.points.size () << std::endl;

	return keypoints;
}

pcl::CorrespondencesPtr correspondences_PFH(const pcl::PointCloud<pcl::PFHSignature125>::Ptr source_features, 
	const pcl::PointCloud<pcl::PFHSignature125>::Ptr target_features,
	const pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints,
	const pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints){
	// estimate correspondences
    pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> est;
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
    est.setInputSource(source_features);
    est.setInputTarget(target_features);
    est.determineCorrespondences(*correspondences);

    // Duplication rejection Duplicate

    pcl::CorrespondencesPtr correspondences_result_rej_one_to_one(new pcl::Correspondences());
    pcl::registration::CorrespondenceRejectorOneToOne corr_rej_one_to_one;
    corr_rej_one_to_one.setInputCorrespondences(correspondences);
    corr_rej_one_to_one.getCorrespondences(*correspondences_result_rej_one_to_one);


    // Correspondance rejection RANSAC

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rejector_sac;
    pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences());
    rejector_sac.setInputSource(source_keypoints);
    rejector_sac.setInputTarget(target_keypoints);
    rejector_sac.setInlierThreshold(2.5); // distance in m, not the squared distance
    rejector_sac.setMaximumIterations(1000000);
    rejector_sac.setRefineModel(false);
    rejector_sac.setInputCorrespondences(correspondences_result_rej_one_to_one);;
    rejector_sac.getCorrespondences(*correspondences_filtered);
    std::cout << correspondences_filtered->size() << " vs. " << correspondences->size() << std::endl;
/*
    transform = rejector_sac.getBestTransformation();   // Transformation Estimation method 1
    std::cout << "Estimated Transform PFH:" << std::endl << transform << std::endl;
*/

    // Transformation Estimation method 2
    //pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformation_estimation;
    //transformation_estimation.estimateRigidTransformation(*source_keypoints, *target_keypoints, *correspondences, transform);

    return correspondences_filtered;
}

pcl::CorrespondencesPtr correspondences_FPFH(const pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features, 
	const pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features,
	const pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints,
	const pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints){
	// estimate correspondences
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
    est.setInputSource(source_features);
    est.setInputTarget(target_features);
    est.determineCorrespondences(*correspondences);

    // Duplication rejection Duplicate

    pcl::CorrespondencesPtr correspondences_result_rej_one_to_one(new pcl::Correspondences());
    pcl::registration::CorrespondenceRejectorOneToOne corr_rej_one_to_one;
    corr_rej_one_to_one.setInputCorrespondences(correspondences);
    corr_rej_one_to_one.getCorrespondences(*correspondences_result_rej_one_to_one);


    // Correspondance rejection RANSAC

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rejector_sac;
    pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences());
    rejector_sac.setInputSource(source_keypoints);
    rejector_sac.setInputTarget(target_keypoints);
    rejector_sac.setInlierThreshold(2.5); // distance in m, not the squared distance
    rejector_sac.setMaximumIterations(1000000);
    rejector_sac.setRefineModel(false);
    rejector_sac.setInputCorrespondences(correspondences_result_rej_one_to_one);;
    rejector_sac.getCorrespondences(*correspondences_filtered);
    std::cout << correspondences_filtered->size() << " vs. " << correspondences->size() << std::endl;
/*
    transform = rejector_sac.getBestTransformation();   // Transformation Estimation method 1
    std::cout << "Estimated Transform FPFH:" << std::endl << transform << std::endl;
*/

    // Transformation Estimation method 2
    //pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformation_estimation;
    //transformation_estimation.estimateRigidTransformation(*source_keypoints, *target_keypoints, *correspondences, transform);

    return correspondences_filtered;
}

pcl::CorrespondencesPtr correspondences_VFH(const pcl::PointCloud<pcl::VFHSignature308>::Ptr source_features, 
	const pcl::PointCloud<pcl::VFHSignature308>::Ptr target_features,
	const pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints,
	const pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints){
	// estimate correspondences
    pcl::registration::CorrespondenceEstimation<pcl::VFHSignature308, pcl::VFHSignature308> est;
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
    est.setInputSource(source_features);
    est.setInputTarget(target_features);
    est.determineCorrespondences(*correspondences);

    // Duplication rejection Duplicate

    pcl::CorrespondencesPtr correspondences_result_rej_one_to_one(new pcl::Correspondences());
    pcl::registration::CorrespondenceRejectorOneToOne corr_rej_one_to_one;
    corr_rej_one_to_one.setInputCorrespondences(correspondences);
    corr_rej_one_to_one.getCorrespondences(*correspondences_result_rej_one_to_one);


    // Correspondance rejection RANSAC

    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rejector_sac;
    pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences());
    rejector_sac.setInputSource(source_keypoints);
    rejector_sac.setInputTarget(target_keypoints);
    rejector_sac.setInlierThreshold(2.5); // distance in m, not the squared distance
    rejector_sac.setMaximumIterations(1000000);
    rejector_sac.setRefineModel(false);
    rejector_sac.setInputCorrespondences(correspondences_result_rej_one_to_one);;
    rejector_sac.getCorrespondences(*correspondences_filtered);
    std::cout << correspondences_filtered->size() << " vs. " << correspondences->size() << std::endl;
/*
    transform = rejector_sac.getBestTransformation();   // Transformation Estimation method 1
    std::cout << "Estimated Transform VFH:" << std::endl << transform << std::endl;
*/

    // Transformation Estimation method 2
    //pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> transformation_estimation;
    //transformation_estimation.estimateRigidTransformation(*source_keypoints, *target_keypoints, *correspondences, transform);

    return correspondences_filtered;
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
	pcl::PointCloud<pcl::PointXYZI> cloud_KPHarris;
	pcl::PointCloud<pcl::PointXYZI> cloud_KPSiftNE;
	pcl::PointCloud<pcl::PointXYZI> cloud_KPSiftZ;
	pcl::PointCloud<pcl::PointXYZI> cloud_KPISS;

	pcl::PointIndicesConstPtr indices_KPHarris;
	pcl::PointIndicesConstPtr indices_KPSiftNE;
	pcl::PointIndicesConstPtr indices_KPSiftZ;
	pcl::PointIndicesConstPtr indices_KPISS;

	cloud_KPHarris = KeyPointsHarris(cloud_filtered3, &indices_KPHarris);
	cloud_KPSiftNE = KeyPointsSiftNE(cloud_filtered3, &indices_KPSiftNE);
	cloud_KPSiftZ = KeyPointsSiftZ(cloud_filtered3, &indices_KPSiftZ);
	cloud_KPISS = KeyPointsISS(cloud_filtered3, &indices_KPISS);

	//Obtengo las normales
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
	cloud_normal = Normals(cloud_filtered3);

	//Obtengo los distintos descriptores de las distintas nubes de keypoints (Obviamente hay que meter las llamadas pk ahora solo está para los KPSiftNE)
	pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptorPFH_actual(new pcl::PointCloud<pcl::PFHSignature125>());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptorFPFH_actual(new pcl::PointCloud<pcl::FPFHSignature33>());
	pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptorVFH_actual(new pcl::PointCloud<pcl::VFHSignature308>());

	descriptorPFH_actual = PFH(cloud_filtered3, cloud_normal, indices_KPSiftNE);
	descriptorFPFH_actual = FPFH(cloud_filtered3, cloud_normal, indices_KPSiftNE);
	descriptorVFH_actual = VFH(cloud_filtered3, cloud_normal, indices_KPSiftNE);

	if (keypoints_anterior != NULL){
		correspondences_PFH(descriptorPFH_anterior, descriptorPFH_actual, keypoints_anterior, cloud_filtered3);
		correspondences_FPFH(descriptorFPHF_anterior, descriptorFPFH_actual, keypoints_anterior, cloud_filtered3);
		correspondences_VFH(descriptorVFH_anterior, descriptorVFH_actual, keypoints_anterior, cloud_filtered3);
	}

	keypoints_anterior = cloud_filtered3;

	descriptorPFH_anterior = descriptorPFH_actual;
	descriptorFPHF_anterior = descriptorFPFH_actual;
	descriptorVFH_anterior = descriptorVFH_actual;

	// Preparo como mensajes la nube filtrada y los KeyPoints que quiero visualizar
   	sensor_msgs::PointCloud2 outputF;
   	pcl::toROSMsg(*cloud_filtered3, outputF);
	outputF.header.frame_id = "/velodyne";

   	sensor_msgs::PointCloud2 outputKP;
   	pcl::toROSMsg(*cloud_filtered3, outputKP);
	outputKP.header.frame_id = "/velodyne";

	// Publish the data
	pubF.publish (outputF);
	pubKP.publish (outputKP);
    std::cout << std::endl << std::endl;
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
