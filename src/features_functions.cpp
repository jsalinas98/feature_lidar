#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include<pcl/visualization/histogram_visualizer.h>

void PFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
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

	// Use all neighbors in a sphere of radius 5cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	pfh.setRadiusSearch (0.1);

	// Compute the features
	pfh.compute (*descriptor);

	// Plotter object.
	pcl::visualization::PCLHistogramVisualizer viewer;
	// We need to set the size of the descriptor beforehand.
	viewer.addFeatureHistogram(*descriptor, 125);

	viewer.spin();
}


#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/visualization/pcl_plotter.h>

void FPFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
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
	
	// Use all neighbors in a sphere of radius 5cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	fpfh.setRadiusSearch (0.1);

	// Compute the features
	fpfh.compute (*descriptor);

	// Plotter object.
	pcl::visualization::PCLPlotter plotter;
	// We need to set the size of the descriptor beforehand.
	plotter.addFeatureHistogram(*descriptor, 33);

	plotter.plot();
}

#include <pcl/features/normal_3d.h>
#include <pcl/features/3dsc.h>
#include <pcl/visualization/pcl_plotter.h>

void SC_3D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
	// 3DSC estimation object.
	pcl::ShapeContext3DEstimation<pcl::PointXYZ, pcl::Normal, pcl::ShapeContext1980> sc3d;
	sc3d.setInputCloud (cloud);
	sc3d.setInputNormals (cloud_normals);
	
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	sc3d.setSearchMethod (tree);

	// Object for storing the 3DSC descriptors for each point.
	pcl::PointCloud<pcl::ShapeContext1980>::Ptr descriptor(new pcl::PointCloud<pcl::ShapeContext1980>());
	
	// Search radius, to look for neighbors. It will also be the radius of the support sphere.
	sc3d.setRadiusSearch(0.05);
	// The minimal radius value for the search sphere, to avoid being too sensitive
	// in bins close to the center of the sphere.
	sc3d.setMinimalRadius(0.05 / 10.0);
	// Radius used to compute the local point density for the neighbors
	// (the density is the number of points within that radius).
	sc3d.setPointDensityRadius(0.05 / 5.0);

	// Compute the features
	sc3d.compute (*descriptor);

	/* No va el plot de esto
	// Plotter object.
	pcl::visualization::PCLPlotter plotter;
	// We need to set the size of the descriptor beforehand.
	plotter.addFeatureHistogram(*descriptor, 1980);

	plotter.plot();
	//*/
}

#include <pcl/features/normal_3d.h>
#include <pcl/features/shot.h>
#include <pcl/visualization/pcl_plotter.h>

void SHOT(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
	// SHOT estimation object.
	pcl::SHOTEstimation<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> shot;
	shot.setInputCloud (cloud);
	shot.setInputNormals (cloud_normals);
	
	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	shot.setSearchMethod (tree);

	// Object for storing the SHOT descriptors for each point.
	pcl::PointCloud<pcl::SHOT352>::Ptr descriptor(new pcl::PointCloud<pcl::SHOT352>());

	// The radius that defines which of the keypoint's neighbors are described.
	// If too large, there may be clutter, and if too small, not enough points may be found.
	shot.setRadiusSearch(0.02);

	// Compute the features
	shot.compute (*descriptor);

	/* No va el plot de esto
	// Plotter object.
	pcl::visualization::PCLPlotter plotter;
	// We need to set the size of the descriptor beforehand.
	plotter.addFeatureHistogram(*descriptor, 352);

	plotter.plot();
	//*/
}

#include <pcl/features/normal_3d.h>
#include <pcl/features/vfh.h>
#include <pcl/visualization/pcl_plotter.h>

void VFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
	// Object for storing the VFH descriptor.
	pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptor(new pcl::PointCloud<pcl::VFHSignature308>);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);

	// VFH estimation object.
	pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud(cloud);
	vfh.setInputNormals(cloud_normals);
	vfh.setSearchMethod(kdtree);
	// Optionally, we can normalize the bins of the resulting histogram,
	// using the total number of points.
	vfh.setNormalizeBins(true);
	// Also, we can normalize the SDC with the maximum size found between
	// the centroid and any of the cluster's points.
	vfh.setNormalizeDistance(false);

	vfh.compute(*descriptor);

	// Plotter object.
	pcl::visualization::PCLPlotter plotter;
	// We need to set the size of the descriptor beforehand.
	plotter.addFeatureHistogram(*descriptor, 308);

	plotter.plot();
}

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/harris_3d.h>

void KeyPointsInd(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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
	
	/*
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZI> (keypoints, "sample cloud");
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");  // Edita la forma de ver los puntos
	viewer->addCoordinateSystem (0.5); //Muestra los ejes
	viewer->initCameraParameters ();  //Inicia la vista en el origen
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
	}
	//*/
}

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>

void KeyPointsSiftNE(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// Parameters for sift computation
	const float min_scale = 0.01f;
	const int n_octaves = 3;
	const int n_scales_per_octave = 4;
	const float min_contrast = 0.001f;

	// Estimate the normals of the cloud_xyz
	pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());

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

	std::cout << "No of SIFT points in the result are " << result.points.size () << std::endl;

	/*
	// Copying the pointwithscale to pointxyz so as visualize the cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
	copyPointCloud(result, *cloud_temp);

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
	//*/
}

#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/visualization/pcl_visualizer.h>

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

void KeyPointsSiftZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{  
	// Parameters for sift computation
	const float min_scale = 0.005f;
	const int n_octaves = 6;
	const int n_scales_per_octave = 4;
	const float min_contrast = 0.005f;

	// Estimate the sift interest points using z values from xyz as the Intensity variants
	pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
	pcl::PointCloud<pcl::PointWithScale> result;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ> ());
	sift.setSearchMethod(kdtree);
	sift.setScales(min_scale, n_octaves, n_scales_per_octave);
	sift.setMinimumContrast(min_contrast);
	sift.setInputCloud(cloud);
	sift.compute(result);

	std::cout << "No of SIFT points in the result are " << result.points.size () << std::endl;

	/*
	// Copying the pointwithscale to pointxyz so as visualize the cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
	copyPointCloud(result, *cloud_temp);

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
	//*/
}