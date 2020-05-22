/*
* PCL Example using ROS and CPP
*/

/*
 * PRUEBA1 -> Calcular normales
 * PRUEBA2 -> Visualizador 3D con las normales
 * PRUEBA DESCRIPTORES -> Calcula y plotea diferentes descriptores:
 *		-Funcionan: PFH, FPFH
 *		-No funcionan: 3DSC, SHOT
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
//#include <pcl/visualization/cloud_viewer.h>
/********************************************** FIN PRUEBA2 ****************************************************************/

/********************************************** PRUEBA DESCRIPTORES ********************************************************************/
#include <pcl/features/pfh.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/shot.h>
#include <pcl/features/vfh.h>
#include <pcl/visualization/histogram_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
/********************************************** FIN PRUEBA DESCRIPTORES ****************************************************************/

/********************************************** PRUEBA KEYPOINTS ********************************************************************/
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
/********************************************** FIN PRUEBA KEYPOINTS ****************************************************************/

#include <pcl/common/common.h>

// Topics
static const std::string IMAGE_TOPIC = "/velodyne_points";
//static const std::string IMAGE_TOPIC = "/point_cloud"; //Para la vaca
static const std::string PUBLISH_TOPIC = "/pcl/points";
static const std::string PUBLISH_TOPIC2 = "/pcl/points2";

// ROS Publisher
ros::Publisher pub;
ros::Publisher pub2;
/*************************************LO NUEVO INSTANTES ANTERIORES*************************************/
//Variables globales
pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_anterior (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_actuales (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptorPFH_anterior(new pcl::PointCloud<pcl::PFHSignature125>());
pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptorPFH_actual(new pcl::PointCloud<pcl::PFHSignature125>());
pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptorFPHF_anterior(new pcl::PointCloud<pcl::FPFHSignature33>());
pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptorFPHF_actuales(new pcl::PointCloud<pcl::FPFHSignature33>());
pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptorVFH_anterior(new pcl::PointCloud<pcl::VFHSignature308>);
pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptorVFH_actuales(new pcl::PointCloud<pcl::VFHSignature308>);

/********************************************************************************************************/
/********************************************** PRUEBA2 ********************************************************************
pcl::visualization::PCLVisualizer::Ptr normalsVis (
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{

	//pcl::visualization::CloudViewer viewer2 ("PCL Viewer");
	//viewer2.showCloud (cloud);

	// --------------------------------------------------------
	// -----Open 3D viewer and add point cloud and normals-----
	// --------------------------------------------------------
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
	//viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");  // Edita la forma de ver los puntos
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 2, "normals");  //Muestra las lineas, (cloud, mormales, por cada x lineas muestra 1, long lineas, "")
	viewer->addCoordinateSystem (0.5); //Muestra los ejes
	viewer->initCameraParameters ();  //Inicia la vista en el origen
	return (viewer);
}
/********************************************** FIN PRUEBA2 ****************************************************************/

/********************************************** PRUEBA DESCRIPTORES ********************************************************************/

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
	pcl::visualization::PCLHistogramVisualizer viewer;
	// We need to set the size of the descriptor beforehand.
	viewer.addFeatureHistogram(*descriptor, 125);

	viewer.spin();
*/
}


pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, int num)
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

	// Imprime valor fpfh para el punto indicado por argumentos (int num)
	pcl::FPFHSignature33 prueba = descriptor->points[num];
	std::cout << prueba << std::endl;
/*
	// Plotter object.
	pcl::visualization::PCLPlotter plotter;
	// We need to set the size of the descriptor beforehand.
	plotter.addFeatureHistogram(*descriptor, 33);

	plotter.plot();
*/
}


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

pcl::PointCloud<pcl::VFHSignature308>::Ptr VFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
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
/*
	// Plotter object.
	pcl::visualization::PCLPlotter plotter;
	// We need to set the size of the descriptor beforehand.
	plotter.addFeatureHistogram(*descriptor, 308);

	plotter.plot();
*/
}

/********************************************** FIN PRUEBA DESCRIPTORES ****************************************************************/

/********************************************** PRUEBA KEYPOINTS ***********************************************************************/

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

pcl::PointCloud<pcl::PointXYZ>::Ptr KeyPointsSiftNE(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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

	// Copying the pointwithscale to pointxyz so as visualize the cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
	copyPointCloud(result, *cloud_temp);

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

pcl::PointCloud<pcl::PointXYZ>::Ptr KeyPointsSiftZ(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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

	// Copying the pointwithscale to pointxyz so as visualize the cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>);
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
	//*/

	return cloud_temp;
}

/********************************************** FIN PRUEBA KEYPOINTS *******************************************************************/

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	// Container for original & filtered data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	// Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pa_filtrar (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2 (*cloudPtr, *cloud_pa_filtrar);

	pcl::PointXYZ minPt, maxPt;
  	pcl::getMinMax3D (*cloud_pa_filtrar, minPt, maxPt);


	// Perform the actual filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize (0.1, 0.1, 0.1);
	sor.filter (cloud_filtered);

	// cascade the floor removal filter and define a container for floorRemoved	
	pcl::PCLPointCloud2::Ptr floorRemoved (new pcl::PCLPointCloud2 ());
	pcl::PCLPointCloud2ConstPtr cloud_filtered_ptr (&cloud_filtered);
		
	// define a PassThrough filter
	pcl::PassThrough<pcl::PCLPointCloud2> pass;
	// set input to cloudVoxel
	pass.setInputCloud(cloud_filtered_ptr);
	// filter along z-axis
	pass.setFilterFieldName("z");
	// set z-limits
	pass.setFilterLimits(minPt.z+2, maxPt.z);
	pass.filter(*floorRemoved);

	std::cout << "Min z: " << minPt.z+2 << std::endl;
	std::cout << "Max z: " << maxPt.z << std::endl;

	/********************************************** PRUEBA1 ********************************************************************/
	//Prueba Superficies Normales
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_nueva (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2 (*floorRemoved, *cloud_nueva);

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
	ne.setRadiusSearch (0.03);

	// Compute the features
	ne.compute (*cloud_normals);

	/*************************************** PRUEBA SALINAS CONCATENAR ***********************************************/
	// concatenate the XYZ and normal fields
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields<pcl::PointXYZ, pcl::Normal, pcl::PointNormal>(*cloud_nueva, *cloud_normals, *cloud_with_normals);
	/*****************************************************************************************************************/



	/********************************************** PRUEBA DESCRIPTORES ********************************************************************/
    /**********************************************LO NUEVO CORRESPONDENCIA****************************************************/

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_kp (new pcl::PointCloud<pcl::PointXYZ>);
	//KeyPointsInd(cloud_nueva);
	//cloud_kp=KeyPointsSiftNE(cloud_nueva);
	cloud_kp=KeyPointsSiftZ(cloud_nueva);

	//descriptorPFH_actual = PFH(cloud_nueva, cloud_normals);
	//descriptorFPFH_actual = FPFH(cloud_nueva, cloud_normals);
    //descriptorVFH_actual = VFH(cloud_nueva, cloud_normals);
	
	/* No van aun
	SC_3D(cloud_nueva, cloud_normals);
	SHOT(cloud_nueva, cloud_normals);
	//*/

	// Aquí cambiar las variables según se vaya a probar uno u otro
	if (keypoints_anterior != NULL){
		// Aquí las fucioines para la correspondencia, q tendrán q devolver el dato de la correspondencia
		// Pasando por argumento las keypoints y features actales y anteriores
	}
	//descriptorPFH_anterior = descriptorPFH_actual;
	//descriptorFPFH_anterior = descriptorPFH_actual;
	//descriptorVFH_anterior = descriptorVFH_actual;
	keypoints_anterior = cloud_kp;

	/***************************************************************************************************************************/
	/********************************************** FIN PRUEBA DESCRIPTORES ****************************************************************/

	/********************************************** FIN PRUEBA1 ****************************************************************/


	/********************************************** PRUEBA2 ********************************************************************
	pcl::visualization::PCLVisualizer::Ptr viewer;
    viewer = normalsVis(cloud_nueva, cloud_normals);
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
	}
	/********************************************** FIN PRUEBA2 ****************************************************************/

	/********************************************* NUEVA CONVERSION SALIANS******************************************/

	pcl::toPCLPointCloud2(*cloud_kp, cloud_filtered);
   	sensor_msgs::PointCloud2 output;
   	pcl_conversions::fromPCL(cloud_filtered, output);
	output.header.frame_id = "/velodyne";

	//pcl::toPCLPointCloud2(*floorRemoved, cloud_filtered);
   	sensor_msgs::PointCloud2 output2;
   	pcl_conversions::fromPCL(*floorRemoved, output2);

	/********************************************************************************************************/
/*
	// Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	//pcl_conversions::fromPCL(cloud_filtered, output);   //Conversion antigua

	/********************************************** PRUEBA1 ********************************************************************
	pcl::toROSMsg(*cloud_normals, output);    //Conversion nueva
	//No da error e imprime el mensaje, pero no lo visualizo en el rviz
	/********************************************** FIN PRUEBA1 ****************************************************************/

	// Publish the data
	pub.publish (output);
	pub2.publish (output2);
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
	pub2 = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC2, 1);

	// Spin
	ros::spin();

	// Success
	return 0;
}
