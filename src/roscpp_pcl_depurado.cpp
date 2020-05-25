/**************************LIBRARIES**************************/

// Include the ROS library
#include <ros/ros.h>

// Include PointCloud2 message
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float32.h>

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

//Include keypoints
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/iss_3d.h>

//Include correspondences
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

/**************************************************************************************************/
/**************************DECLARATIONS***********************/
// Topics
static const std::string IMAGE_TOPIC = "/velodyne_points";
static const std::string PUBLISH_TOPIC = "/pcl/points";
static const std::string PUBLISH_TOPIC2 = "/pcl/points2";
static const std::string PUBLISH_TOPIC_PY = "/data/ToPy";

//Variables globales
pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_anterior (new pcl::PointCloud<pcl::PointXYZI>);

pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptorPFH_anterior(new pcl::PointCloud<pcl::PFHSignature125>());
pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptorFPHF_anterior(new pcl::PointCloud<pcl::FPFHSignature33>());
pcl::PointCloud<pcl::VFHSignature308>::Ptr descriptorVFH_anterior(new pcl::PointCloud<pcl::VFHSignature308>);


// ROS Publisher
ros::Publisher pubF;
ros::Publisher pubKP;
ros::Publisher pubToPy;

// ROS Subscriber
ros::Subscriber sub;
#define COLA_RECEPCION 1

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
	if(keypoints_indices->indices.size() > 0)
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
	return descriptor;
}

pcl::PointCloud<pcl::PFHSignature125>::Ptr PFH(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
	pcl::PointIndices::Ptr indices_KP_null(new pcl::PointIndices);
	return PFH(cloud, cloud_normals, indices_KP_null);
}


pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFH(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, const pcl::PointIndicesConstPtr keypoints_indices)
{
	// FPFH estimation object.
	pcl::FPFHEstimation<pcl::PointXYZI, pcl::Normal, pcl::FPFHSignature33> fpfh;
	fpfh.setInputCloud (cloud);
	fpfh.setInputNormals (cloud_normals);
	if(keypoints_indices->indices.size() > 0)
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
	return descriptor;
}

pcl::PointCloud<pcl::FPFHSignature33>::Ptr FPFH(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
{
	pcl::PointIndices::Ptr indices_KP_null(new pcl::PointIndices);
	return FPFH(cloud, cloud_normals, indices_KP_null);
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

int all_kp[]={0,0,0,0};
void AllKeyPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::PointIndicesConstPtr* keypoints_indices)
{
	all_kp[0]+=KeyPointsHarris(cloud, keypoints_indices).points.size();
	all_kp[1]+=KeyPointsISS(cloud, keypoints_indices).points.size();
	all_kp[2]+=KeyPointsSiftZ(cloud, keypoints_indices).points.size();
	all_kp[3]+=KeyPointsSiftNE(cloud, keypoints_indices).points.size();
}

pcl::CorrespondencesPtr correspondences_PFH(const pcl::PointCloud<pcl::PFHSignature125>::Ptr source_features, 
	const pcl::PointCloud<pcl::PFHSignature125>::Ptr target_features,
	const pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints,
	const pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints)
{
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

	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rejector_sac;
	pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences());
	rejector_sac.setInputSource(source_keypoints);
	rejector_sac.setInputTarget(target_keypoints);
	rejector_sac.setInlierThreshold(2.5); // distance in m, not the squared distance
	rejector_sac.setMaximumIterations(1000);
	rejector_sac.setRefineModel(false);
	rejector_sac.setInputCorrespondences(correspondences_result_rej_one_to_one);;
	rejector_sac.getCorrespondences(*correspondences_filtered);
	std::cout << correspondences_filtered->size() << " vs. " << correspondences->size() << std::endl;

	return correspondences_filtered;
}

pcl::CorrespondencesPtr correspondences_FPFH(const pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features, 
	const pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features,
	const pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints,
	const pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints)
{
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

	return correspondences_filtered;
}

int i=0;
int n_corr=0;
int n_kp=0;
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, char* keypoints_type, char* feature_type)
{
	i++;
	std::cout << "Aplico los metodos indicados: Keypoint-> " << keypoints_type <<" , feature -> " << feature_type << std::endl;
	std::cout << std::endl;
	// Container for original data
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::fromROSMsg(*cloud_msg, *cloud);

/********************** FILTRADO DE LA NUBE DE PUNTOS ******************************************/
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

/********************* OBTENCION DE LOS KEYPOINTS POR EL METODO ELEGIDO ***************************/
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_KP(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointIndicesConstPtr indices_KP;

	if (strcmp(keypoints_type,"KPH")==0)
	{
		std::cout << "Calculo Keypoints por Harris3D" << std::endl;
		*cloud_KP = KeyPointsHarris(cloud_filtered3, &indices_KP);
	}
	else if (strcmp(keypoints_type,"KPISS")==0)
	{
		std::cout << "Calculo Keypoints por ISS3D" << std::endl;
		*cloud_KP = KeyPointsISS(cloud_filtered3, &indices_KP);
	}
	else if (strcmp(keypoints_type,"KP")==0)
	{
		std::cout << "Calculo Keypoints por diferentes metodos: Harris3D, ISS3D, SiftZ, SiftNE" << std::endl;
		AllKeyPoints(cloud_filtered3, &indices_KP);
	}
	else if (strcmp(keypoints_type,"0")==0)
	{
		cloud_KP = cloud_filtered3;
		std::cout << "No calculo Keypoints" << std::endl;
	}
	else
	{
		std::cout << "[ERROR]: Metodo "<< keypoints_type << " no encontrado" << std::endl;
		std::cout << "Introduzca uno de los siguientes metodos como primer parametro de la funcion, si procede: Harris -> KPH, ISS3D -> KPISS" << std::endl;
	}
	std::cout << std::endl;
	n_kp+=cloud_KP->points.size();
	//Obtengo las normales
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>);
	cloud_normal = Normals(cloud_filtered3);

/********************** OBTENCION DE LA FEATURE ELEGIDA Y CALCULO DE LA CORRESPONDENCIA, SI PROCEDE ***************************/
	if (strcmp(feature_type,"PFH")==0)
	{
		std::cout << "Calculo feature PFH" << std::endl;
		pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptorPFH_actual(new pcl::PointCloud<pcl::PFHSignature125>());
		if (strcmp(keypoints_type,"0")==0)
		{
			descriptorPFH_actual = PFH(cloud_filtered3, cloud_normal);
		}
		else
		{
			descriptorPFH_actual = PFH(cloud_filtered3, cloud_normal, indices_KP);
		}
		if (keypoints_anterior->points.size() > 0)
		{
			std::cout << "Calculo correspondencia PFH" << std::endl;
			int corr=correspondences_PFH(descriptorPFH_anterior, descriptorPFH_actual, keypoints_anterior, cloud_KP)->size();
			std_msgs::Float32 outToPy;
			outToPy.data = (float)(corr);
			pubToPy.publish(outToPy);
			n_corr+=corr;
		}
		descriptorPFH_anterior = descriptorPFH_actual;
	}
	else if (strcmp(feature_type,"FPFH")==0)
	{
		std::cout << "Calculo feature FPFH" << std::endl;
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr descriptorFPFH_actual(new pcl::PointCloud<pcl::FPFHSignature33>());
		if (strcmp(keypoints_type,"0")==0)
		{
			descriptorFPFH_actual = FPFH(cloud_filtered3, cloud_normal);
		}
		else
		{
			descriptorFPFH_actual = FPFH(cloud_filtered3, cloud_normal, indices_KP);
		}
		if (keypoints_anterior->points.size() > 0)
		{
			std::cout << "Calculo correspondencia FPFH" << std::endl;
			int corr=correspondences_FPFH(descriptorFPHF_anterior, descriptorFPFH_actual, keypoints_anterior, cloud_KP)->size();
			std_msgs::Float32 outToPy;
			outToPy.data = (float)(corr);
			pubToPy.publish(outToPy);
			n_corr+=corr;
		}
		descriptorFPHF_anterior = descriptorFPFH_actual;
	}
	else if (strcmp(feature_type,"0")==0 && strcmp(keypoints_type,"KP")==0);
	else
	{
		std::cout << "[ERROR] Feature no detectado" << std::endl;
		std::cout << "Introduzca una de las siguientes opciones como segundo o único parámetro de la funcion: PFH, FPFH, VFH" << std::endl;
	}
	keypoints_anterior = cloud_KP;


/********************** PUBLICACION NUBES DE PUNTOS PARA SU VISUALIZACION (DESARROLLO) ***********************************/
	// Preparo como mensajes la nube filtrada y los KeyPoints que quiero visualizar
   	sensor_msgs::PointCloud2 outputF;
   	pcl::toROSMsg(*cloud_filtered3, outputF);
	outputF.header.frame_id = "/velodyne";

   	sensor_msgs::PointCloud2 outputKP;
   	pcl::toROSMsg(*cloud_KP, outputKP);
	outputKP.header.frame_id = "/velodyne";

	// Publish the data
	pubF.publish (outputF);
	pubKP.publish (outputKP);

	std::cout << "Numero de mensajes procesados: " << i << std::endl << std::endl;
	if(strcmp(feature_type,"0")==0 && strcmp(keypoints_type,"KP")==0){
		if(i>0){
			std::cout << "Numero medio de KeyPoints por scan: " << std::endl;
			std::cout << "  Harris 3D: " << (float)all_kp[0]/(float)i << "  ISS 3D: " << (float)all_kp[1]/(float)i << "  Sift Z: " << (float)all_kp[2]/(float)i << "  Sift NE: " << (float)all_kp[3]/(float)i << std::endl << std::endl;
		}
	}
	else{
		if(i>1)	std::cout << "Numero medio de correspondencias: " << ((float)n_corr/(float)(i-1)) << std::endl << std::endl;
		if(strcmp(keypoints_type,"0")!=0 && i>0) std::cout << "Numero medio de KP: " << (float)n_kp/(float)i << std::endl << std::endl;
	}
}

/*************** FUNCIONES DE DETERMINACION DE METODOS ELEGIDOS ***********************/
void pfh(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
	char kp[] = "0";
	char feature[] = "PFH";
	cloud_cb(cloud_msg, kp, feature);
}
void fpfh(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
	char kp[] = "0";
	char feature[] = "FPFH";
	cloud_cb(cloud_msg, kp, feature);
}
void kph_pfh(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
	char kp[] = "KPH";
	char feature[] = "PFH";
	cloud_cb(cloud_msg, kp, feature);
}
void kph_fpfh(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
	char kp[] = "KPH";
	char feature[] = "FPFH";
	cloud_cb(cloud_msg, kp, feature);
}
void kpiss_pfh(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
	char kp[] = "KPISS";
	char feature[] = "PFH";
	cloud_cb(cloud_msg, kp, feature);
}
void kpiss_fpfh(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
	char kp[] = "KPISS";
	char feature[] = "FPFH";
	cloud_cb(cloud_msg, kp, feature);
}
void all_kps(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
	char kp[] = "KP";
	char feature[] = "0";
	cloud_cb(cloud_msg, kp, feature);
}

int main (int argc, char** argv)
{
	// Initialize the ROS Node "roscpp_pcl_depurado"
	ros::init (argc, argv, "roscpp_pcl_depurado");
	ros::NodeHandle nh;
	// Print "Hello" message with node name to the terminal and ROS log file
	ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());
	std::cout << std::endl;
	std::cout << std::endl;

/************* ELECCION DEL METODO DE KEYPOINTS Y FEATURE SEGUN PARAMETROS DE ENTRADA *****************/
	if (argc > 1){
		if (strcmp(argv[1],"KPH")==0 && strcmp(argv[2],"PFH")==0)
			sub = nh.subscribe(IMAGE_TOPIC, COLA_RECEPCION, kph_pfh);
		else if (strcmp(argv[1],"KPH")==0 && strcmp(argv[2],"FPFH")==0)
			sub = nh.subscribe(IMAGE_TOPIC, COLA_RECEPCION, kph_fpfh);
		else if (strcmp(argv[1],"KPH")==0 && argc == 2)
			sub = nh.subscribe(IMAGE_TOPIC, COLA_RECEPCION, kph_fpfh);
		else if (strcmp(argv[1],"KPISS")==0 && strcmp(argv[2],"PFH")==0)
			sub = nh.subscribe(IMAGE_TOPIC, COLA_RECEPCION, kpiss_pfh);
		else if (strcmp(argv[1],"KPISS")==0 && strcmp(argv[2],"FPFH")==0)
			sub = nh.subscribe(IMAGE_TOPIC, COLA_RECEPCION, kpiss_fpfh);
		else if (strcmp(argv[1],"KPISS")==0 && argc == 2)
			sub = nh.subscribe(IMAGE_TOPIC, COLA_RECEPCION, kpiss_fpfh);
		else if (strcmp(argv[1],"PFH")==0)
			sub = nh.subscribe(IMAGE_TOPIC, COLA_RECEPCION, pfh);
		else if (strcmp(argv[1],"FPFH")==0)
			sub = nh.subscribe(IMAGE_TOPIC, COLA_RECEPCION, fpfh);
		else if (strcmp(argv[1],"KP")==0 && argc == 2)
			sub = nh.subscribe(IMAGE_TOPIC, COLA_RECEPCION, all_kps);
		else
		{
			std::cout << "[ERROR]: Metodo no encontrado: roscpp_pcl_depurado (keypoint) (feature)" << std::endl;
			std::cout << "keypoint (opcional): Harris -> KPH, ISS3D -> KPISS" << std::endl;
			std::cout << "feature: PFH, FPFH" << std::endl;
			std::cout << "Si introduzca como único parámetro 'KP' para realizar una comparación entre métodos de KeyPoints" << std::endl;
			return 0;
		}
	}
	else
	{
		std::cout << "[ERROR]: Metodo no encontrado: roscpp_pcl_depurado (keypoint) (feature)" << std::endl;
		std::cout << "keypoint (opcional): Harris -> KPH, ISS3D -> KPISS" << std::endl;
		std::cout << "feature: PFH, FPFH" << std::endl;
		std::cout << "Si introduzca como único parámetro 'KP' para realizar una comparación entre métodos de KeyPoints" << std::endl;
		return 0;
	}

	// Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
	pubF = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);
	pubKP = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC2, 1);
	pubToPy = nh.advertise<std_msgs::Float32>(PUBLISH_TOPIC_PY, 1);

	// Spin
	ros::spin();

	// Success
	return 0;
}
