#include <ros/ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <pcl/visualization/pcl_visualizer.h>

pcl::visualization::PCLVisualizer::Ptr Vis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
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
	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 10, 2, "normals");  //Muestra las lineas, (cloud, mormales, por cada x lineas muestra 1, long lineas, "")
	viewer->addCoordinateSystem (0.5); //Muestra los ejes
	viewer->initCameraParameters ();  //Inicia la vista en el origen
	return (viewer);
}

int main(int argc, char** argv)
{
	// Object for storing the point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Read a PCD file from disk.
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) != 0)
	{
		return -1;
	}

	pcl::visualization::PCLVisualizer::Ptr viewer;
    	viewer = Vis(cloud);
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
	}
	ros::spin();
}
