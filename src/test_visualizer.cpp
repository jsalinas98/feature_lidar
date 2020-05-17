#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

int main(){
  for (float z(-1.0); z <= 1.0; z += 0.05)
  {
    for (float angle(0.0); angle <= 6.28; angle += 1.0)
    {
      pcl::PointXYZ basic_point;
      basic_point.x = 0.5 * cosf (angle);
      basic_point.y = sinf (angle);
      basic_point.z = z;
      basic_cloud_ptr->points.push_back(basic_point);
    }
  }
  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;

 boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->addPointCloud<pcl::PointXYZ> (basic_cloud_ptr , "sample cloud");
return 0;
}
