//pcl progressive morphological filter--
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/visualization/pcl_visualizer.h>

int
main (int argc, char** argv)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("ground detect viewer"));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_ground (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_noground (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointIndicesPtr ground (new pcl::PointIndices);

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZ> ("../data/faw_inroom_left.pcd", *cloud);

//  std::cerr << "Cloud before filtering: " << std::endl;
//  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
  pmf.setInputCloud (cloud);
  pmf.setMaxWindowSize (20);
  pmf.setSlope (1.0f);
  pmf.setInitialDistance (0.3f);//0.5
  pmf.setMaxDistance (0.6f);//3
  pmf.extract (ground->indices);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud (cloud);
  extract.setIndices (ground);
  extract.filter (*cloud_filtered_ground);

//  std::cerr << "Ground cloud after filtering: " << std::endl;
//  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
//  writer.write<pcl::PointXYZ> ("../data/faw_inroom_left_ground.pcd", *cloud_filtered_ground, false);

  // Extract non-ground returns
  extract.setNegative (true);
  extract.filter (*cloud_filtered_noground);

//  std::cerr << "Object cloud after filtering: " << std::endl;
//  std::cerr << *cloud_filtered << std::endl;

//  writer.write<pcl::PointXYZ> ("../data/faw_inroom_left_no_ground.pcd", *cloud_filtered_noground, false);

//visualization
/*
        if (!viewer->updatePointCloud<pcl::PointXYZ>(cloud, "original_cloud"))
    {
        viewer->addPointCloud<pcl::PointXYZ>(cloud, "original_cloud");
    }
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");
        viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 1.0F, 0.0F, 0.0F, "original_cloud");
*/
        viewer->addCoordinateSystem (1.0);

        if (!viewer->updatePointCloud<pcl::PointXYZ>(cloud_filtered_ground, "ground_cloud"))
    {
        viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered_ground, "ground_cloud");
    }
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "ground_cloud");
        viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0.0F, 1.0F, 0.0F, "ground_cloud");

        if (!viewer->updatePointCloud<pcl::PointXYZ>(cloud_filtered_noground, "noground_cloud"))
    {
        viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered_noground, "noground_cloud");
    }
    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "noground_cloud");
        viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_COLOR, 0.0F, 0.0F, 1.0F, "noground_cloud");


 while (!viewer->wasStopped ())
        {
                viewer->spinOnce (100);
                boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }

  return (0);
}
