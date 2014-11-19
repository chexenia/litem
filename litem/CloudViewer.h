#pragma once
#include "stdafx.h"
#include <pcl/visualization/pcl_visualizer.h>

class CloudViewer {
private:
	std::string view_name;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
public:
	CloudViewer(std::string name) : view_name(name) {
		viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("3D Viewer"));
	};

	int view(const LITEM_CLOUD::ConstPtr & cloud) {
		viewer->removePointCloud();
		pcl::visualization::PointCloudColorHandlerRGBField<LITEM_POINT> rgb(cloud);
//		pcl::visualization::PointCloudColorHandlerGenericField<LITEM_POINT> single_color(cloud, "intensity");
//		viewer->addPointCloud<LITEM_POINT>(cloud, single_color, view_name);
		viewer->addPointCloud<LITEM_POINT>(cloud, rgb, view_name);
		viewer->setBackgroundColor (0, 0, 0);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, view_name);
		viewer->addCoordinateSystem (1.0);
		viewer->initCameraParameters ();
		while (!viewer->wasStopped ()) {
		  viewer->spinOnce (100);
		  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}
		//viewer->spin();
		return 0;
	}
};