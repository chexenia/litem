/*------------------------------------------------------------------------------------------*\
  Test.h
  Testing routine.

  Copyright (C) 2013 Kseniya Cherenkova
\*------------------------------------------------------------------------------------------*/

#pragma once
#include "stdafx.h"
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/time.h>

void initCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) {
	//pcl::io::loadPCDFile ("test_cloud.pcd", cloud);
	uint8_t r(255), g(15), b(15);
  for (float z(-1.0); z <= 1.0; z += 0.05)
  {
    for (float angle(0.0); angle <= 360.0; angle += 5.0)
    {
      pcl::PointXYZRGB basic_point;
      basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
      basic_point.y = sinf (pcl::deg2rad(angle));
      basic_point.z = z;
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      basic_point.rgb = *reinterpret_cast<float*>(&rgb);
      cloud->points.push_back(basic_point);
    }
    if (z < 0.0)
    {
      r -= 12;
      g += 12;
    }
    else
    {
      g -= 12;
      b += 12;
    }
  }
  cloud->width = (int) cloud->points.size ();
  cloud->height = 1;
}


int pclTest () {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	initCloud(cloud);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	while (!viewer->wasStopped ())
	{
	  viewer->spinOnce (100);
	  boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	return 0;
}


class SimpleOpenNIProcessor
{
public:
  void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
  {
    static unsigned count = 0;
    static double last = pcl::getTime ();
    if (++count == 30)
    {
      double now = pcl::getTime ();
      std::cout << "distance of center pixel :" << cloud->points [(cloud->width >> 1) * (cloud->height + 1)].z << " mm. Average framerate: " << double(count)/double(now - last) << " Hz" <<  std::endl;
      count = 0;
      last = now;
    }
  }
  
  void run () {
    // create a new grabber for OpenNI devices
    pcl::Grabber* interface = new pcl::OpenNIGrabber();

    // make callback function from member function
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
      boost::bind (&SimpleOpenNIProcessor::cloud_cb_, this, _1);

    // connect callback function for desired signal. In this case its a point cloud with color values
    boost::signals2::connection c = interface->registerCallback (f);

    // start receiving point clouds
    interface->start ();

    // wait until user quits program with Ctrl-C, but no busy-waiting -> sleep (1);
    while (true)
      boost::this_thread::sleep (boost::posix_time::seconds (1));

    // stop the grabber
    interface->stop ();
  }
};

