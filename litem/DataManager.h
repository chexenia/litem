/*
	Cloud data loading, storing and converting
*/
#pragma once 
#include "stdafx.h"
#include <pcl/io/openni_grabber.h>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include "CloudViewer.h"
#include "pcl/io/oni_grabber.h"

using namespace boost::filesystem;


class DataManager {
protected:
	//number of read files
	int n;
public:
	DataManager(){}
	virtual ~DataManager(){}
	virtual int init(string str) = 0;
	virtual int nextCloud(LITEM_CLOUD::Ptr cloud) = 0;
};

class CloudLoader : public DataManager {
private:
	vector<string> clouds;
public:
	CloudLoader() {}
	int init(string str) {
		if(!exists(str)) {
			cout << "No "+str+" exists";
			return -1;
		}
		//fill in the names of point clouds in folder
		path dir_root(str);
		directory_iterator end_itr; // default construction yields past-the-end
		for ( directory_iterator itr( dir_root ); itr != end_itr; ++itr ) {
			if(itr->path().has_filename()) {
				string cloud = canonical(itr->path()).string();
				string format = cloud.substr(cloud.size()-2, 3);
				if(format.compare("pcd")) {
					clouds.push_back(cloud);
				}
			}
		} 
		n = 0;
		return 0;
	}
	int nextCloud(LITEM_CLOUD::Ptr cloud) {
		if(n<clouds.size()) {
			pcl::io::loadPCDFile(clouds[n++], *cloud);
			return 0;
		}
		return -1;
	}
};

class CloudGrabber : public DataManager {
public:
	CloudGrabber() {}
	int nextCloud(LITEM_CLOUD::Ptr cloud) {
		return -1;
	}

	int init(string str) {
		return -1;
	}
};


//Stream cloud data from .oni files into point cloud
class LitemGrabber {
private:
	string oni_name;
	CloudViewer * viewer;
	void cloud_cb_ (const LITEM_CLOUD::ConstPtr & cloud) {
		viewer->view(cloud);
	}
public:
	LitemGrabber(string oni_name) : oni_name(oni_name) {
		viewer = new CloudViewer("Oni grabber");
	}
	void run() {
		 	// make callback function from member function
		boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&LitemGrabber::cloud_cb_, this, _1);

		// create a new grabber for OpenNI devices
		pcl::ONIGrabber* interface = new pcl::ONIGrabber(oni_name, true, true);

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