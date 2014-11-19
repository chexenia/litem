// stdafx.h : include file for standard system include files,
// or project specific include files that are used frequently, but
// are changed infrequently
//

#pragma once

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>

//#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files:
//#include <windows.h>

//GLOG logging
//#define GLOG_NO_ABBREVIATED_SEVERITIES
//undef conflicting WINDOWS ERROR macros
//#undef ERROR
//#include "..glog\logging.h"


//PCL includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace std;

#define LITEM_GRAB 234
#define LITEM_LOAD 578

//typedef pcl::PointXYZI LITEM_POINT;
typedef pcl::PointXYZRGBA LITEM_POINT;
typedef pcl::PointCloud<LITEM_POINT> LITEM_CLOUD;