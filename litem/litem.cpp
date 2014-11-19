#include "stdafx.h"
#include "DataManager.h"
#include "Test.h"

int main () {

		//GLOG logging initialization
	//google::SetLogDestination(google::INFO, "../logs/info.log");
	//google::InitGoogleLogging("litem");

	//LitemDetector detector;
	//detector.run();
	LitemGrabber grabber("..data\\test1.oni");
	grabber.run();
	//SimpleOpenNIProcessor v;
	//v.run ();
	return (0);
}
