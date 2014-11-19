#pragma once
#include "stdafx.h"
//#include <pcl/common/time.h>
#include "ChangeDetector.h"
#include "DataManager.h"

//main processor class
class LitemDetector {
public:
	LitemDetector() {}
	void run() {
	/*	static unsigned count = 0;
		static double last = pcl::getTime ();
		double now = pcl::getTime ();
	*/	ChangeDetector changeDetector;
		CloudProcessor processor;
		//processor.registerProcessor(&changeDetector);
		processor.run();
	}
	
};
