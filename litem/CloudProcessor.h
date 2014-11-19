/*
	Processor interface
*/
#pragma once
#include "stdafx.h"
#include "DataManager.h"
#include "CloudViewer.h"

class CloudProcessorIface {
protected:
	bool displayOut;
public:
	virtual int process(const LITEM_CLOUD::Ptr & inCloud, LITEM_CLOUD::Ptr outCloud) = 0;
};

class CloudProcessor {
private:
	//the processing queue
	std::vector<CloudProcessorIface*> processorQueue;
	DataManager * dataManager;
	CloudViewer * viewer;

public:
	CloudProcessor(bool displayOut = true) {
		if(displayOut) {
			viewer = new CloudViewer("litem");
		}
		dataManager = new CloudLoader();
		if(!dataManager->init("..\\data\\PCD")) {
		};

	}

	void registerProcessor(CloudProcessorIface * processor) {
		processorQueue.push_back(processor);
	}

	//int readNextCloudGen(LITEM_CLOUD::Ptr cloud) {
	//	uint8_t r(255), g(15), b(15);
	//	// Generate pointcloud data for cloudA
	//	for (size_t i = 0; i < 1024; ++i) {
	//		LITEM_POINT pt;
	//		pt.x = 64.0f * rand () / (RAND_MAX + 1.0f);
	//		pt.y = 64.0f * rand () / (RAND_MAX + 1.0f);
	//		pt.z = 64.0f * rand () / (RAND_MAX + 1.0f);
	//		uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
	//		pt.rgb = *reinterpret_cast<float*>(&rgb);
	//		//pt.a = .5;
	//		cloud->points.push_back(pt);
	//	}
	//	cloud->width = (int) cloud->points.size ();
	//	cloud->height = 1;
	//	return 0;
	//}

	//read the new cloud provided by DataManager
	int readNextCloud(LITEM_CLOUD::Ptr cloud) {
		return dataManager->nextCloud(cloud);
	}

	void run() {
		bool stopProcess = false;
		LITEM_CLOUD::Ptr out (new LITEM_CLOUD);
		LITEM_CLOUD::Ptr in (new LITEM_CLOUD);

		while(!stopProcess) {
			if(readNextCloud(in)==0) {
				//display input cloud
				if(viewer) {
					viewer->view(in);
				}
				if (!processorQueue.empty())  {
					std::vector<CloudProcessorIface*>::iterator it;
					for(it = processorQueue.begin(); it != processorQueue.end(); it++) {
						int response = (*it)->process(in, out);
						if(response == -1) {
							stopProcess = true;
						}
						//display processed cloud
						if(viewer) {
							viewer->view(out);
						}
					}
				} 
			}
			else {
				stopProcess = true;
			}
		}
	}

};
