/*
	Detect changes between two point clouds subtracting one from another
*/
#pragma once
#include "stdafx.h"
#include "CloudProcessor.h"
#include <pcl/octree/octree.h>

class ChangeDetector : public CloudProcessorIface {
private:
	// Octree resolution - side length of octree voxels
	float resolution;

	//we store previous point cloud
	LITEM_CLOUD::Ptr prevCloud;

	//reference point cloud
	LITEM_CLOUD::Ptr refCloud;

public:				
	ChangeDetector(float resolution = 32.0f) : resolution(resolution) {
	}

	int process(const LITEM_CLOUD::Ptr & inCloud, LITEM_CLOUD::Ptr outCloud) {
		if(prevCloud==NULL) {
			prevCloud = inCloud;

			//store first cloud as reference cloud
			refCloud = inCloud;
		}
		// Instantiate octree-based point cloud change detection class
		pcl::octree::OctreePointCloudChangeDetector<LITEM_POINT> octree(resolution);
		
		// Add points from cloudA to octree
		octree.setInputCloud(prevCloud);
		octree.addPointsFromInputCloud();

		// Switch octree buffers: This resets octree but keeps previous tree structure in memory.
		octree.switchBuffers ();

		// Add points from cloudB to octree
		octree.setInputCloud (inCloud);
		octree.addPointsFromInputCloud ();

		std::vector<int> newPointIdxVector;

		// Get vector of point indices from octree voxels which did not exist in previous buffer
		octree.getPointIndicesFromNewVoxels (newPointIdxVector);

		//initialize output cloud
		for (size_t i = 0; i < newPointIdxVector.size (); ++i) {
			outCloud->points.push_back(inCloud->points[i]);			
		}

		return 0;
	}
};
