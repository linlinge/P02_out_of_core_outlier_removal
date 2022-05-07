#pragma once
#include <string>
#include <Eigen/Core>
#include <iostream>
using namespace std;
// downsampling 
#define TEST_RUNTIME_DOWNSAMPLING 1
#define EXTRACT_V_NEGTIVE 0

// grid clustering mode (0: save with truncate result 1: save with random color)
#define GRID_CLUSTERING_MODE 0

/* segmentation */
#define TEST_RUNTIME_SEGMENTATION 1
#define SEGMENTATION_EXTRACT_PATCH 1
// select only one
#define SEGMENTATION_MODE 3
#define SEGMENTATION_GET_RAW_PART_ON (SEGMENTATION_MODE==1)
#define SEGMENTATION_GET_LOW_INDICES_ON (SEGMENTATION_MODE==2)
#define SEGMENTATION_GET_COLOR_LOW_CLOUD_ON (SEGMENTATION_MODE==3)
#define SEGMENTATION_GET_LOW_PART_ON 0

// indexing
#define TEST_RUNTIME_INDEXING 1

// save mode
#define COLOR_ON 1

extern int pid;
extern string path_to_ht;
void PrintVector3f(Eigen::Vector3f& dat,string seperator=" ");
