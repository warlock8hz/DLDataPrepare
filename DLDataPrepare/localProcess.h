#pragma once
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

void CentralizePtr(pcl::PointCloud<pcl::PointXYZ>::Ptr ptrTheCld); 

void GetNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr ptrTheCld,
	pcl::PointCloud<pcl::Normal>::Ptr ptrTheNormal); 