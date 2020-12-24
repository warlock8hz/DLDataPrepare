#pragma once
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

void CentralizePtr(pcl::PointCloud<pcl::PointXYZ>::Ptr ptrTheCld); 

void GetNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr ptrTheCld,
	pcl::PointCloud<pcl::Normal>::Ptr ptrTheNormal); 

void Combine2Larger(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& vptrInput,
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& vptrOutput); 

void GenrateRandomSubset(pcl::PointCloud<pcl::PointXYZ>::Ptr ptrRaw,
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& vptrRandSubset,
	size_t sHowMany, const size_t sSize);

void GenerateRandomIndices(const size_t sHowMany, std::vector<size_t>& vsIndices); 