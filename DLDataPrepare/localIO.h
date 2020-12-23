#pragma once
#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

size_t LoadTxtSpaceFile(std::string strPathName,
	std::vector<std::string>& vstrBuf); 
int WriteTxtCommaFile(std::string strPathName,
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCld);

bool Line2Pt3d(std::string strLine, pcl::PointXYZ& ptXYZ);
size_t LineBuf2Ptr(std::vector<std::string>& vstrBuf,
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCld); 

size_t GetAllFiles(std::string strInputPath, std::string strOutputPath,
	std::vector<std::string>& vstrFileNameList,
	std::vector<std::string>& vstrOutputPathNameList); 