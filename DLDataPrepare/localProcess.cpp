#include "localProcess.h"
#include <algorithm>
#include <numeric>
#include <random>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/features/normal_3d_omp.h>
#include <pcl/common/transforms.h>

void CentralizePtr(pcl::PointCloud<pcl::PointXYZ>::Ptr ptrTheCld)
{
	Eigen::Vector4f v4fCentroid = Eigen::Vector4f::Zero(); 
	pcl::compute3DCentroid(*ptrTheCld, v4fCentroid); 

	size_t sMax = ptrTheCld->points.size(); 
	for (size_t si = 0; si < sMax; si++)
	{
		pcl::PointXYZ& ptThePt = ptrTheCld->points.at(si); 
		ptThePt.x = ptThePt.x - v4fCentroid[0];
		ptThePt.y = ptThePt.y - v4fCentroid[1];
		ptThePt.z = ptThePt.z - v4fCentroid[2];
	}
	return; 
}

void GetNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr ptrTheCld,
	pcl::PointCloud<pcl::Normal>::Ptr ptrTheNormal)
{
	/** this function requires the fix in pcl source code first 
	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(ptrTheCld);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	ne.setSearchMethod(tree);

	// Output datasets
	//pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	//ne.setRadiusSearch(0.03);
	ne.setKSearch(16); 

	// Compute the features
	ne.compute(*ptrTheNormal);
	*/
	return; 
}

void Combine2Larger(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& vptrInput,
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& vptrOutput)
{
	// combine the adjacent point clouds to make it larger
	return; 
}

void GenrateRandomSubset(pcl::PointCloud<pcl::PointXYZ>::Ptr ptrRaw,
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& vptrRandSubset, 
	size_t sHowMany, const size_t sSize)
{
	std::cout << "Generate " << sHowMany << " subsets from " << ptrRaw->points.size() << "points. \n"; 
	
	// generate raw index 
	std::vector<int>viIndexRaw(ptrRaw->points.size()); 
	std::iota(std::begin(viIndexRaw), std::end(viIndexRaw), 0); 

	std::random_device rd; 
	std::mt19937 g(rd()); 
	pcl::ExtractIndices<pcl::PointXYZ> extract; 
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices()); 
	std::vector<int> viLoopIndex; 
	Eigen::Matrix4f m4fRotate = Eigen::Matrix4f::Identity(); 
	Eigen::Affine3f a3fRotate; 

	for (size_t si = 0; si < sHowMany; si++)
	{
		viLoopIndex = viIndexRaw; 
		std::shuffle(viIndexRaw.begin(), viIndexRaw.end(), g); 
		viLoopIndex.resize(sSize);
		inliers->indices = viLoopIndex; 

		pcl::PointCloud<pcl::PointXYZ>::Ptr ptrNew(new pcl::PointCloud<pcl::PointXYZ>); 

		extract.setInputCloud(ptrRaw); 
		extract.setIndices(inliers);
		extract.setNegative(false); 
		extract.filter(*ptrNew); 

		// rotate a random angle
		pcl::getTransformation(0.0f, 0.0f, 0.0f,
			(float)rand() / (float)RAND_MAX * M_PI * 2, 
			(float)rand() / (float)RAND_MAX * M_PI * 2,
			(float)rand() / (float)RAND_MAX * M_PI * 2,
			a3fRotate);
		m4fRotate = a3fRotate.matrix(); 

		pcl::transformPointCloud(*ptrNew, *ptrNew, m4fRotate);

		vptrRandSubset.push_back(ptrNew); 
	}
}

void GenerateRandomIndices(const size_t sHowMany, std::vector<size_t>& vsIndices)
{
	vsIndices = std::vector<size_t>(sHowMany);
	std::iota(std::begin(vsIndices), std::end(vsIndices), 0);

	std::random_device rd;
	std::mt19937 g(rd());
	std::shuffle(vsIndices.begin(), vsIndices.end(), g);
}