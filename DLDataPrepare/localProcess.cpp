#include "localProcess.h"
#include <pcl/common/centroid.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/features/normal_3d_omp.h>

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