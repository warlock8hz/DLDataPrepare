// DLDataPrepare.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>

#include "localIO.h"
#include "localProcess.h"
#include <pcl/visualization/pcl_visualizer.h>

//int main(int argc, char* argv[])
int FormatChangeOnly(std::string strInput, std::string strOutput)
{
	//std::string strInput = argv[1];
	//std::string strOutput = argv[2];

	std::vector<std::string>vstrFileNameList; // name only 
	std::vector<std::string>vstrFilePathNameList; // path and name 
	std::vector<std::string>vstrLineBuf; vstrLineBuf.reserve(5000); 

	size_t sNum = GetAllFiles(0, strInput, strOutput, vstrFileNameList, vstrFilePathNameList); 

	// load all 
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vptrClds; vptrClds.reserve(sNum);
	std::cout << "loading files..."; 
	for (size_t si = 0; si < sNum; si++)
	{
		std::cout << " " << si; 
		pcl::PointCloud<pcl::PointXYZ>::Ptr ptrNew(new pcl::PointCloud<pcl::PointXYZ>);
		LoadTxtSpaceFile(strInput + "\\" + vstrFileNameList.at(si), vstrLineBuf); 
		size_t sCldSize = LineBuf2Ptr(vstrLineBuf, ptrNew); 
		vstrLineBuf.clear(); 
		vptrClds.push_back(ptrNew); 
	}
	std::cout << "\nProcessing... ";

	// centralize and others 
	for (size_t si = 0; si < vptrClds.size(); si++)
	{
		std::cout << " " << si;
		CentralizePtr(vptrClds.at(si));
		// if needed, get normal here 
		// GetNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr ptrTheCld, pcl::PointCloud<pcl::Normal>::Ptr ptrTheNormal);
	}

	// duplicate for randomize 
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vptrRandClds; vptrClds.reserve(vptrClds.size());
	std::copy(vptrClds.begin(), vptrClds.end(), back_inserter(vptrRandClds)); 

	// initialize visualization 
		// visualization component 
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer
		(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0.5, 0.5, 0.5);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptrVisualizer(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> normal_color(ptrVisualizer, 0, 0, 0);
	viewer->removeAllPointClouds();
	viewer->addPointCloud<pcl::PointXYZ>(ptrVisualizer, normal_color, "Positions"); //(accumAlignedClouds_UNorg_PN, visColorSource, "accumAlignedClouds_UNorg_PN");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Positions");
	viewer->resetCamera();
	viewer->spinOnce(10);

	size_t sSqId = 0; 
	// randomize
	std::cout << "\nExporting... ";
	while (vptrRandClds.size() > 0)
	{
		std::cout << " " << sSqId;
		size_t sRandNum = vptrRandClds.size();
		size_t sId = rand() % sRandNum; 

		// visualize it 
		if (true)
		{
			*ptrVisualizer = *vptrRandClds.at(sId); 
			viewer->removeAllPointClouds();
			viewer->addPointCloud<pcl::PointXYZ>(ptrVisualizer, normal_color, "Positions"); //(accumAlignedClouds_UNorg_PN, visColorSource, "accumAlignedClouds_UNorg_PN");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Positions");
			viewer->resetCamera();
			viewer->spinOnce(10);
		}

		WriteTxtCommaFile(vstrFilePathNameList.at(sSqId), vptrRandClds.at(sId)); 

		sSqId++; 
		vptrRandClds.erase(vptrRandClds.begin() + sId); 
	}
	std::cout << "\nsome cleaning...\n ";

	// clean the RAM 
	for (size_t si = 0; si < sNum; si++)
	{
		vptrClds.at(si)->clear(); 
		vptrClds.at(si).reset(new pcl::PointCloud<pcl::PointXYZ>); 
	}
	std::cout << "\nfinished\n ";

	return 1; 
}

int main(int argc, char* argv[])
{
	std::string strInput = argv[1];
	std::string strOutput = argv[2];
	size_t sHowMany = 500; 
	size_t sSize = 10000; 

	std::vector<std::string>vstrFileNameList; // name only 
	std::vector<std::string>vstrFilePathNameList; // path and name 
	std::vector<std::string>vstrLineBuf; vstrLineBuf.reserve(5000);

	size_t sNum = GetAllFiles(sHowMany, strInput, strOutput, vstrFileNameList, vstrFilePathNameList);

	// load all 
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vptrClds; vptrClds.reserve(sNum);
	std::cout << "loading files...";
	for (size_t si = 0; si < sNum; si++)
	{
		std::cout << " " << si;
		pcl::PointCloud<pcl::PointXYZ>::Ptr ptrNew(new pcl::PointCloud<pcl::PointXYZ>);

		if (pcl::io::loadPCDFile<pcl::PointXYZ>(strInput + "\\" + vstrFileNameList.at(si), 
			*ptrNew) == -1) //* load the file
		{
			PCL_ERROR("Couldn't read file test_pcd.pcd \n");
			return (-1);
		}

		vptrClds.push_back(ptrNew);
	}
	std::cout << "\nProcessing... \n";

	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> vptrRandSubset; 
	vptrRandSubset.reserve(sHowMany);

	// centralize and others 
	for (size_t si = 0; si < vptrClds.size(); si++)
	{
		std::cout << " " << si;
		CentralizePtr(vptrClds.at(si));
		// if needed, get normal here 
		// GetNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr ptrTheCld, pcl::PointCloud<pcl::Normal>::Ptr ptrTheNormal);
	
		GenrateRandomSubset(vptrClds.at(si), vptrRandSubset, sHowMany, sSize); 
	}

	// initialize visualization 
		// visualization component 
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer
	(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0.5, 0.5, 0.5);
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptrVisualizer(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> normal_color(ptrVisualizer, 0, 0, 0);
	viewer->removeAllPointClouds();
	viewer->addPointCloud<pcl::PointXYZ>(ptrVisualizer, normal_color, "Positions"); //(accumAlignedClouds_UNorg_PN, visColorSource, "accumAlignedClouds_UNorg_PN");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Positions");
	viewer->resetCamera();
	viewer->spinOnce(10);

	size_t sSqId = 0;
	std::vector<size_t>vsRandomIndices; 
	GenerateRandomIndices(sHowMany * sNum, vsRandomIndices);
	// randomize
	std::cout << "\nExporting... ";
	while (vptrRandSubset.size() > 0)
	{
		// visualize it 
		if (true)
		{
			*ptrVisualizer = *vptrRandSubset.at(vsRandomIndices.at(sSqId));
			viewer->removeAllPointClouds();
			viewer->addPointCloud<pcl::PointXYZ>(ptrVisualizer, normal_color, "Positions"); //(accumAlignedClouds_UNorg_PN, visColorSource, "accumAlignedClouds_UNorg_PN");
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Positions");
			viewer->resetCamera();
			viewer->spinOnce(10);
		}

		WriteTxtCommaFile(vstrFilePathNameList.at(sSqId), vptrRandSubset.at(vsRandomIndices.at(sSqId)));

		sSqId++;
		if (sSqId == sHowMany * sNum)
			break; 
	}
	std::cout << "\nsome cleaning...\n ";

	// clean the RAM 
	for (size_t si = 0; si < sNum; si++)
	{
		vptrClds.at(si)->clear();
		vptrClds.at(si).reset(new pcl::PointCloud<pcl::PointXYZ>);
	}
	for (size_t si = 0; si < sHowMany * sNum; si++)
	{
		vptrRandSubset.at(si)->clear();
		vptrRandSubset.at(si).reset(new pcl::PointCloud<pcl::PointXYZ>);
	}
	std::cout << "\nfinished\n ";

	return 1; 
}


// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
