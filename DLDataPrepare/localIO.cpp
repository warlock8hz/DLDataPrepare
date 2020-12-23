#include "localIO.h"
#include <fstream>

size_t LoadTxtSpaceFile(std::string strPathName, 
	std::vector<std::string>& vstrBuf)
{
	std::ifstream ifsFile(strPathName.c_str());
	std::string strLine;
	while (std::getline(ifsFile, strLine))
		vstrBuf.push_back(strLine);

	// close when finished 
	ifsFile.close();
	return vstrBuf.size();
}

int WriteTxtCommaFile(std::string strPathName,
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCld)
{
	std::ofstream ofFile;
	ofFile.open(strPathName, std::ios::out | std::ios::app);
	//ofFile << "1,2,3,4,5,6";

	size_t loopNum = ptrCld->points.size();
	for (size_t si = 0; si < loopNum; si++)
	{
		ofFile << std::fixed << std::setprecision(6) << ptrCld->points.at(si).x << ",";
		ofFile << std::fixed << std::setprecision(6) << ptrCld->points.at(si).y << ",";
		ofFile << std::fixed << std::setprecision(6) << ptrCld->points.at(si).z << ",";
		// fake normal part 
		ofFile << "0,0,0\n";
	}

	ofFile.close();
	return 1;
}
bool Line2Pt3d(std::string strLine, pcl::PointXYZ&ptXYZ)
{
	size_t sCurPos = 0;
	size_t sLastPos = 0;
	std::string strLook4 = " ";
	std::string strAvalue = "";
	sCurPos = strLine.find(strLook4);
	if (sCurPos == std::string::npos)
		return false; // no comma at all

	std::vector<double>vdBuf; vdBuf.reserve(3); 
	// first value 
	for (size_t si = 0; si < 2; si++)
	{
		strAvalue = strLine.substr(sLastPos, sCurPos - sLastPos);
		remove_if(strAvalue.begin(), strAvalue.end(), isspace);
		vdBuf.push_back(stod(strAvalue));
		sLastPos = sCurPos + 1; // skip the comma
		sCurPos = strLine.find(strLook4, sLastPos);

		if (sCurPos == std::string::npos)
		{
			if (si < 1)
				return false;
			break; // no comma at all
		}
	}
	strAvalue = strLine.substr(sLastPos, sCurPos - sLastPos);
	remove_if(strAvalue.begin(), strAvalue.end(), isspace);
	vdBuf.push_back(stod(strAvalue));

	ptXYZ.x = vdBuf[0]; 
	ptXYZ.y = vdBuf[1]; 
	ptXYZ.z = vdBuf[2]; 

	return true;
}

size_t LineBuf2Ptr(std::vector<std::string>& vstrBuf,
	pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCld)
{
	pcl::PointXYZ ptNewPt(0.0, 0.0, 0.0); 
	size_t sMax = vstrBuf.size(); 
	for (size_t si = 0; si < sMax; si++)
	{
		if (Line2Pt3d(vstrBuf.at(si), ptNewPt))
			ptrCld->points.push_back(ptNewPt); 
	}
	return ptrCld->points.size(); 
}

size_t GetAllFiles(std::string strInputPath, std::string strOutputPath,
	std::vector<std::string>& vstrFileNameList,
	std::vector<std::string>& vstrOutputPathNameList)
{
	vstrFileNameList.clear();
	vstrOutputPathNameList.clear(); 

	std::string strTxtExt = "txt";
	std::string strSlash = "\\";
	std::string strPath = strInputPath + strSlash; //strTemp;

	boost::filesystem::path p(strInputPath);
	for (auto i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++)
	{
		if (!is_directory(i->path())) //we eliminate directories
		{
			//std::cout << i->path().filename().string() << std::endl; 
			std::string strFileName = i->path().filename().string();
			std::string strFileExt(strFileName, strFileName.size() - 3, 3);
			if (boost::iequals(strFileExt, strTxtExt))
			{
				std::string strPathName = strInputPath + strSlash + strFileName;
				//std::string strOutputFolderName = strFileName.substr(0, strFileName.size() - 4);
				vstrFileNameList.push_back(strFileName);
				vstrOutputPathNameList.push_back(strOutputPath + strSlash + strFileName);
			}
		}
		else
			continue;
	}

	return vstrFileNameList.size();
}
