#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Core>

#include "JRMPC_CU.cuh"
#include "..//ReaderWriterIO/point_RW.cpp"
#include "..//ReaderWriterIO//ListFiles_x64.h"

using namespace std;

int main()
{
	//读取文件夹下文件
	string FilePath = "F:\\Data2_test\\2";
	int image_width = 5;
	int image_length = 5;
	int EdgeSize = 60;
	vector<string> Files;
	getFiles(FilePath, Files);
	int Num = Files.size();
	point_RW R_Writer;


	int i = 0, j = 0;
	int FirstImage = j * image_width + i;
	int SecondImage = j * image_width + i + 1;
	Result imageFirst = R_Writer.point_Read(Files.at(FirstImage));
	Result imageSecond = R_Writer.point_Read(Files.at(SecondImage));
	
	Eigen::setNbThreads(6);
	int n = Eigen::nbThreads();
	int k = 8;
	


}