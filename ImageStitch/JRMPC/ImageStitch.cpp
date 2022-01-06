#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>
#include <Eigen/Core>

#include "JRMPC_CU.cuh"
#include "..//ReaderWriterIO//point_RW.cpp"
#include "..//ReaderWriterIO/ListFiles_x64.h"

using namespace std;

int main()
{
	//读取文件夹下文件
	// F://Data2_test//groundTRUTH           F://Data2_test//2
	string FilePath = "F://Data2_test//groundTRUTH";
	int image_width = 5;
	int image_length = 5;
	int EdgeSize = 60;
	vector<string> Files;
    getFiles(FilePath,Files);
	int Num = Files.size();
	point_RW R_Writer;

	int i = 0, j = 0;
	int FirstImage = j * image_width + i;
	int SecondImage = j * image_width + i + 1;
	Result imageFirst = R_Writer.point_Read(Files.at(FirstImage));
	Result imageSecond = R_Writer.point_Read(Files.at(SecondImage));
	JRMPC StitchClass_Row;
	StitchClass_Row.setInputPoint(&imageFirst, &imageSecond);
	StitchClass_Row.edgeArea_extraction(EdgeSize, 1);//设置为1表示截取横向区域，2表示纵向区域
	MatrixXf moveDis = StitchClass_Row.jrmpcMethod();
	printf("%f \n", moveDis(0, 0));
	printf("%f", moveDis(0, 1));
/*
	for (int i = 0; i < image_width; i++) {
		for (int j = 0; j < image_length; j++) {
			int FirstImage = j * image_width + i;
			if (i < image_width - 1) {
				//横向拼图
				int SecondImage = j * image_width + i + 1;
				Result imageFirst = R_Writer.point_Read(Files.at(FirstImage));
				Result imageSecond = R_Writer.point_Read(Files.at(SecondImage));
				JRMPC StitchClass_Row;
				StitchClass_Row.setInputPoint(&imageFirst, &imageSecond);
				StitchClass_Row.edgeArea_extraction(EdgeSize, 1);//设置为1表示截取横向区域，2表示纵向区域
				Matrix<float, 2, 2> T = StitchClass_Row.jrmpcMethod();
			}
			if (j < image_length - 1) {
				//纵向拼图
				int SecondImage = (j + 1) * image_width + i;
				Result imageFirst = R_Writer.point_Read(Files.at(FirstImage));
				Result imageSecond = R_Writer.point_Read(Files.at(SecondImage));
				JRMPC StitchClass_Row;
				StitchClass_Row.setInputPoint(&imageFirst, &imageSecond);
				StitchClass_Row.edgeArea_extraction(EdgeSize, 2);
				StitchClass_Row.jrmpcMethod();
				Matrix<float, 2, 2> T = StitchClass_Row.jrmpcMethod();
			}
		}
	}
*/




	//临接图像拼接

	//路径优化与大图拼接
	system("pause");
}
