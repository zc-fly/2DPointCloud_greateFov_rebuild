#pragma once

#ifndef _IMAGERENDER_CUH_
#define _IMAGERENDER_CUH_

#include <iostream>
#include "opencv.hpp"
#include "..//ReaderWriterIO//point_RW.cpp"
#include "cuda_runtime.h"
#include <opencv2/highgui/highgui_c.h>
#include "device_launch_parameters.h"

using namespace std;

class ImageRender
{
private:
	static const int M_nano = 3;             //三种信息，x,y,localizationError
	int r_imgRawSize;                        //获得定位表的原始采集图像像元尺寸
	int r_renderdPixelSize;			         //生成图像像元尺寸
	int r_gaussianKernalSize;                //高斯核大小
	int addEdge = r_gaussianKernalSize + 4;  //保证即使边缘点也可以生成完整高斯核
	Result r_table;                          //定位点信息

public:
	ImageRender(Result table, int imgRawSize, int renderdPixelSize, int gaussianKernalSize);
	void Render();
};

__global__ static void GaussKernal(uchar* Image, float* Table, int ROISize, int N, int Width);

#endif