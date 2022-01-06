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
	static const int M_nano = 3;             //������Ϣ��x,y,localizationError
	int r_imgRawSize;                        //��ö�λ���ԭʼ�ɼ�ͼ����Ԫ�ߴ�
	int r_renderdPixelSize;			         //����ͼ����Ԫ�ߴ�
	int r_gaussianKernalSize;                //��˹�˴�С
	int addEdge = r_gaussianKernalSize + 4;  //��֤��ʹ��Ե��Ҳ��������������˹��
	Result r_table;                          //��λ����Ϣ

public:
	ImageRender(Result table, int imgRawSize, int renderdPixelSize, int gaussianKernalSize);
	void Render();
};

__global__ static void GaussKernal(uchar* Image, float* Table, int ROISize, int N, int Width);

#endif