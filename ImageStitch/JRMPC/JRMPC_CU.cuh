#pragma once

#ifndef _JRMPC_CU_CUH_
#define _JRMPC_CU_CUH_
#define EIGEN_USE_MKL_ALL//Using MKL for Eigen

#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <algorithm>
#include <Eigen/Core>

#include "cuda_runtime.h"
#include "device_launch_parameters.h"
#include "..//ReaderWriterIO//point_RW.cpp"

using namespace Eigen;

class JRMPC
{
public:
	void setInputPoint(struct Result *Data_1, struct Result *Data_2);
	void edgeArea_extraction(int Edge, int Flag);
	//void preProcess();Ԥ��������ټ�
	MatrixXf jrmpcMethod();

public:
	Result PointData_1;//Storge point Datas
	Result PointData_2;
	MatrixXf Tans;

private:
	static const int M_Nano = 3;//��ȡ���Ƶ��������Ϣά��
	static const int K = 300;//�������ĵ�����
	const int maxNumIter = 15;//��������

};

__global__ static void MatrixEularDis(float *P_1, float *C, float *A_1, int N, int K);


#endif