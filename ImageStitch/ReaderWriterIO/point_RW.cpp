#pragma once

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <vector>

using namespace std;

//�洢��ȡ�ĵ�����Ϣ
struct Result {
	float* pointData;
	int Num;
};

class point_RW{

public:
	//��ȡM=12ά���Ƴ�3ά����
	Result point_Read(string pathName) {
		ifstream fin1(pathName, ios::binary);
		assert(fin1);
		fin1.seekg(0, ios::end);
		const int N = fin1.tellg() / (M * sizeof(float));
		float* temp = new float[M*N];
		float* temp_1 = new float[M_Nano*N];
		fin1.seekg(0, ios::beg);
		fin1.read((char*)temp, M*N * sizeof(float));
		fin1.close();
		for (int i = 0; i < N; i++) {
			temp_1[M_Nano*i] = temp[M*i + 1];
			temp_1[M_Nano*i + 1] = temp[M*i + 2];
			temp_1[M_Nano*i + 2] = temp[M*i + 4];//PSF/pixel
		}
		delete temp;
		Result result;
		result.pointData = temp_1;
		result.Num = N;
		return result;
	};

	//��ȡ3ά����
	Result point_Read_3(string pathName) {
		ifstream fin1(pathName, ios::binary);
		assert(fin1);
		fin1.seekg(0, ios::end);
		const int N = fin1.tellg() / (M_Nano * sizeof(float));
		float* temp = new float[M_Nano*N];
		fin1.seekg(0, ios::beg);
		fin1.read((char*)temp, M_Nano*N * sizeof(float));
		fin1.close();
		Result result;
		result.pointData = temp;
		result.Num = N;
		return result;
	};

	void point_Write(Result result, string totalFile) {
		ofstream fin2(totalFile, ios::binary | ios::app);
		fin2.write((char*)result.pointData, sizeof(float)*M_Nano*result.Num);
		fin2.close();
	};

private:
	static const int M = 12;//��������ӵĶ�λ��Ϣ��12��
	static const int M_Nano = 3;//X_coordi Y_coordi LocalizationError
};