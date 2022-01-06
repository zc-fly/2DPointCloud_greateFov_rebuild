#pragma once

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <vector>

using namespace std;

struct Result {
	float* pointData;
	int Num;
};

class point_RW{

public:
	Result point_Read(string pathName) {
		ifstream fin1(pathName,ios::binary);
		assert(fin1);
		fin1.seekg(0, ios::end);
		const int N = fin1.tellg() / (M * sizeof(float));
		float* temp=new float[M*N];
		fin1.seekg(0, ios::beg);
		fin1.read((char*)temp, M*N*sizeof(float));
		fin1.close();
		Result result;
		result.pointData = temp;
		result.Num = N;
		return result;
	};

	void point_Write(Result result, string totalFile) {
		ofstream fin2(totalFile, ios::binary | ios::ate | ios::out);
		fin2.write((char*)result.pointData, sizeof(float)*M*result.Num);
		fin2.close();
	};

private:
	static const int M = 12;//单个点分子的定位信息有12列
};