#pragma once

#include <list>
#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>

#include "EularDistance.h"
#include "..//ImageRender//ImageRender.cuh"
#include "HassianMatrix.h"
#include "../JRMPC/JRMPC_CU.cuh"
#include "..//ReaderWriterIO//ListFiles_x64.h"

using namespace std;

#define MyMax (float)1000000;

struct UnitInfo {//ƴͼ���
	pair<int, int> order;//ƴ�ӵ�ͼ���
	pair<float, float> xy_move;//��ͼ�����λ�� 
	float eularDis;//��ͼŷʽ���룬��������ƴ��Ч������ȫ���Ż�
};

int main()
{
	//**********************����ƴ�Ӳ�������Ⱦ����**************************
	string FilePath = "F://Data2_test//groundTRUTH//rawData_small//";        //�ļ�Ŀ¼
	string SavePath = "F://Data2_test//groundTRUTH//Stitch_Panoramic.txt"; //PanoramicImage����·��
	
	const int image_col = 2;//��ƴ��С�ӳ�ͼ����������
	const int image_row = 2;//��ƴ��С�ӳ�ͼ����������
	int EdgeSize = 60;//��ȡ�ص���Ե���ִ�С
	const int M_Nano = 3;//����������Ϣά��

	int imgRawSize = 108;//�ɼ�ͼ���ȡ��λ���Ƶ�ԭʼ�����Ԫ�ߴ�
	int renderdPixelSize = 1;//��Ⱦͼ�����Ԫ�ߴ�
	int gaussianKernalSize = 15;//��Ⱦͼ��ĸ�˹�˴�С
	
	//**********************����ͼ��ƴ��**********************************
	vector<string> Files;
	getFiles(FilePath, Files);
	int Num = Files.size();
	point_RW R_Writer;
	list<UnitInfo> StitchInfo;
	MatrixXf HassianM = HassianMatrix(image_col, image_row);
	const int n = image_col * image_row;

	for (int i = 0; i < image_col*image_row -1; i++) {//����ͼ�����ƴ��
		for (int j = i+1; j < image_row*image_col; j++) {
			if (HassianM(i, j) == 1) {
				if (i%image_col == j % image_col) {
					Result imageFirst = R_Writer.point_Read(Files.at(i));
					Result imageSecond = R_Writer.point_Read(Files.at(j));
					JRMPC StitchClass_Row;
					StitchClass_Row.setInputPoint(&imageFirst, &imageSecond);
					StitchClass_Row.edgeArea_extraction(EdgeSize, 2);//����Ϊ1��ʾ��ȡ��������2��ʾ��������
					MatrixXf moveDis = StitchClass_Row.jrmpcMethod();
					float eularDistance = EularDis(&StitchClass_Row.PointData_1, &StitchClass_Row.PointData_2, &moveDis);
					delete StitchClass_Row.PointData_1.pointData;
					delete StitchClass_Row.PointData_2.pointData;

					UnitInfo info;
					info.order = make_pair(i, j);
					info.xy_move = make_pair(moveDis(0, 0), moveDis(0, 1));
					info.eularDis = eularDistance;
					StitchInfo.push_back(info);
					cout << "-------------------------" << endl;
					cout << "����ƴ��ͼ:" << i << "\t" << j << endl;
					cout << "���:" << "x:" << moveDis(0, 0) << "\t" << "y:" << moveDis(0, 1) << endl;
					cout << "ƴ�ӵ÷�:"<< 100/eularDistance << endl;

				}
				else {
					Result imageFirst = R_Writer.point_Read(Files.at(i));
					Result imageSecond = R_Writer.point_Read(Files.at(j));
					JRMPC StitchClass_Col;
					StitchClass_Col.setInputPoint(&imageFirst, &imageSecond);
					StitchClass_Col.edgeArea_extraction(EdgeSize, 1);//����Ϊ1��ʾ��ȡ��������2��ʾ��������
					MatrixXf moveDis = StitchClass_Col.jrmpcMethod();
					float eularDistance = EularDis(&StitchClass_Col.PointData_1, &StitchClass_Col.PointData_2, &moveDis);
					delete StitchClass_Col.PointData_1.pointData;
					delete StitchClass_Col.PointData_2.pointData;

					UnitInfo info;
					info.order = make_pair(i, j);
					info.xy_move = make_pair(moveDis(0, 0), moveDis(0, 1));
					info.eularDis = eularDistance;
					StitchInfo.push_back(info);
					cout << "-------------------------" << endl;
					cout << "����ƴ��ͼ:" << i << "\t" << j << endl;
					cout << "���:" << "x:" << moveDis(0, 0) << "\t" << "y:" << moveDis(0, 1) << endl;
					cout << "ƴ�ӵ÷�:" << 100/eularDistance << endl;
				}
			}
		}
	}

	//*******************��С��������ƴ��·���Ż�*****************
	MatrixXf DistanceMatrix = MatrixXf::Ones(n, n);
	DistanceMatrix = DistanceMatrix * MyMax;
	for (list< UnitInfo >::iterator iter = StitchInfo.begin(); iter != StitchInfo.end(); iter++) {
		DistanceMatrix((*iter).order.first, (*iter).order.second) = (*iter).eularDis;
		DistanceMatrix((*iter).order.second, (*iter).order.first) = (*iter).eularDis;
	}

	list<int> MyTree;
	vector<pair<float, float>> MyTreeDis;
	int startPoint = 0;
	MyTree.push_back(startPoint);
	MyTreeDis.push_back(make_pair(0, 0));

	for (int i = 0; i < n-1; i++) {
		float minV = MyMax;
		int currentPoint;
		int nestPoint;
		for (list<int>::iterator iter = MyTree.begin(); iter != MyTree.end(); iter++) {//��С��������Prim�㷨��
			for (int j = 0; j < n; j++) {
				bool flag = true;
				for (list<int>::iterator iter_2 = MyTree.begin(); iter_2 != MyTree.end(); iter_2++) {
					if ((*iter_2) == j) 
						flag = false;
				}
				if ( flag && minV > DistanceMatrix((*iter), j)) {
					minV = DistanceMatrix((*iter), j);
					currentPoint = (*iter);
					nestPoint = j;
				}
			}
		}

		MyTree.push_back(nestPoint);
		for (list< UnitInfo >::iterator iter = StitchInfo.begin(); iter != StitchInfo.end(); iter++) {
			if (currentPoint < nestPoint) {
				if (make_pair(currentPoint, nestPoint) == (*iter).order)
					MyTreeDis.push_back(make_pair((*iter).xy_move.first + MyTreeDis[i].first, (*iter).xy_move.second + MyTreeDis[i].second));
			}
			else {
				if (make_pair(nestPoint, currentPoint) == (*iter).order) {
					MyTreeDis.push_back(make_pair(-(*iter).xy_move.first + MyTreeDis[i].first, -(*iter).xy_move.second + MyTreeDis[i].second));
				}
			}
		}

	}

	//************�������λ�ù�ϵͳһ��������ϵ*****************
	if (access(SavePath.c_str(), 0) == 0) remove(SavePath.c_str());

	int j = 0;
	for (list<int>::iterator iterlist = MyTree.begin(); iterlist != MyTree.end(); ++iterlist, j++) {
		point_RW RWriter_all;
		Result ThisImage = RWriter_all.point_Read(Files.at(*iterlist));
		float(*point)[M_Nano] = (float(*)[M_Nano]) ThisImage.pointData;
		//cout << *iterlist<<"\t"<< MyTreeDis[j].first << "\t" << MyTreeDis[j].second << endl;
		for (int i = 0; i < ThisImage.Num; i++) {
			point[i][0] = point[i][0] + MyTreeDis[j].first;
			point[i][1] = point[i][1] + MyTreeDis[j].second;
		}
		Result DataP;
		DataP.pointData = (float(*)) ThisImage.pointData;
		DataP.Num = ThisImage.Num;
		RWriter_all.point_Write(DataP, SavePath);
		delete ThisImage.pointData;
	}
	
	//*********************��Ⱦ*********************************
	point_RW RWriter_render;
	Result PanoramicImage = RWriter_render.point_Read_3(SavePath);
	ImageRender RenderObject(PanoramicImage, imgRawSize, renderdPixelSize, gaussianKernalSize);
	RenderObject.Render();

}