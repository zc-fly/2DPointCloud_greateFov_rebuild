#include "EularDistance.h"

float EularDis(Result *Data_1, Result *Data_2, Eigen::MatrixXf *move) {
	
	if ((*Data_1).Num > (*Data_2).Num) {//set the minimum PointSet as the centers
		Result temp = *Data_1;
		*Data_1 = *Data_2;
		*Data_2 = temp;
	}

	const int M_nano = 3;//Store pointCloud as a pcl point type
	float(*pointDataPCL_1)[M_nano] = (float(*)[M_nano]) (*Data_1).pointData;
	float(*pointDataPCL_2)[M_nano] = (float(*)[M_nano]) (*Data_2).pointData;
	PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
	PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_1->width = (*Data_1).Num;
	cloud_1->height = 1;
	cloud_1->points.resize((*Data_1).Num);
	for (int i = 0; i < (*Data_1).Num; i++) {
		cloud_1->points[i].x = pointDataPCL_1[i][0];
		cloud_1->points[i].y = pointDataPCL_1[i][1];
		cloud_1->points[i].z = 0;
	}
	cloud_2->width = (*Data_2).Num;
	cloud_2->height = 1;
	cloud_2->points.resize((*Data_2).Num);
	for (int i = 0; i < (*Data_2).Num; i++) {
		cloud_2->points[i].x = pointDataPCL_2[i][0];
		cloud_2->points[i].y = pointDataPCL_2[i][1];
		cloud_2->points[i].z = 0;
	}

	KdTreeFLANN<PointXYZ> NearestPoint;
	NearestPoint.setInputCloud(cloud_2);
	std::vector<int> index;
	std::vector<float> dis;
	float eularDis = 0;
	for (const auto& p : cloud_1->points) {
		NearestPoint.nearestKSearch(p, 1, index, dis);
		eularDis = dis[0] + eularDis;
	}
	eularDis = eularDis / (*Data_1).Num;

	return eularDis;
};