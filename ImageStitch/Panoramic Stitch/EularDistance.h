#pragma once
#ifndef _EULARDISTANCE_H_
#define _EULARDISTANCE_H_

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "..//ReaderWriterIO//point_RW.cpp"

using namespace pcl;

float EularDis(Result *Data_1, Result *Data_2, Eigen::MatrixXf *move);

#endif