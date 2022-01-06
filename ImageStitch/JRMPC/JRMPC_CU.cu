#include "JRMPC_CU.cuh"

using namespace std;
using namespace Eigen;


__global__ static void MatrixEularDis(float *P, float *C, float *A, int N, int K)
{
	int i = threadIdx.x + blockIdx.x * blockDim.x;
	for (int j = 0; j < K; j++) {
		if (i < N) {
			A[j*N + i] = abs(P[i] + C[j] - 2 * A[j*N+i]);
		}
	}

}


void JRMPC::setInputPoint(struct Result *Data_1, struct Result *Data_2)
{
	PointData_1 = *Data_1;
	PointData_2 = *Data_2;
}

void JRMPC::edgeArea_extraction(int Edge, int Flag)
{
	int Num_1 = 0;
	int Num_2 = 0;
	float Max_1 = 0;
	float Min_2 = INT_MAX;
	float(*point_1)[M_Nano] = (float(*)[M_Nano]) PointData_1.pointData;
	float(*point_2)[M_Nano] = (float(*)[M_Nano]) PointData_2.pointData;

	if (Flag == 1) {
		for (int i = 0; i < PointData_1.Num; i++)//find the Max or Min Value
			if (point_1[i][0] > Max_1) Max_1 = point_1[i][0];
		for (int i = 0; i < PointData_2.Num; i++)
			if (point_2[i][0] < Min_2) Min_2 = point_2[i][0];
		for (int i = 0; i < PointData_1.Num; i++) {//Count the edgeArea points nums
			if (point_1[i][0] > Max_1 - Edge) Num_1++;
		}
		for (int i = 0; i < PointData_2.Num; i++) {//Count the edgeArea points nums
			if (point_2[i][0] < Min_2 + Edge) Num_2++;
		}
		int i_1 = 0;
		float *pointEdge_1 = new float[Num_1*M_Nano];
		for (int i = 0; i < PointData_1.Num; i++) {//Get the edgeArea points
			if (point_1[i][0] > Max_1 - Edge) {
				for (int j = 0; j < M_Nano; j++) {
					pointEdge_1[i_1] = point_1[i][j];
					i_1++;
				}
			}
		}
		delete[] PointData_1.pointData;

		int i_2 = 0;
		float *pointEdge_2 = new float[Num_2*M_Nano];
		for (int i = 0; i < PointData_2.Num; i++) {//Get the edgeArea points
			if (point_2[i][0] < Min_2 + Edge) {
				for (int j = 0; j < M_Nano; j++) {
					pointEdge_2[i_2] = point_2[i][j];
					i_2++;
				}
			}
		}
		delete[] PointData_2.pointData;

		PointData_1.Num = Num_1;
		PointData_1.pointData = pointEdge_1;
		PointData_2.Num = Num_2;
		PointData_2.pointData = pointEdge_2;

	}
	else if (Flag == 2) {
		for (int i = 0; i < PointData_1.Num; i++)//find the Max or Min Value
			if (point_1[i][1] > Max_1) Max_1 = point_1[i][1];
		for (int i = 0; i < PointData_2.Num; i++)
			if (point_2[i][1] < Min_2) Min_2 = point_2[i][1];
		for (int i = 0; i < PointData_1.Num; i++) {//Count the edgeArea points nums
			if (point_1[i][1] > Max_1 - Edge) Num_1++;
		}
		for (int i = 0; i < PointData_2.Num; i++) {//Count the edgeArea points nums
			if (point_2[i][1] < Min_2 + Edge) Num_2++;
		}
		int i_1 = 0;
		float *pointEdge_1 = new float[Num_1*M_Nano];
		for (int i = 0; i < PointData_1.Num; i++) {//Get the edgeArea points
			if (point_1[i][1] > Max_1 - Edge) {
				for (int j = 0; j < M_Nano; j++) {
					pointEdge_1[i_1] = point_1[i][j];
					i_1++;
				}
			}
		}
		delete[] PointData_1.pointData;

		int i_2 = 0;
		float *pointEdge_2 = new float[Num_2*M_Nano];
		for (int i = 0; i < PointData_2.Num; i++) {//Get the edgeArea points
			if (point_2[i][1] < Min_2 + Edge) {
				for (int j = 0; j < M_Nano; j++) {
					pointEdge_2[i_2] = point_2[i][j];
					i_2++;
				}
			}
		}
		delete[] PointData_2.pointData;

		PointData_1.Num = Num_1;
		PointData_1.pointData = pointEdge_1;
		PointData_2.Num = Num_2;
		PointData_2.pointData = pointEdge_2;
	}
	else {
		cout << "Set a correct Flag Value";
		exit(0);
	}

};

/*
void JRMPC::preProcess()
{

};*/

MatrixXf JRMPC::jrmpcMethod()
{

	//Setting Paraments for JRMPC
	const int K = this->K;//聚类中心点数量
	const int maxNumIter = this->maxNumIter;//迭代次数
	double epsilon = pow(10.0,-5);
	float gamma = 0.0500;//(1 / K)
	Matrix<float, K, 1> pk;//K个点集权重+噪声点集权重
	MatrixXf moveDistanceXY(1, 2);
	//*******pk = 0*pk.array() + (1 / (K*(gamma + 1)));
	pk = 0 * pk.array() + 0.003200;

	//vector<pair<float, float>> T(2);//位移量
	Matrix<float, 2, 2> T;
	T.fill(0);
	//MatrixXf T = MatrixXf::Zero(2,2);//位移量

	//Create Data Matrix
	Matrix<float, Dynamic, Dynamic, RowMajor> imageRaw_1;
	imageRaw_1.resize(PointData_1.Num, 2);
	Matrix<float, Dynamic, Dynamic, RowMajor> imageRaw_2;
	imageRaw_2.resize(PointData_2.Num, 2);
	Matrix<float, K, 2, RowMajor> K_CenterPoint;
	Matrix<float, K, 1> az;

	float(*point_1)[M_Nano] = (float(*)[M_Nano]) PointData_1.pointData;
	for (int i = 0; i < PointData_1.Num; i++) {
		imageRaw_1(i, 0) = point_1[i][0];
		imageRaw_1(i, 1) = point_1[i][1];
	}

	float(*point_2)[M_Nano] = (float(*)[M_Nano]) PointData_2.pointData;
	for (int i = 0; i < PointData_2.Num; i++) {
		imageRaw_2(i, 0) = point_2[i][0];
		imageRaw_2(i, 1) = point_2[i][1];
	}

	Matrix<float, K, 1> vec;
	for (int cc = 0; cc < 300; cc++) {
		vec(cc) = cc + 1;
	}
	K_CenterPoint.col(0) = vec;
	K_CenterPoint.col(1) = vec;

	Matrix<float, Dynamic, 2> image_1;
	Matrix<float, Dynamic, 2> image_2;
	image_1 = imageRaw_1.rowwise() + T.col(0).transpose();
	image_2 = imageRaw_2.rowwise() + T.col(1).transpose();
	
	Matrix<float, K, 1> Q;
	Q = 0 * Q.array() + 0.50197*pow(10, -6);

	//h
	float h = 2 / Q.mean();
	float beta = 1.195200*pow(10,-8);

	//IterBegin
	for (int i = 0; i < maxNumIter; i++) {
		//两矩阵欧拉距离计算
		static const int Nu_1 = PointData_1.Num;
		Matrix<float, Dynamic, K, RowMajor> a_1,a_2;
		a_1.resize(PointData_1.Num, K);
		a_2.resize(PointData_2.Num, K);

		//计算两向量的欧拉距离
		a_1 = image_1 * K_CenterPoint.transpose();
		a_2 = image_2 * K_CenterPoint.transpose();
		MatrixXf image_1_sqare = image_1.rowwise().squaredNorm();
		MatrixXf image_2_sqare = image_2.rowwise().squaredNorm();
		MatrixXf K_CenterPoint_sqare = K_CenterPoint.rowwise().squaredNorm();
		float *P_1, *C, *A_1;
		cudaMalloc((void**)&P_1, PointData_1.Num*sizeof(float));
		cudaMalloc((void**)&C, K*sizeof(float));
		cudaMalloc((void**)&A_1, PointData_1.Num*K*sizeof(float));
		cudaMemcpy(P_1, image_1_sqare.data(), PointData_1.Num * sizeof(float), cudaMemcpyHostToDevice);
		cudaMemcpy(C, K_CenterPoint_sqare.data(), K * sizeof(float), cudaMemcpyHostToDevice);
		cudaMemcpy(A_1, a_1.data(), PointData_1.Num*K * sizeof(float), cudaMemcpyHostToDevice);
		dim3 Gridsize_1(PointData_1.Num/1024,1);
		dim3 Blocksize_1(1024);
		MatrixEularDis << <Gridsize_1, Blocksize_1 >> > (P_1, C, A_1, PointData_1.Num, K);
		float *P_2, *A_2;
		cudaMalloc((void**)&P_2, PointData_2.Num * sizeof(float));
		cudaMalloc((void**)&A_2, PointData_2.Num*K * sizeof(float));
		cudaMemcpy(P_2, image_2_sqare.data(), PointData_2.Num * sizeof(float), cudaMemcpyHostToDevice);
		cudaMemcpy(A_2, a_2.data(), PointData_2.Num*K * sizeof(float), cudaMemcpyHostToDevice);
		dim3 Gridsize_2(PointData_2.Num / 1024, 1);
		dim3 Blocksize_2(1024);
		MatrixEularDis << <Gridsize_2, Blocksize_2 >> > (P_2, C, A_2, PointData_2.Num, K);
		cudaMemcpy(a_1.data(), A_1, PointData_1.Num*K * sizeof(float), cudaMemcpyDeviceToHost);
		cudaMemcpy(a_2.data(), A_2, PointData_2.Num*K * sizeof(float), cudaMemcpyDeviceToHost);

		//pk*S^-1.5*exp(-.5/S^2*||.||) and normalize
		for (int t = 0; t < PointData_1.Num; t++) {
			a_1.row(t) = (a_1.row(t).array()*Q.transpose().array().pow(2)*(-0.5)).array().exp();
			a_1.row(t) = a_1.row(t).array()*pk.transpose().array()*(Q.transpose().array().pow(1.5));
			a_1.row(t) = a_1.row(t) / (a_1.row(t).sum()+beta);
		}
		for (int t = 0; t < PointData_2.Num; t++) {
			a_2.row(t) = (a_2.row(t).array()*Q.transpose().array().pow(2)*(-0.5)).array().exp();
			a_2.row(t) = a_2.row(t).array()*pk.transpose().array()*(Q.transpose().array().pow(1.5));
			a_2.row(t) = a_2.row(t) / (a_2.row(t).sum() + beta);
		}

		Matrix<float, K, 2>lambda;
		Matrix<float, 2, K>W_1;
		Matrix<float, 2, K>W_2;
		Matrix<float, K, 2>b;
		Matrix<float, 2, 2>mW;
		Matrix<float, 2, 2>mX;
		Matrix<float, 1, 2>sumOfWeight;
		lambda.col(0) = a_1.colwise().sum();
		lambda.col(1) = a_2.colwise().sum();
		W_1.row(0) = (imageRaw_1.transpose()*a_1).row(0).cwiseProduct(Q.transpose());
		W_1.row(1) = (imageRaw_1.transpose()*a_1).row(1).cwiseProduct(Q.transpose());
		W_2.row(0) = (imageRaw_2.transpose()*a_2).row(0).cwiseProduct(Q.transpose());
		W_2.row(1) = (imageRaw_2.transpose()*a_2).row(1).cwiseProduct(Q.transpose());

		b.col(0) = lambda.col(0).cwiseProduct(Q);
		b.col(1) = lambda.col(1).cwiseProduct(Q);
		mW.col(0) = W_1.rowwise().sum();
		mW.col(1) = W_2.rowwise().sum();
		mX.col(0) = K_CenterPoint.transpose()*b.col(0);
		mX.col(1) = K_CenterPoint.transpose()*b.col(1);
		sumOfWeight(0, 0) = lambda.col(0).dot(Q);
		sumOfWeight(0, 1) = lambda.col(1).dot(Q);

		T.col(0) = (mX.col(0) - mW.col(0))/sumOfWeight(0,0);
		T.col(1) = (mX.col(1) - mW.col(1))/sumOfWeight(0,1);

		image_1 = imageRaw_1.rowwise() + T.col(0).transpose();
		image_2 = imageRaw_2.rowwise() + T.col(1).transpose();

		Matrix<float, 1, K>den;
		den = lambda.rowwise().sum().transpose();
		K_CenterPoint = a_1.transpose()*image_1 + a_2.transpose()*image_2;
		K_CenterPoint.col(0) = K_CenterPoint.col(0).cwiseQuotient(den.transpose());
		K_CenterPoint.col(1) = K_CenterPoint.col(1).cwiseQuotient(den.transpose());

		Matrix<float, K, 2>wnormes;
		Matrix<float, Dynamic, K> temp_a_1, temp_a_2;
		temp_a_1.resize(PointData_1.Num, K);
		temp_a_2.resize(PointData_2.Num, K);

		//计算两向量的欧拉距离
		temp_a_1 = image_1 * K_CenterPoint.transpose();
		temp_a_2 = image_2 * K_CenterPoint.transpose();
		image_1_sqare = image_1.rowwise().squaredNorm();
		image_2_sqare = image_2.rowwise().squaredNorm();
		K_CenterPoint_sqare = K_CenterPoint.rowwise().squaredNorm();
		cudaMemcpy(C, K_CenterPoint_sqare.data(), K * sizeof(float), cudaMemcpyHostToDevice);
		cudaMemcpy(P_1, image_1_sqare.data(), PointData_1.Num * sizeof(float), cudaMemcpyHostToDevice);
		cudaMemcpy(A_1, temp_a_1.data(), PointData_1.Num*K * sizeof(float), cudaMemcpyHostToDevice);
		MatrixEularDis << <Gridsize_1, Blocksize_1 >> > (P_1, C, A_1, PointData_1.Num, K);
		cudaMemcpy(P_2, image_2_sqare.data(), PointData_2.Num * sizeof(float), cudaMemcpyHostToDevice);
		cudaMemcpy(A_2, temp_a_2.data(), PointData_2.Num*K * sizeof(float), cudaMemcpyHostToDevice);
		MatrixEularDis << <Gridsize_2, Blocksize_2 >> > (P_2, C, A_2, PointData_2.Num, K);
		cudaMemcpy(temp_a_1.data(), A_1, PointData_1.Num*K * sizeof(float), cudaMemcpyDeviceToHost);
		cudaMemcpy(temp_a_2.data(), A_2, PointData_2.Num*K * sizeof(float), cudaMemcpyDeviceToHost);
		cudaFree(P_1);
		cudaFree(P_2);
		cudaFree(C);
		cudaFree(A_1);
		cudaFree(A_2);

		wnormes.col(0) = (a_1.cwiseProduct(temp_a_1)).colwise().sum();
		wnormes.col(1) = (a_2.cwiseProduct(temp_a_2)).colwise().sum();

		Q = 3*den.transpose().cwiseQuotient(wnormes.rowwise().sum() + 3 * epsilon * den.transpose());

	}

	moveDistanceXY(0, 0) = -(T(0, 1) - T(0, 0));
	moveDistanceXY(0, 1) = -(T(1, 1) - T(1, 0));
	Tans = moveDistanceXY;
	return moveDistanceXY;
};
