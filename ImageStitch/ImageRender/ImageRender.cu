#include "ImageRender.cuh"

__global__ static void GaussKernal(uchar* Image, float* Table, int ROISize, int N, int Width)
{
	int i = threadIdx.x + blockIdx.x*blockDim.x;
	int x = Table[3 * i];
	int y = Table[3 * i + 1];
	int c = 0.03*Table[3 * i + 2];

	for (int p = 0; p < ROISize; p++) {
		for (int q = 0; q < ROISize; q++) {
			if (i < N) {
				int x_current = x + p - ((ROISize - 1) / 2);
				int y_current = y + q - ((ROISize - 1) / 2);
				Image[x_current*Width + y_current] = 200*exp(-(powf((x_current - x), 2) + powf((y_current - y), 2)) / (2 * c*c));
			}
		}
	}
}

ImageRender::ImageRender(Result table, int imgRawSize, int renderdPixelSize, int gaussianKernalSize) {
		r_table = table;
		r_imgRawSize = imgRawSize;
		r_renderdPixelSize = renderdPixelSize;
		r_gaussianKernalSize = gaussianKernalSize;
	}

void ImageRender::Render() {
	int MaxX = 0, MaxY = 0;
	for (int i = 0; i < r_table.Num; i++) {
		r_table.pointData[M_nano*i] = round(r_table.pointData[M_nano*i] * r_imgRawSize / r_renderdPixelSize) + floor(r_gaussianKernalSize / 2 + 1);
		r_table.pointData[M_nano*i + 1] = round(r_table.pointData[M_nano*i + 1] * r_imgRawSize / r_renderdPixelSize) + floor(r_gaussianKernalSize / 2 + 1);
		r_table.pointData[M_nano*i + 2] = r_table.pointData[M_nano*i + 2] * r_imgRawSize / r_renderdPixelSize;//转换为像素个数
		if (r_table.pointData[M_nano*i] > MaxX) MaxX = r_table.pointData[M_nano*i];
		if (r_table.pointData[M_nano*i + 1] > MaxY) MaxY = r_table.pointData[M_nano*i + 1];
	}

	cv::Mat renderImage = cv::Mat::zeros(MaxX + 10, MaxY + 10, CV_8UC1);
	
	size_t memSize_Image = renderImage.cols * renderImage.rows * sizeof(uchar);//一个字节，8位，256，对应CV_8UC1
	size_t memSize_Table = M_nano * r_table.Num * sizeof(float);
	uchar* Image_cuda = NULL;
	float* pointTable_cuda = NULL;
	cudaMalloc((void**)&Image_cuda, memSize_Image);
	cudaMalloc((void**)&pointTable_cuda, memSize_Table);
	cudaMemcpy(Image_cuda, renderImage.data ,memSize_Image, cudaMemcpyHostToDevice);
	cudaMemcpy(pointTable_cuda, r_table.pointData, memSize_Table, cudaMemcpyHostToDevice);
	int Num = r_table.Num / 1024 + 1;
	GaussKernal << <Num, 1024 >> > (Image_cuda, pointTable_cuda, r_gaussianKernalSize, r_table.Num, renderImage.cols);
	cudaMemcpy(renderImage.data, Image_cuda, memSize_Image, cudaMemcpyDeviceToHost);
	cudaFree(Image_cuda);
	cudaFree(pointTable_cuda);

	cv::namedWindow("RenderdImage", CV_WINDOW_NORMAL);
	cv::imshow("RenderdImage",renderImage);
	cv::waitKey(0);
}