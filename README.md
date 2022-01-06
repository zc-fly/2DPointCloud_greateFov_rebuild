# 2DPanoramaCloud_Reconstruction
序列图像的获取包括下图所示的一些扫描方式，不论我们采用何种扫描方法，最终将获得许多小视场图像，这些小视场图像的大体位置可以根据扫描方式确定，而图像拼接问题就是如何根据已知大体位置关系和相邻两幅图像的变换关系(由两两图像对准算法求取)，重建出完整的大视场图像。
代码详细方法见论文：Computational framework for generating large panoramic super-resolution images from localization microscopy[J]. Biomedical Optics Express, 2021, 12(8).
