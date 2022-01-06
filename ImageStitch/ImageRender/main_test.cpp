#include <stdio.h>
#include <string>
#include <iostream>
#include <fstream>

#include "ImageRender.cuh"
#include "point_RW.cpp"

using namespace std;

int main()
{

	string File = "F://Œ¢π‹Õº_—È÷§//1_result2D7_M_DriftCorrected_g400f.txt";
	point_RW R_Writer;
	Result imageFirst = R_Writer.point_Read_Nano(File);
	ImageRender render(imageFirst,107,3,5);
	render.Render();

	system("pause");
}
