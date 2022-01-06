#include "ListFiles_x64.h"

/*
@author:CodingMengmeng
@theme:��ȡָ���ļ����µ������ļ���
@time:2017-1-13 11:46:22
@blog:http://www.cnblogs.com/codingmengmeng/
*/


void GetFilesWithType(string filePath, string subname, vector<string>& ofiles)
{
	vector<string> files;

	// ��ȡ��·���µ������ļ�  
	getFiles(filePath, files);

	string str1;
	for (int i = 0; i < files.size(); i++)
	{
		str1 = files[i].c_str();
		if (str1.find(subname) != string::npos)
		{
			ofiles.push_back(files[i]);
		}
	}


}


void getFiles(string path, vector<string>& files)
{
	//�ļ����  
	_int64 hFile = 0;
	//�ļ���Ϣ������һ���洢�ļ���Ϣ�Ľṹ��  
	struct _finddatai64_t fileinfo;
	string p;//�ַ��������·��
	if ((hFile = _findfirsti64(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)//�����ҳɹ��������
	{
		do
		{
			//�����Ŀ¼,����֮�����ļ����ڻ����ļ��У�  
			if ((fileinfo.attrib &  _A_SUBDIR))
			{
				continue;

				//�ļ���������"."&&�ļ���������".."
				//.��ʾ��ǰĿ¼
				//..��ʾ��ǰĿ¼�ĸ�Ŀ¼
				//�ж�ʱ�����߶�Ҫ���ԣ���Ȼ�����޵ݹ�������ȥ�ˣ�
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
				{
					getFiles(p.assign(path).append("\\").append(fileinfo.name), files);

				}
			}
			//�������,�����б�  
			else
			{
				files.push_back(p.assign(path).append("\\").append(fileinfo.name));
			}
		} while (_findnexti64(hFile, &fileinfo) == 0);
		//_findclose������������
		_findclose(hFile);
	}
}


/*

int main(){
char * filePath = "E:\\experiment data\\20171115 quality control 1qc\\with qc\\group1\\MMStack_Pos0";//�Լ�����Ŀ¼
vector<string> files;

////��ȡ��·���µ������ļ�
getFiles(filePath, files);

char str[30];
int size = files.size();
for (int i = 0; i < size; i++)
{
cout << files[i].c_str() << endl;
}
}

*/