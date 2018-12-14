#include "IoUtils.h"


void IoUtils::WriteMat(ofstream &f, const cv::Mat &_m)
{
	cv::Mat m;
	// 这里需要确保 输入矩阵是内存连续的
	if (m.isSubmatrix())
		m = _m.clone();
	else
		m = _m;
	uint32_t header[] = {m.rows, m.cols, m.type()};
	f.write((char*)header, sizeof(header));
	f.write((char*)m.data, m.rows * m.step);
}

void IoUtils::ReadMat(ifstream &f, cv::Mat &m)
{
	uint32_t header[3];
	f.read((char*)header, sizeof(header));
	m = cv::Mat(header[0], header[1], header[2]);
	f.read((char*)m.data, m.rows * m.step);
}

void IoUtils::WriteKeyPoint(ofstream &f, const cv::KeyPoint& kp)
{
	f.write((char*)&kp, sizeof(kp));
}

void IoUtils::ReadKeyPoint(ifstream &f, cv::KeyPoint& kp)
{
	f.read((char*)&kp, sizeof(kp));
}

void IoUtils::WriteKeyPoints(ofstream &f, const vector<cv::KeyPoint>& kps)
{
	uint64_t size = kps.size();
	Write(f, size);
	for (uint64_t i = 0; i < size; ++i)
		WriteKeyPoint(f, kps[i]);
}

void IoUtils::ReadKeyPoints(ifstream &f, vector<cv::KeyPoint>& kps)
{
	uint64_t size;
	Read(f, size);
	kps.resize(size);
	for (uint64_t i = 0; i < size; ++i)
		ReadKeyPoint(f, kps[i]);
}

void IoUtils::WriteFloatVector(ofstream &f, const vector<float>& vf)
{
	uint64_t size = vf.size();
	Write(f, size);
	for (auto i : vf)
		Write(f, i);
}

void IoUtils::ReadFloatVector(ifstream &f, vector<float>& vf)
{
	uint64_t size;
	Read(f, size);
	vf.resize(size);
	for (uint64_t i = 0; i < size; ++i)
	{
		float a;
		Read(f, a);
		vf[i] = a;
	}
}

void IoUtils::WriteUInt32Vector3s(ofstream &f, const vector<vector<vector<uint32_t>>> &v3s)
{
	uint32_t size1 = v3s.size();
	Write(f, size1);
	for (uint32_t i = 0; i < size1; ++i)
	{
		uint32_t size2 = v3s[i].size();
		Write(f, size2);
		for (uint32_t j = 0; j < size2; ++j)
		{
			uint32_t size3 = v3s[i][j].size();
			Write(f, size3);
			for (uint32_t k = 0; k < size3; ++k)
			{
				Write(f, v3s[i][j][k]);
			}
		}
	}
}

void IoUtils::ReadUInt32Vector3s(ifstream &f, vector<vector<vector<uint32_t>>> &v3s)
{
	uint32_t size1;
	Read(f, size1);
	v3s.resize(size1);
	for (uint32_t i = 0; i < size1; ++i)
	{
		uint32_t size2;
		Read(f, size2);
		v3s[i].resize(size2);
		for (uint32_t j = 0; j < size2; ++j)
		{
			uint32_t size3;
			Read(f, size3);
			v3s[i][j].resize(size3);
			for (uint32_t k = 0; k < size3; ++k)
			{
				Read(f, v3s[i][j][k]);
			}
		}
	}
}
