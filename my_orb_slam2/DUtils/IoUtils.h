// write/read cv::Mat https://blog.csdn.net/huixingshao/article/details/43635401

#ifndef IOUTILS_H
#define IOUTILS_H

#include <fstream>
#include <vector>
#include <cstdint>
#include <opencv2/core/core.hpp>

using namespace std;

class IoUtils
{
public:
	IoUtils() = delete;

	// 下面两个模板类最好仅用于基础类型，例如 int, float, 其他类型尽量不要使用
	template<class T>
	static void Write(ofstream &f, const T& m)
	{
		f.write((char*)&m, sizeof(m));
	}
	template<class T>
	static void Read(ifstream &f, T& m)
	{
		f.read((char*)&m, sizeof(m));
	}

	static void WriteMat(ofstream &f, const cv::Mat &m);
	static void ReadMat(ifstream &f, cv::Mat &m);

	static void WriteKeyPoint(ofstream &f, const cv::KeyPoint& kp);
	static void ReadKeyPoint(ifstream &f, cv::KeyPoint& kp);

	static void WriteKeyPoints(ofstream &f, const vector<cv::KeyPoint>& kps);
	static void ReadKeyPoints(ifstream &f, vector<cv::KeyPoint>& kps);

	static void WriteFloatVector(ofstream &f, const vector<float>& vf);
	static void ReadFloatVector(ifstream &f, vector<float>& vf);

	static void WriteUInt32Vector3s(ofstream &f, const vector<vector<vector<uint32_t>>>& v3s);
	static void ReadUInt32Vector3s(ifstream &f, vector<vector<vector<uint32_t>>>& v3s);

};

#endif