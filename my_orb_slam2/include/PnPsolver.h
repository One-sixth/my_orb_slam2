/**
* This file is part of ORB-SLAM2.
* This file is a modified version of EPnP <http://cvlab.epfl.ch/EPnP/index.php>, see FreeBSD license below.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

/**
* Copyright (c) 2009, V. Lepetit, EPFL
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
*   either expressed or implied, of the FreeBSD Project
*/

/*
PnPsolver 仅用于Tracking::Relocalization 函数中
原本计划迁移本类中所有 opencv 1.0 的函数到 opencv 3.0
但是因为 cv::SVDecomp 与 cvSVD 行为有些许不同(排除法，还没有看opencv源代码差别)，迁移过去后重定位失败
然后观察到 PnPsolver 与 cv::solvePnPRansac 有许多相似之处，然后尝试修改，没想到居然可行
原始代码看起来对相机位姿的重复多次优化，可以得到更好的位姿，此修改，做了类似的迭代求解，目前测试正常。
*/

#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include <vector>
#include <opencv2/core.hpp>

using namespace std;

namespace ORB_SLAM2
{

	class MapPoint;
	class Frame;
	class Map;

	class PnPsolver
	{
	public:
		PnPsolver(Map *pMap, const Frame &F, const vector<MapPoint*> &vpMapPointMatches);

		cv::Mat iterate(int iterationsCount, bool &bNoMore, vector<bool> &vbInliers, int &nInliers);

	private:
		//
		Map *mpMap = nullptr;

		// cam
		double fx, fy, cx, cy;

		// 2D Points
		vector<cv::Point2f> mvP2D;

		// 3D Points
		vector<cv::Point3f> mvP3Dw;

		// 当前已迭代次数
		int mnCurrentIterationsCount;
		// 旋转向量
		cv::Mat mrvec;
		// 位置向量
		cv::Mat mtvec;

		// 最大迭代次数
		int mMaxIterationsCount = 300;
		// 重投影误差，重投影误差小于该值的点对的编号会加入inliers
		// 该值太小，会引起重跟踪变得困难，太大，误差也会变大
		float mReprojectionError = 6.f;
		float mConfidence = 0.99;
		// 至少有这个数量的点对的重投影误差小于 reprojectionError，否则返回空矩阵
		int mMinInliers = 50;

	};

} //namespace ORB_SLAM

#endif //PNPSOLVER_H