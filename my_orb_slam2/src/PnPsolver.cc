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

#include <iostream>
#include <cmath>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "DUtils/Random.h"
#include "PnPsolver.h"
#include "MapPoint.h"
#include "Frame.h"
#include "GlobalData.h"

namespace ORB_SLAM2
{

	PnPsolver::PnPsolver(Map *pMap, const Frame &F, const vector<MapPoint*> &vpMapPointMatches) :
		mpMap(pMap), mnCurrentIterationsCount(0)
	{
		// 预先分配空间
		mvP2D.reserve(F.mvpMapPoints.size());
		mvP3Dw.reserve(F.mvpMapPoints.size());

		for (size_t i = 0, iend = vpMapPointMatches.size(); i < iend; i++)
		{
			MapPoint *pMP = vpMapPointMatches[i];

			if (pMP)
			{
				if (!pMP->isBad(pMP, mpMap))
				{
					const cv::KeyPoint &kp = F.mvKeysUn[i];
					mvP2D.push_back(kp.pt);

					cv::Mat Pos = pMP->GetWorldPos();
					mvP3Dw.push_back(cv::Point3f(Pos.at<float>(0), Pos.at<float>(1), Pos.at<float>(2)));
				}
			}
		}

		//Set camera calibration parameters
		//得到当前帧的相机内部参数
		fx = F.fx;
		fy = F.fy;
		cx = F.cx;
		cy = F.cy;

		//mMaxIterationsCount = 300;
		//mReprojectionError = 3.f;
		//mConfidence = 0.99;
		//mMinInliers = 50;
	}

	cv::Mat PnPsolver::iterate(int iterationsCount, bool &bNoMore, vector<bool> &vbInliers, int &nInliers)
	{
		bNoMore = false;
		vbInliers.clear();
		nInliers = 0;

		if (iterationsCount < 1)
		{
			cerr << "Error：iterationsCount must be greater than 0, but now is " << iterationsCount << "; location PnPsolver::solve" << endl;
			iterationsCount = 1;
		}
		if (mnCurrentIterationsCount >= mMaxIterationsCount)
		{
			bNoMore = true;
			return cv::Mat();
		}

		float cammat[9] = { fx, 0, cx, 0, fy, cy, 0, 0, 0 };
		cv::Mat camMat(3, 3, CV_32F, (void*)cammat);
		cv::Mat M = cv::Mat::eye(4, 4, CV_32F);
		cv::Mat rmat;
		cv::Mat tmat;
		cv::Mat inliers;// 误差小于指定值的点对编号

		//cv::solvePnPRansac(mvP3Dw, mvP2D, camMat, cv::noArray(), rvec, tvec, false, mRansacMaxIts, mRansacEpsilon, mRansacProb, inliers, cv::SOLVEPNP_EPNP);
		cv::solvePnPRansac(mvP3Dw, mvP2D, camMat, cv::noArray(), mrvec, mtvec, mnCurrentIterationsCount > 0, iterationsCount, mReprojectionError, mConfidence, inliers, cv::SOLVEPNP_EPNP);
		
		cv::Rodrigues(mrvec, rmat);
		rmat.copyTo(M.rowRange(0, 3).colRange(0, 3));
		tmat = mtvec.reshape(1, 1);
		tmat.copyTo(M.row(3).colRange(0, 3));
		nInliers = inliers.rows;

		mnCurrentIterationsCount += iterationsCount;
		if (mnCurrentIterationsCount >= mMaxIterationsCount)
			bNoMore = true;

		//if (nInliers < mRansacMinInliers)
		if (nInliers < mMinInliers)
		{
			return cv::Mat();
		}

		vbInliers.resize(mvP3Dw.size(), false);
		for (int i = 0; i < inliers.rows; ++i)
		{
			vbInliers[((int*)inliers.data)[i]] = true;
		}
		return M;
	}

} //namespace ORB_SLAM
