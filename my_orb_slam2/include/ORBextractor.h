/**
* This file is part of ORB-SLAM2.
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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv2/core.hpp>

using namespace std;

namespace ORB_SLAM2
{

	class ExtractorNode
	{
	public:
		ExtractorNode() :bNoMore(false) {}

		void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

		vector<cv::KeyPoint> vKeys;
		cv::Point2i UL, UR, BL, BR;
		list<ExtractorNode>::iterator lit;
		bool bNoMore;
	};

	class ORBextractor
	{
	public:

		enum { HARRIS_SCORE = 0, FAST_SCORE = 1 };

		ORBextractor(int nfeatures, float scaleFactor, int nlevels,
			int iniThFAST, int minThFAST);

		~ORBextractor() {}

		//// pull request #600
		//static bool CompareSmall(pair<int, ExtractorNode*> &p1, pair<int, ExtractorNode*> &p2)
		//{
		//	return (p1.first < p2.first);
		//}

		// Compute the ORB features and descriptors on an image.
		// ORB are dispersed on the image using an octree.
		// Mask is ignored in the current implementation.
		void operator()(cv::InputArray image, cv::InputArray mask,
			vector<cv::KeyPoint>& keypoints,
			cv::OutputArray descriptors);

		int inline GetLevels()
		{
			return nlevels;
		}

		float inline GetScaleFactor()
		{
			return scaleFactor;
		}

		vector<float> inline GetScaleFactors()
		{
			return mvScaleFactor;
		}

		vector<float> inline GetInverseScaleFactors()
		{
			return mvInvScaleFactor;
		}

		vector<float> inline GetScaleSigmaSquares()
		{
			return mvLevelSigma2;
		}

		vector<float> inline GetInverseScaleSigmaSquares()
		{
			return mvInvLevelSigma2;
		}

		vector<cv::Mat> mvImagePyramid;

	protected:

		void ComputePyramid(cv::Mat image);
		void ComputeKeyPointsOctTree(vector<vector<cv::KeyPoint> >& allKeypoints);
		vector<cv::KeyPoint> DistributeOctTree(const vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
			const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);

		void ComputeKeyPointsOld(vector<vector<cv::KeyPoint> >& allKeypoints);
		vector<cv::Point> pattern;

		int nfeatures;
		double scaleFactor;
		int nlevels;
		int iniThFAST;
		int minThFAST;

		vector<int> mnFeaturesPerLevel;

		vector<int> umax;

		vector<float> mvScaleFactor;
		vector<float> mvInvScaleFactor;
		vector<float> mvLevelSigma2;
		vector<float> mvInvLevelSigma2;
	};

} //namespace ORB_SLAM

#endif

