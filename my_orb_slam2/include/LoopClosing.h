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

#ifndef LOOPCLOSING_H
#define LOOPCLOSING_H

#include <thread>
#include <mutex>
#include <set>
#include <list>
#include <vector>
#include <map>
#include <opencv2/core.hpp>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include "ORBVocabulary.h"
#include "GlobalData.h"

using namespace std;

namespace ORB_SLAM2
{

	class KeyFrame;
	class Map;
	class MapPoint;
	class Tracking;
	class LocalMapping;
	class KeyFrameDatabase;
	class GlobalData;

	class LoopClosing
	{
	public:

		typedef pair<set<KeyFrame*>, int> ConsistentGroup;
		// pull requset #585
		typedef map<KeyFrame*, g2o::Sim3, less<KeyFrame*>,
			Eigen::aligned_allocator<std::pair<KeyFrame*const, g2o::Sim3>>> KeyFrameAndPose;
		//typedef map<KeyFrame*, g2o::Sim3, less<KeyFrame*>> KeyFrameAndPose;

	public:

		LoopClosing(GlobalData *pGlobalData, Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale);
		~LoopClosing();

		void SetTracker(Tracking *pTracker);

		void SetLocalMapper(LocalMapping *pLocalMapper);

		// Main function
		void Run();

		void InsertKeyFrame(KeyFrame *pKF);

		void RequestReset();

		// This function will run in a separate thread
		void RunGlobalBundleAdjustment(unsigned long nLoopKF);

		bool isRunningGBA()
		{
			unique_lock<mutex> lock(mMutexGBA);
			return mbRunningGBA;
		}

		bool isFinishedGBA()
		{
			unique_lock<mutex> lock(mMutexGBA);
			return mbFinishedGBA;
		}

		void RequestFinish();

		bool isFinished();

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

		//等待删除无效地图点
		mutex mMutexRecycling;
		mutex mMutexGBARecycling;

	protected:

		bool CheckNewKeyFrames();

		bool DetectLoop();

		bool ComputeSim3();

		void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

		void CorrectLoop();

		void ResetIfRequested();
		bool mbResetRequested = false;
		mutex mMutexReset;

		bool CheckFinish();
		void SetFinish();
		bool mbFinishRequested = false;
		bool mbFinished = true;
		mutex mMutexFinish;

		Map *mpMap = nullptr;
		Tracking *mpTracker = nullptr;
		GlobalData *mpGlobalData = nullptr;

		KeyFrameDatabase *mpKeyFrameDB = nullptr;
		ORBVocabulary *mpORBVocabulary = nullptr;

		LocalMapping *mpLocalMapper = nullptr;

		list<KeyFrame*> mlpLoopKeyFrameQueue;

		mutex mMutexLoopQueue;

		// Loop detector parameters
		float mnCovisibilityConsistencyTh = 3;

		// Loop detector variables
		KeyFrame *mpCurrentKF = nullptr;
		KeyFrame *mpMatchedKF = nullptr;
		vector<ConsistentGroup> mvConsistentGroups;
		vector<KeyFrame*> mvpEnoughConsistentCandidates;
		vector<KeyFrame*> mvpCurrentConnectedKFs;
		vector<MapPoint*> mvpCurrentMatchedPoints;
		vector<MapPoint*> mvpLoopMapPoints;
		cv::Mat mScw;
		g2o::Sim3 mg2oScw;

		uint64_t mLastLoopKFid = 0;

		// Variables related to Global Bundle Adjustment
		bool mbRunningGBA = false;
		bool mbFinishedGBA = true;
		bool mbStopGBA = false;
		mutex mMutexGBA;
		thread* mpThreadGBA = nullptr;

		// Fix scale in the stereo/RGB-D case
		bool mbFixScale = false;

		int mnFullBAIdx = 0;

		thread mtLoopClosing;
	};

} //namespace ORB_SLAM

#endif // LOOPCLOSING_H
