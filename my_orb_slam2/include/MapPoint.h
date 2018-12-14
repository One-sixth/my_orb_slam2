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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include <mutex>
#include <map>
#include <cstdint>
#include <opencv2/core.hpp>
#include "Map.h"

using namespace std;

namespace ORB_SLAM2
{

	class KeyFrame;
	class Map;
	class Frame;
	class GlobalData;


	class MapPoint
	{
		friend bool Map::IsBadMapPoint(MapPoint *pMP);
		friend void Map::CheckAndClean();
	public:
		MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, GlobalData *pGlobalData);
		MapPoint(const cv::Mat &Pos, GlobalData *pGlobalData, Frame *pFrame, const int32_t idxF);
		MapPoint(const cv::Mat &Pos, GlobalData *pGlobalData, uint64_t nid);
		~MapPoint();

		void SetWorldPos(const cv::Mat &Pos);
		cv::Mat GetWorldPos();

		cv::Mat GetNormal();
		KeyFrame *GetReferenceKeyFrame();
		void SetReferenceKeyFrame(KeyFrame *refKf);

		map<KeyFrame*, size_t> GetObservations();
		int32_t Observations();

		void AddObservation(KeyFrame *pKF, size_t idx);
		void EraseObservation(KeyFrame *pKF);

		int32_t GetIndexInKeyFrame(KeyFrame *pKF);
		bool IsInKeyFrame(KeyFrame *pKF);

		//SetBadFlag后，此地图点将会加入待删除列表，所以必须在里面解除所有类对该地图点的引用
		void SetBadFlag();
		static bool isBad(MapPoint *pMP, Map *pMap);

		//Replace后，此地图点将会加入待删除列表，所以必须在里面解除所有类对该地图点的引用
		void Replace(MapPoint *pMP);
		MapPoint* GetReplaced();

		void IncreaseVisible(int32_t n = 1);
		void IncreaseFound(int32_t n = 1);
		float GetFoundRatio();
		inline int32_t GetFound()
		{
			return mnFound;
		}

		void ComputeDistinctiveDescriptors();

		cv::Mat GetDescriptor();

		void UpdateNormalAndDepth();

		float GetMinDistanceInvariance();
		float GetMaxDistanceInvariance();
		int32_t PredictScale(const float currentDist, KeyFrame *pKF);
		int32_t PredictScale(const float currentDist, Frame *pF);

		static void SaveMapPoints(ofstream &f, const set<MapPoint*>& _mps, Map *pMap);
		static void LoadMapPoints(ifstream &f, map<uint64_t, MapPoint*> &mMapPointById, GlobalData *pGlobalData);

	public:
		uint64_t mnId = 0;
		//static uint32_t nNextId;
		// 生成该地图点的关键帧id
		uint64_t mnFirstKFid = 0;
		// 生成该点的普通帧id
		uint64_t mnFirstFrame = 0;
		int32_t nObs = 0;

		// Variables used by the tracking
		float mTrackProjX = 0;
		float mTrackProjY = 0;
		float mTrackProjXR = 0;
		bool mbTrackInView = false;
		int32_t mnTrackScaleLevel = 0;
		float mTrackViewCos = 0;
		uint64_t mnTrackReferenceForFrame = 0;
		uint64_t mnLastFrameSeen = 0;

		// Variables used by local mapping
		uint64_t mnBALocalForKF = 0;
		uint64_t mnFuseCandidateForKF = 0;

		// Variables used by loop closing
		uint64_t mnLoopPointForKF = 0;
		uint64_t mnCorrectedByKF = 0;
		uint64_t mnCorrectedReference = 0;
		cv::Mat mPosGBA;
		uint64_t mnBAGlobalForKF = 0;


		//static mutex mGlobalMutex;

	protected:

		// Position in absolute coordinates
		cv::Mat mWorldPos;

		// Keyframes observing the point and associated index in keyframe
		map<KeyFrame*, size_t> mObservations;

		// Mean viewing direction
		cv::Mat mNormalVector;

		// Best descriptor to fast matching
		cv::Mat mDescriptor;

		// Reference KeyFrame
		KeyFrame *mpRefKF = nullptr;

		// Tracking counters
		int32_t mnVisible = 1;
		int32_t mnFound = 1;

		// Bad flag (we do not currently erase MapPoint from memory)
		bool mbBad = false;
		MapPoint *mpReplaced = nullptr;

		// Scale invariance distances
		float mfMinDistance = 0;
		float mfMaxDistance = 0;

		Map *mpMap = nullptr;
		GlobalData *mpGlobalData = nullptr;

		recursive_mutex mMutexPos;
		recursive_mutex mMutexFeatures;
	};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
