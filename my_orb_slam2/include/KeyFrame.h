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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include <mutex>
#include <vector>
#include <set>
#include <cstdint>
#include <fstream>
#include <DBoW3.h>
#include <opencv2/core.hpp>
#include "Map.h"

using namespace std;

namespace ORB_SLAM2
{

	class Map;
	class MapPoint;
	class Frame;
	class KeyFrameDatabase;
	class ORBVocabulary;
	class KeyFrameInitializer;
	class GlobalData;


	class KeyFrame
	{
		friend bool Map::IsBadKeyFrame(KeyFrame *pKF);
		friend void Map::CheckAndClean();
	public:
		KeyFrame(GlobalData *pGlobalData);
		KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB);
		~KeyFrame();

		// Pose functions
		void SetPose(const cv::Mat &Tcw);
		cv::Mat GetPose();
		cv::Mat GetPoseInverse();
		cv::Mat GetCameraCenter();
		cv::Mat GetStereoCenter();
		cv::Mat GetRotation();
		cv::Mat GetTranslation();

		// Bag of Words Representation
		void ComputeBoW();

		// Covisibility graph functions
		void AddConnection(KeyFrame *pKF, const int32_t weight);
		void EraseConnection(KeyFrame *pKF);
		void UpdateConnections();
		void UpdateBestCovisibles();
		set<KeyFrame*> GetConnectedKeyFrames();
		vector<KeyFrame*> GetVectorCovisibleKeyFrames();
		vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int32_t N);
		vector<KeyFrame*> GetCovisiblesByWeight(const int32_t w);
		int32_t GetWeight(KeyFrame *pKF);

		// Spanning tree functions
		void AddChild(KeyFrame *pKF);
		void EraseChild(KeyFrame *pKF);
		void ChangeParent(KeyFrame *pKF);
		set<KeyFrame*> GetChilds();
		KeyFrame *GetParent();
		bool hasChild(KeyFrame *pKF);

		// Loop Edges
		void AddLoopEdge(KeyFrame *pKF);
		set<KeyFrame*> GetLoopEdges();

		// MapPoint observation functions
		void AddMapPoint(MapPoint *pMP, const size_t idx);
		void EraseMapPointMatch(const size_t idx);
		void EraseMapPointMatch(MapPoint *pMP);
		void ReplaceMapPointMatch(const size_t idx, MapPoint *pMP);
		set<MapPoint*> GetMapPoints();
		vector<MapPoint*> GetMapPointMatches();
		int32_t TrackedMapPoints(const int32_t minObs);
		MapPoint *GetMapPoint(const size_t idx);

		// KeyPoint functions
		vector<size_t> GetFeaturesInArea(const float x, const float y, const float r) const;
		cv::Mat UnprojectStereo(int32_t i);

		// Image
		bool IsInImage(const float x, const float y) const;

		// Enable/Disable bad flag changes
		void SetNotErase();
		void SetErase();

		// Compute Scene Depth (q=2 median). Used in monocular.
		float ComputeSceneMedianDepth(const int32_t q);

		// Set/check bad flag
		void SetBadFlag();
		static bool isBad(KeyFrame *pKF, Map *pMap);

		static inline bool weightComp(int32_t a, int32_t b)
		{
			return a > b;
		}

		static inline bool CompareWithId(KeyFrame *pKF1, KeyFrame *pKF2)
		{
			return pKF1->mnId < pKF2->mnId;
		}

		// for map save/load
		static void SaveKeyFrames(ofstream &f, const set<KeyFrame*> &kfs, Map *pMap);
		static void LoadKeyFrames(ifstream &f, GlobalData *mpGlobalData, const map<uint64_t, MapPoint*> &mMapPointById, vector<KeyFrame*> &kfs);

		// The following variables are accesed from only 1 thread or never change (no mutex needed).
	public:
		GlobalData *mpGlobalData = nullptr;

		//static uint32_t nNextId;
		uint64_t mnId = 0;
		// 生成该关键帧的普通帧id
		uint64_t mnFrameId = 0;

		double mTimeStamp;

		// Grid (to speed up feature matching)
		int32_t mnGridCols;
		int32_t mnGridRows;
		float mfGridElementWidthInv;
		float mfGridElementHeightInv;

		// Variables used by the tracking
		uint64_t mnTrackReferenceForFrame = 0;
		uint64_t mnFuseTargetForKF = 0;

		// Variables used by the local mapping
		uint64_t mnBALocalForKF = 0;
		uint64_t mnBAFixedForKF = 0;

		// Variables used by the keyframe database
		uint64_t mnLoopQuery = 0;
		int32_t mnLoopWords = 0;
		float mLoopScore = 0;
		uint64_t mnRelocQuery = 0;
		int32_t mnRelocWords = 0;
		float mRelocScore = 0;

		// Variables used by loop closing
		cv::Mat mTcwGBA;
		cv::Mat mTcwBefGBA;
		uint64_t mnBAGlobalForKF = 0;

		// Calibration parameters
		float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

		// Number of KeyPoints
		uint32_t N;

		// KeyPoints, stereo coordinate and descriptors (all associated by an index)
		vector<cv::KeyPoint> mvKeys;
		vector<cv::KeyPoint> mvKeysUn;
		vector<float> mvuRight; // negative value for monocular points
		vector<float> mvDepth; // negative value for monocular points
		cv::Mat mDescriptors;

		//BoW
		DBoW3::BowVector mBowVec;
		DBoW3::FeatureVector mFeatVec;

		// Pose relative to parent (this is computed when bad flag is activated)
		cv::Mat mTcp;

		// Scale
		int32_t mnScaleLevels;
		float mfScaleFactor;
		float mfLogScaleFactor;
		vector<float> mvScaleFactors;
		vector<float> mvLevelSigma2;
		vector<float> mvInvLevelSigma2;

		// Image bounds and calibration
		int32_t mnMinX;
		int32_t mnMinY;
		int32_t mnMaxX;
		int32_t mnMaxY;
		cv::Mat mK;


		// The following variables need to be accessed trough a mutex to be thread safe.
	protected:

		// SE3 Pose and camera center
		cv::Mat Tcw;
		cv::Mat Twc;
		cv::Mat Ow;

		cv::Mat Cw; // Stereo middel point. Only for visualization

		// MapPoints associated to keypoints
		vector<MapPoint*> mvpMapPoints;

		// BoW
		KeyFrameDatabase *mpKeyFrameDB = nullptr;
		ORBVocabulary *mpORBvocabulary = nullptr;

		// Grid over the image to speed up feature matching
		vector<vector<vector<uint32_t>>> mGrid;

		map<KeyFrame*, int32_t> mConnectedKeyFrameWeights;
		vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
		vector<int32_t> mvOrderedWeights;

		// Spanning Tree and Loop Edges
		bool mbFirstConnection = true;
		KeyFrame *mpParent = nullptr;
		set<KeyFrame*> mspChildrens;
		set<KeyFrame*> mspLoopEdges;

		// Bad flags
		bool mbNotErase = false;
		bool mbToBeErased = false;
		bool mbBad = false;

		float mHalfBaseline = 0; // Only for visualization

		Map *mpMap = nullptr;

		mutex mMutexPose;
		// 此处改为递归锁，SetBadFlag()中会让Child对自身使用ChangeParent()，此时会造成死锁
		recursive_mutex mMutexConnections;
		// 此处改为递归锁，SetBadFlag()中对MapPoint执行EraseObservation()可能会触发MapPoint::SetBadFlag，然后造成死锁
		recursive_mutex mMutexFeatures;
	};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
