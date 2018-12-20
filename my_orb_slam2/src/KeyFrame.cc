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

#include "DUtils/IoUtils.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "Frame.h"
#include "MapPoint.h"
#include "Converter.h"
#include "Map.h"
#include "ORBVocabulary.h"
#include "GlobalData.h"

namespace ORB_SLAM2
{

	//uint32_t KeyFrame::nNextId = 0;

	KeyFrame::KeyFrame(GlobalData *pGlobalData) :
		mpGlobalData(pGlobalData), mpMap(pGlobalData->mpMap), mpKeyFrameDB(pGlobalData->mpKeyFrameDatabase), mpORBvocabulary(pGlobalData->mpVocabulary)
	{
		mpMap->AddExistKeyFrame(this);
	}

	KeyFrame::KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB) :
		mnFrameId(F.mnId), mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
		mbf(F.mbf), mb(F.mb), mThDepth(F.mThDepth), N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
		mvuRight(F.mvuRight), mvDepth(F.mvDepth), mDescriptors(F.mDescriptors.clone()),
		mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
		mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
		mvInvLevelSigma2(F.mvInvLevelSigma2), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
		mpORBvocabulary(F.mpORBvocabulary), mHalfBaseline(F.mb / 2), mpMap(pMap)
	{
		mpGlobalData = F.mpGlobalData;
		mfGridElementWidthInv = mpGlobalData->fFrameGridElementWidthInv;
		mfGridElementHeightInv = mpGlobalData->fFrameGridElementHeightInv;

		fx = F.fx;
		fy = F.fy;
		cx = F.cx;
		cy = F.cy;
		invfx = F.invfx;
		invfy = F.invfy;

		mnMinX = mpGlobalData->nFrameMinX;
		mnMinY = mpGlobalData->nFrameMinY;
		mnMaxX = mpGlobalData->nFrameMaxX;
		mnMaxY = mpGlobalData->nFrameMaxY;

		mnId = mpGlobalData->nKeyFrameNextId++;

		mGrid.resize(mnGridCols);
		for (int i = 0; i < mnGridCols; i++)
		{
			mGrid[i].resize(mnGridRows);
			for (int j = 0; j < mnGridRows; j++)
				mGrid[i][j] = F.mGrid[i][j];
		}

		SetPose(F.mTcw);

		//// 尝试排除掉bad MapPoint
		//for (auto i = mvpMapPoints.begin(); i != mvpMapPoints.end(); ++i)
		//	if (MapPoint::isBad(*i, mpMap))
		//	{
		//		*i = nullptr;
		//		//cerr << "found bad MapPoint in KeyFrame::KeyFrame" << endl;
		//	}

		mpMap->AddExistKeyFrame(this);
	}

	KeyFrame::~KeyFrame()
	{
		mpMap->EraseExistKeyFrame(this);
	}

	void KeyFrame::ComputeBoW()
	{
		if (mBowVec.empty() || mFeatVec.empty())
		{
			vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
			// Feature vector associate features with nodes in the 4th level (from leaves up)
			// We assume the vocabulary tree has 6 levels, change the 4 otherwise
			mpORBvocabulary->transform(vCurrentDesc, mBowVec, mFeatVec, 4);
		}
	}

	void KeyFrame::SetPose(const cv::Mat &Tcw_)
	{
		unique_lock<mutex> lock(mMutexPose);
		Tcw_.copyTo(Tcw);
		cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
		cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
		cv::Mat Rwc = Rcw.t();
		Ow = -Rwc * tcw;

		Twc = cv::Mat::eye(4, 4, Tcw.type());
		Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
		Ow.copyTo(Twc.rowRange(0, 3).col(3));
		cv::Mat center = (cv::Mat_<float>(4, 1) << mHalfBaseline, 0, 0, 1);
		Cw = Twc * center;
	}

	cv::Mat KeyFrame::GetPose()
	{
		unique_lock<mutex> lock(mMutexPose);
		return Tcw.clone();
	}

	cv::Mat KeyFrame::GetPoseInverse()
	{
		unique_lock<mutex> lock(mMutexPose);
		return Twc.clone();
	}

	cv::Mat KeyFrame::GetCameraCenter()
	{
		unique_lock<mutex> lock(mMutexPose);
		return Ow.clone();
	}

	cv::Mat KeyFrame::GetStereoCenter()
	{
		unique_lock<mutex> lock(mMutexPose);
		return Cw.clone();
	}

	cv::Mat KeyFrame::GetRotation()
	{
		unique_lock<mutex> lock(mMutexPose);
		return Tcw.rowRange(0, 3).colRange(0, 3).clone();
	}

	cv::Mat KeyFrame::GetTranslation()
	{
		unique_lock<mutex> lock(mMutexPose);
		return Tcw.rowRange(0, 3).col(3).clone();
	}

	void KeyFrame::AddConnection(KeyFrame *pKF, const int32_t weight)
	{
		if (KeyFrame::isBad(pKF, mpMap))
		{
			cerr << "Found bad KeyFrame in KeyFrame::AddConnection" << endl;
			return;
		}

		{
			unique_lock<recursive_mutex> lock(mMutexConnections);
			mConnectedKeyFrameWeights[pKF] = weight;
			/*if (!mConnectedKeyFrameWeights.count(pKF))
				mConnectedKeyFrameWeights[pKF] = weight;
			else if (mConnectedKeyFrameWeights[pKF] != weight)
				mConnectedKeyFrameWeights[pKF] = weight;
			else
				return;*/
		}

		UpdateBestCovisibles();
	}

	void KeyFrame::UpdateBestCovisibles()
	{
		/*这里干了
		将所有关联帧按照weight，从小到大排序
		然后分别有序储存在 mvpOrderedConnectedKeyFrames 和 mvOrderedWeights 中
		*/
		unique_lock<recursive_mutex> lock(mMutexConnections);
		vector<pair<int, KeyFrame*>> vPairs;
		vPairs.reserve(mConnectedKeyFrameWeights.size());
		for (map<KeyFrame*, int>::iterator mit = mConnectedKeyFrameWeights.begin(); mit != mConnectedKeyFrameWeights.end(); ++mit)
			vPairs.push_back(make_pair(mit->second, mit->first));

		sort(vPairs.begin(), vPairs.end());
		list<KeyFrame*> lKFs;
		list<int> lWs;
		for (size_t i = 0; i < vPairs.size(); ++i)
		{
			lKFs.push_front(vPairs[i].second);
			lWs.push_front(vPairs[i].first);
		}

		mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(), lKFs.end());
		mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());
	}

	set<KeyFrame*> KeyFrame::GetConnectedKeyFrames()
	{
		unique_lock<recursive_mutex> lock(mMutexConnections);
		set<KeyFrame*> s;
		for (map<KeyFrame*, int>::iterator mit = mConnectedKeyFrameWeights.begin(); mit != mConnectedKeyFrameWeights.end(); ++mit)
			s.insert(mit->first);
		return s;
	}

	vector<KeyFrame*> KeyFrame::GetVectorCovisibleKeyFrames()
	{
		unique_lock<recursive_mutex> lock(mMutexConnections);
		return mvpOrderedConnectedKeyFrames;
	}

	vector<KeyFrame*> KeyFrame::GetBestCovisibilityKeyFrames(const int32_t N)
	{
		unique_lock<recursive_mutex> lock(mMutexConnections);
		if ((int)mvpOrderedConnectedKeyFrames.size() <= N)
			return mvpOrderedConnectedKeyFrames;
		else
			return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + N);
	}

	vector<KeyFrame*> KeyFrame::GetCovisiblesByWeight(const int32_t w)
	{
		unique_lock<recursive_mutex> lock(mMutexConnections);

		if (mvpOrderedConnectedKeyFrames.empty())
			return vector<KeyFrame*>();

		vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(), mvOrderedWeights.end(), w, KeyFrame::weightComp);
		if (it == mvOrderedWeights.end())
			return vector<KeyFrame*>();
		else
		{
			int n = it - mvOrderedWeights.begin();
			return vector<KeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin() + n);
		}
	}

	int32_t KeyFrame::GetWeight(KeyFrame *pKF)
	{
		unique_lock<recursive_mutex> lock(mMutexConnections);
		if (mConnectedKeyFrameWeights.count(pKF))
			return mConnectedKeyFrameWeights[pKF];
		else
			return 0;
	}

	void KeyFrame::AddMapPoint(MapPoint *pMP, const size_t idx)
	{
		if (MapPoint::isBad(pMP, mpMap))
		{
			cerr << "Found bad MapPoint in KeyFrame::AddMapPoint" << endl;
			return;
		}
		unique_lock<recursive_mutex> lock(mMutexFeatures);
		mvpMapPoints[idx] = pMP;
	}

	void KeyFrame::EraseMapPointMatch(const size_t idx)
	{
		unique_lock<recursive_mutex> lock(mMutexFeatures);
		mvpMapPoints[idx] = nullptr;
	}

	void KeyFrame::EraseMapPointMatch(MapPoint *pMP)
	{
		//增加代码，如果pMP是无效的，则在自身中查找
		if (MapPoint::isBad(pMP, mpMap))
		{
			unique_lock<recursive_mutex> lock(mMutexFeatures);
			for (size_t i = 0; i < N; ++i)
				if (mvpMapPoints[i] == pMP)
					mvpMapPoints[i] = nullptr;
		}
		else
		{
			unique_lock<recursive_mutex> lock(mMutexFeatures);
			int idx = pMP->GetIndexInKeyFrame(this);
			if (idx >= 0)
				mvpMapPoints[idx] = nullptr;
		}
	}

	void KeyFrame::ReplaceMapPointMatch(const size_t idx, MapPoint *pMP)
	{
		unique_lock<recursive_mutex> lock(mMutexFeatures);
		mvpMapPoints[idx] = pMP;
	}

	set<MapPoint*> KeyFrame::GetMapPoints()
	{
		unique_lock<recursive_mutex> lock(mMutexFeatures);

		//此段代码很少使用，并且用到的时候也有坏点检查
		//set<MapPoint*> s;
		//int nBadPoint = 0;
		//for (size_t i = 0; i < N; ++i)
		//{
		//	MapPoint *pMP = mvpMapPoints[i];

		//	if (!pMP)
		//		continue;
		//	if (!MapPoint::isBad(pMP, mpMap))
		//		s.insert(pMP);
		//	else
		//	{
		//		mvpMapPoints[i] = nullptr;
		//		++nBadPoint;
		//	}
		//}

		//if (nBadPoint)
		//	cerr << "Found " << nBadPoint << " bad MapPoint in KeyFrame::GetMapPoints" << endl;
		set<MapPoint*> s(mvpMapPoints.begin(), mvpMapPoints.end());
		s.erase(nullptr);

		return s;
	}

	int KeyFrame::TrackedMapPoints(const int32_t minObs)
	{
		// 返回自身所有 观察者数量大于等于minObs的地图点中的数量
		unique_lock<recursive_mutex> lock(mMutexFeatures);

		int nPoints = 0;
		const bool bCheckObs = minObs > 0;

		int nBadObserver = 0;

		for (int i = 0; i < N; ++i)
		{
			MapPoint *pMP = mvpMapPoints[i];
			if (pMP)
			{
				if (!MapPoint::isBad(pMP, mpMap))
				{
					if (bCheckObs)
					{
						if (mvpMapPoints[i]->Observations() >= minObs)
							nPoints++;
					}
					else
						nPoints++;
				}
				else
				{
					mvpMapPoints[i] = nullptr;
					++nBadObserver;
					//throw;
				}
			}
		}

		if (nBadObserver)
			cerr << "Found " << nBadObserver << " bad MapPoint in KeyFrame::TrackedMapPoints" << endl;

		return nPoints;
	}

	vector<MapPoint*> KeyFrame::GetMapPointMatches()
	{
		unique_lock<recursive_mutex> lock(mMutexFeatures);
		return mvpMapPoints;
	}

	MapPoint *KeyFrame::GetMapPoint(const size_t idx)
	{
		unique_lock<recursive_mutex> lock(mMutexFeatures);
		return mvpMapPoints[idx];
	}

	void KeyFrame::UpdateConnections()
	{
		/*此函数干了
		根据与其他关键帧共享的地图点，设置了自己与其他关键帧的链接
		当有任何关键帧与此关键帧共享的地图点超过阀值，将他们全部添加到mConnectedKeyFrameWeights内，
			并且在排序后输入mvpOrderedConnectedKeyFrames和mvOrderedWeights
		如果没有任何关键帧与此关键帧共享的地图点的数目超过阀值，则只添加一条与此关键帧拥有最大共享地图点数目的链接
		在 mbFirstConnection 为真 和 不是第0帧的情况下，设置此帧parent Kf，然后在parent Kf的 child 表中加入自己。
		parent Kf 由拥有最小共享地图点数目的帧担任。
		*/

		// 其他关键帧与本关键帧共享的地图点数量
		// KeyFrame *其他关键帧，int 共享地图点数量
		map<KeyFrame*, int> KFcounter;

		vector<MapPoint*> vpMP;
		vpMP.reserve(mvpMapPoints.size());

		{
			unique_lock<recursive_mutex> lockMPs(mMutexFeatures);
			int nBadPoint = 0;
			for (int i = 0; i < N; ++i)
				if (mvpMapPoints[i])
					if (!MapPoint::isBad(mvpMapPoints[i], mpMap))
						vpMP.push_back(mvpMapPoints[i]);
					else
					{
						mvpMapPoints[i] = nullptr;
						++nBadPoint;
					}
			if (nBadPoint)
				cerr << "Found " << nBadPoint << " bad MapPoint in KeyFrame::UpdateConnections" << endl;
		}

		//For all map points in keyframe check in which other keyframes are they seen
		//Increase counter for those keyframes
		int nBadObserver = 0;
		for (vector<MapPoint*>::iterator vit = vpMP.begin(); vit != vpMP.end(); ++vit)
		{
			MapPoint *pMP = *vit;

			//if (!pMP)
			//	continue;

			//if (MapPoint::isBad(pMP, mpMap))
			//{
			//	cerr << "found bad MapPoint in KeyFrame::UpdateConnections" << endl;
			//	//throw;
			//	continue;
			//}

			map<KeyFrame*, size_t> observations = pMP->GetObservations();

			for (map<KeyFrame*, size_t>::iterator mit = observations.begin(); mit != observations.end(); ++mit)
			{
				if (KeyFrame::isBad(mit->first, mpMap))
				{
					pMP->EraseObservation(mit->first);
					++nBadObserver;
					//throw;
					continue;
				}

				if (mit->first->mnId == mnId)
					continue;

				KFcounter[mit->first]++;
			}
		}
		if (nBadObserver)
			cerr << "Found " << nBadObserver << " bad KF Observer in a good MapPoint when KeyFrame::UpdateConnections" << endl;

		// This should not happen
		if (KFcounter.empty())
		{
			cerr << "No any MapPoint share with other keyframe. This should not happen" << endl;
			return;
		}

		//If the counter is greater than threshold add connection
		//In case no keyframe counter is over threshold add the one with maximum counter
		// 如果计数器大于阈值则添加连接
		// 如果没有关键帧计数器超过阈值，则添加具有一个拥有最大计数器的链接
		int nmax = 0;
		KeyFrame *pKFmax = nullptr;
		int th = 15;

		vector<pair<int, KeyFrame*>> vPairs;
		vPairs.reserve(KFcounter.size());
		for (map<KeyFrame*, int>::iterator mit = KFcounter.begin(); mit != KFcounter.end(); ++mit)
		{
			if (mit->second > nmax)
			{
				nmax = mit->second;
				pKFmax = mit->first;
			}
			if (mit->second >= th)
			{
				vPairs.push_back(make_pair(mit->second, mit->first));
				(mit->first)->AddConnection(this, mit->second);
			}
		}

		if (vPairs.empty())
		{
			vPairs.push_back(make_pair(nmax, pKFmax));
			pKFmax->AddConnection(this, nmax);
		}

		sort(vPairs.begin(), vPairs.end());
		list<KeyFrame*> lKFs;
		list<int> lWs;
		for (size_t i = 0; i < vPairs.size(); i++)
		{
			lKFs.push_front(vPairs[i].second);
			lWs.push_front(vPairs[i].first);
		}

		{
			unique_lock<recursive_mutex> lockCon(mMutexConnections);

			// mspConnectedKeyFrames = spConnectedKeyFrames;
			mConnectedKeyFrameWeights = KFcounter;
			mvpOrderedConnectedKeyFrames = vector<KeyFrame*>(lKFs.begin(), lKFs.end());
			mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

			if (mbFirstConnection && mnId != 0)
			{
				mpParent = mvpOrderedConnectedKeyFrames.front();
				mpParent->AddChild(this);
				mbFirstConnection = false;
			}

		}
	}

	void KeyFrame::AddChild(KeyFrame *pKF)
	{
		if (KeyFrame::isBad(pKF, mpMap))
		{
			cerr << "Found bad KeyFrame in KeyFrame::AddChild" << endl;
			return;
		}
		unique_lock<recursive_mutex> lockCon(mMutexConnections);
		mspChildrens.insert(pKF);
	}

	void KeyFrame::EraseChild(KeyFrame *pKF)
	{
		unique_lock<recursive_mutex> lockCon(mMutexConnections);
		mspChildrens.erase(pKF);
	}

	void KeyFrame::ChangeParent(KeyFrame *pKF)
	{
		//if (KeyFrame::isBad(pKF, mpMap))
		//{
		//	cerr << "Found bad KeyFrame in KeyFrame::ChangeParent" << endl;
		//	return;
		//}
		//unique_lock<recursive_mutex> lockCon(mMutexConnections);
		//// 我增加的代码，改变Parent后，Parent现在将会删除当前关键帧链接
		//if (mpParent)
		//	mpParent->EraseChild(this);
		mpParent = pKF;
		//pKF->AddChild(this);
	}

	set<KeyFrame*> KeyFrame::GetChilds()
	{
		unique_lock<recursive_mutex> lockCon(mMutexConnections);
		return mspChildrens;
	}

	KeyFrame* KeyFrame::GetParent()
	{
		unique_lock<recursive_mutex> lockCon(mMutexConnections);
		return mpParent;
	}

	bool KeyFrame::hasChild(KeyFrame *pKF)
	{
		unique_lock<recursive_mutex> lockCon(mMutexConnections);
		return mspChildrens.count(pKF);
	}

	void KeyFrame::AddLoopEdge(KeyFrame *pKF)
	{
		if (KeyFrame::isBad(pKF, mpMap))
		{
			cerr << "Found bad KeyFrame in KeyFrame::AddLoopEdge" << endl;
			return;
		}
		unique_lock<recursive_mutex> lockCon(mMutexConnections);
		mbNotErase = true;
		mspLoopEdges.insert(pKF);
	}

	set<KeyFrame*> KeyFrame::GetLoopEdges()
	{
		unique_lock<recursive_mutex> lockCon(mMutexConnections);
		return mspLoopEdges;
	}

	void KeyFrame::SetNotErase()
	{
		unique_lock<recursive_mutex> lock(mMutexConnections);
		mbNotErase = true;
	}

	void KeyFrame::SetErase()
	{
		{
			unique_lock<recursive_mutex> lock(mMutexConnections);
			if (mspLoopEdges.empty())
			{
				mbNotErase = false;
			}
		}

		if (mbToBeErased)
		{
			SetBadFlag();
		}
	}

	void KeyFrame::SetBadFlag()
	{
		if (mnId == 0)
			return;
		{
			unique_lock<recursive_mutex> lock(mMutexConnections);

			if (mbNotErase)
			{
				mbToBeErased = true;
				return;
			}

			int nBadConnection = 0;

			// 删除与此关键帧有关的任何链接
			for (map<KeyFrame*, int>::iterator mit = mConnectedKeyFrameWeights.begin(); mit != mConnectedKeyFrameWeights.end(); ++mit)
				if (!KeyFrame::isBad(mit->first, mpMap))
					mit->first->EraseConnection(this);
				else
					++nBadConnection;

			if (nBadConnection)
				cerr << "Found " << nBadConnection << " bad KeyFrame connection in KeyFrame::SetBadFlag" << endl;

			mConnectedKeyFrameWeights.clear();
			mvpOrderedConnectedKeyFrames.clear();
		}
		
		{
			//unique_lock<recursive_mutex> lock2(mMutexFeatures);

			int nBadPoint = 0;

			// 在所有地图点中删除自己
			for (size_t i = 0; i < N; ++i)
				if (mvpMapPoints[i])
				{
					if (!MapPoint::isBad(mvpMapPoints[i], mpMap))
					{
						mvpMapPoints[i]->EraseObservation(this);
						mvpMapPoints[i] = nullptr;
					}
					else
					{
						++nBadPoint;
						mvpMapPoints[i] = nullptr;
					}
				}

			if (nBadPoint)
				cerr << "Found " << nBadPoint << " bad MapPoint in KeyFrame::SetBadFlag" << endl;
		}

		mbBad = true;

		{
			unique_lock<recursive_mutex> lock2(mMutexFeatures);
			unique_lock<recursive_mutex> lockCon(mMutexConnections);

			if (KeyFrame::isBad(mpParent, mpMap))
			{
				cerr << "Found parent KeyFrame is bad! Now will choose an id that is smaller than myself as the parent KeyFrame. in KeyFrame::SetBadFlag" << endl;
				vector<KeyFrame*> kfs = mpMap->GetAllKeyFrames();
				sort(kfs.begin(), kfs.end(), KeyFrame::CompareWithId);
				vector<KeyFrame*>::iterator it = find(kfs.begin(), kfs.end(), this);
				mpParent = nullptr;
				while (!mpParent)
				{
					--it;
					if (!KeyFrame::isBad(*it, mpMap))
						mpParent = *it;
				}
			}
			// Update Spanning Tree
			set<KeyFrame*> sParentCandidates;
			sParentCandidates.insert(mpParent);

			// Assign at each iteration one children with a parent (the pair with highest covisibility weight)
			// Include that children as new parent candidate for the rest
			while (!mspChildrens.empty())
			{
				bool bContinue = false;

				int max = -1;
				KeyFrame *pC;
				KeyFrame *pP;

				int nBadChild = 0;

				for (set<KeyFrame*>::iterator sit = mspChildrens.begin(); sit != mspChildrens.end();)
				{
					KeyFrame *pKF = *sit;
					if (KeyFrame::isBad(pKF, mpMap))
					{
						sit = mspChildrens.erase(sit);
						++nBadChild;
						continue;
					}

					// Check if a parent candidate is connected to the keyframe
					vector<KeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();

					for (size_t i = 0; i < vpConnected.size(); ++i)
					{
						for (set<KeyFrame*>::iterator spcit = sParentCandidates.begin(); spcit != sParentCandidates.end(); ++spcit)
						{
							if (vpConnected[i]->mnId == (*spcit)->mnId)
							{
								int w = pKF->GetWeight(vpConnected[i]);
								if (w > max)
								{
									pC = pKF;
									pP = vpConnected[i];
									max = w;
									bContinue = true;
								}
							}
						}
					}
					++sit;
				}

				if (nBadChild)
					cerr << "Found " << nBadChild << " bad child KeyFrame in KeyFrame::SetBadFlag";

				if (bContinue)
				{
					pC->ChangeParent(pP);
					pP->AddChild(pC);
					sParentCandidates.insert(pC);
					mspChildrens.erase(pC);
				}
				else
					break;
			}

			// If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
			if (!mspChildrens.empty())
				for (set<KeyFrame*>::iterator sit = mspChildrens.begin(); sit != mspChildrens.end(); ++sit)
				{
					(*sit)->ChangeParent(mpParent);
					mpParent->AddChild(*sit);
				}

			mpParent->EraseChild(this);
			mTcp = Tcw * mpParent->GetPoseInverse();
		}

		mpKeyFrameDB->erase(this);
		mpMap->EraseKeyFrame(this);
	}

	bool KeyFrame::isBad(KeyFrame *pKF, Map *pMap)
	{
		return pMap->IsBadKeyFrame(pKF);
	}

	void KeyFrame::EraseConnection(KeyFrame *pKF)
	{
		bool bUpdate = false;
		{
			unique_lock<recursive_mutex> lock(mMutexConnections);
			if (mConnectedKeyFrameWeights.count(pKF))
			{
				mConnectedKeyFrameWeights.erase(pKF);
				bUpdate = true;
			}
		}

		if (bUpdate)
			UpdateBestCovisibles();
	}

	vector<size_t> KeyFrame::GetFeaturesInArea(const float x, const float y, const float r) const
	{
		vector<size_t> vIndices;
		vIndices.reserve(N);

		const int nMinCellX = max(0, (int)floor((x - mnMinX - r)*mfGridElementWidthInv));
		if (nMinCellX >= mnGridCols)
			return vIndices;

		const int nMaxCellX = min((int)mnGridCols - 1, (int)ceil((x - mnMinX + r)*mfGridElementWidthInv));
		if (nMaxCellX < 0)
			return vIndices;

		const int nMinCellY = max(0, (int)floor((y - mnMinY - r)*mfGridElementHeightInv));
		if (nMinCellY >= mnGridRows)
			return vIndices;

		const int nMaxCellY = min((int)mnGridRows - 1, (int)ceil((y - mnMinY + r)*mfGridElementHeightInv));
		if (nMaxCellY < 0)
			return vIndices;

		for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
		{
			for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
			{
				const vector<uint32_t> vCell = mGrid[ix][iy];
				for (size_t j = 0, jend = vCell.size(); j < jend; j++)
				{
					const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
					const float distx = kpUn.pt.x - x;
					const float disty = kpUn.pt.y - y;

					if (fabs(distx) < r && fabs(disty) < r)
						vIndices.push_back(vCell[j]);
				}
			}
		}

		return vIndices;
	}

	bool KeyFrame::IsInImage(const float x, const float y) const
	{
		return (x >= mnMinX && x < mnMaxX && y >= mnMinY && y < mnMaxY);
	}

	cv::Mat KeyFrame::UnprojectStereo(int32_t i)
	{
		const float z = mvDepth[i];
		if (z > 0)
		{
			const float u = mvKeys[i].pt.x;
			const float v = mvKeys[i].pt.y;
			const float x = (u - cx)*z*invfx;
			const float y = (v - cy)*z*invfy;
			cv::Mat x3Dc = (cv::Mat_<float>(3, 1) << x, y, z);

			unique_lock<mutex> lock(mMutexPose);
			return Twc.rowRange(0, 3).colRange(0, 3)*x3Dc + Twc.rowRange(0, 3).col(3);
		}
		else
			return cv::Mat();
	}

	float KeyFrame::ComputeSceneMedianDepth(const int32_t q)
	{
		vector<MapPoint*> vpMapPoints;
		cv::Mat Tcw_;
		{
			unique_lock<recursive_mutex> lock(mMutexFeatures);
			unique_lock<mutex> lock2(mMutexPose);

			vpMapPoints = mvpMapPoints;
			vpMapPoints.resize(mvpMapPoints.size(), nullptr);

			int nBadPoint = 0;

			for (auto i = 0; i < N; ++i)
				if (mvpMapPoints[i])
					if (!MapPoint::isBad(mvpMapPoints[i], mpMap))
						vpMapPoints[i] = mvpMapPoints[i];
					else
					{
						mvpMapPoints[i] = nullptr;
						++nBadPoint;
					}

			if (nBadPoint)
				cerr << "Found " << nBadPoint << " bad MapPoint in KeyFrame::ComputeSceneMedianDepth" << endl;

			Tcw_ = Tcw.clone();
		}

		vector<float> vDepths;
		vDepths.reserve(N);
		cv::Mat Rcw2 = Tcw_.row(2).colRange(0, 3);
		Rcw2 = Rcw2.t();
		float zcw = Tcw_.at<float>(2, 3);
		for (int i = 0; i < N; i++)
		{
			if (mvpMapPoints[i])
			{
				MapPoint *pMP = mvpMapPoints[i];
				cv::Mat x3Dw = pMP->GetWorldPos();
				float z = Rcw2.dot(x3Dw) + zcw;
				vDepths.push_back(z);
			}
		}

		sort(vDepths.begin(), vDepths.end());

		return vDepths[(vDepths.size() - 1) / q];
	}

//#pragma optimize("", off)
	void KeyFrame::SaveKeyFrames(ofstream &f, const set<KeyFrame*> &kfs, Map *pMap)
	{
		set<KeyFrame*> skfs;
		for (auto i : kfs)
			if (!KeyFrame::isBad(i, pMap))
				skfs.insert(i);

		// 保存关键帧数量
		uint64_t nKeyFrames = skfs.size();
		IoUtils::Write(f, nKeyFrames);
		cout << "The number of KeyFrames:" << nKeyFrames << endl;

		for (auto kf : skfs)
		{
			//保存当前关键帧的ID，时间戳
			IoUtils::Write(f, kf->mnId);
			IoUtils::Write(f, kf->mnFrameId);
			IoUtils::Write(f, kf->mTimeStamp);

			//
			IoUtils::Write(f, kf->mnGridCols);
			IoUtils::Write(f, kf->mnGridRows);
			IoUtils::Write(f, kf->mfGridElementWidthInv);
			IoUtils::Write(f, kf->mfGridElementHeightInv);

			//
			IoUtils::Write(f, kf->fx);
			IoUtils::Write(f, kf->fy);
			IoUtils::Write(f, kf->cx);
			IoUtils::Write(f, kf->cy);
			IoUtils::Write(f, kf->invfx);
			IoUtils::Write(f, kf->invfy);
			IoUtils::Write(f, kf->mbf);
			IoUtils::Write(f, kf->mb);
			IoUtils::Write(f, kf->mThDepth);

			//保存当前关键帧包含的ORB特征数目
			IoUtils::Write(f, kf->N);

			//保存特征点
			IoUtils::WriteKeyPoints(f, kf->mvKeys);
			IoUtils::WriteKeyPoints(f, kf->mvKeysUn);

			//
			IoUtils::WriteFloatVector(f, kf->mvuRight);
			IoUtils::WriteFloatVector(f, kf->mvDepth);

			//保存特征点的描述符
			IoUtils::WriteMat(f, kf->mDescriptors);

			IoUtils::Write(f, kf->mnScaleLevels);
			IoUtils::Write(f, kf->mfScaleFactor);
			IoUtils::Write(f, kf->mfLogScaleFactor);

			IoUtils::WriteFloatVector(f, kf->mvScaleFactors);
			IoUtils::WriteFloatVector(f, kf->mvLevelSigma2);
			IoUtils::WriteFloatVector(f, kf->mvInvLevelSigma2);

			IoUtils::Write(f, kf->mnMinX);
			IoUtils::Write(f, kf->mnMinY);
			IoUtils::Write(f, kf->mnMaxX);
			IoUtils::Write(f, kf->mnMaxY);
			//IoUtils::WriteMat(f, kf->mK);
			
			IoUtils::WriteUInt32Vector3s(f, kf->mGrid);

			//保存当前关键帧的位姿矩阵
			//IoUtils::WriteMat(f, kf->GetPose());
			IoUtils::WriteMat(f, kf->Tcw);

			//保存每一个ORB特征点对应的地图点编号
			for (uint64_t i = 0; i < kf->N; ++i)
			{
				//保存当前ORB特征点对应的MapPoint的索引值
				//不是每一个ORB特征点都有对应的MapPoint
				uint64_t mnId;
				//MapPoint *mp = kf->GetMapPoint(i);
				MapPoint *mp = kf->mvpMapPoints[i];
				//排除无效地图点
				mnId = MapPoint::isBad(mp, pMap) ? -1 : mp->mnId;
				IoUtils::Write(f, mnId);
			}

			//保存父关键帧id
			if (kf->mpParent && KeyFrame::isBad(kf->mpParent, pMap))
			{
				cerr << "bad parent" << endl;
				throw;
			}
			uint64_t parentId = kf->mpParent ? kf->mpParent->mnId : -1;
			IoUtils::Write(f, parentId);

			//保存关联关键帧信息
			auto connectedKeyFrames = kf->GetConnectedKeyFrames();
			//排除无效关键帧
			for (auto it = connectedKeyFrames.begin(); it != connectedKeyFrames.end();)
			{
				// 删除元素后已经返回了下一个元素，此时如果++it则会跳过该元素，造成漏检，所以将++it移动至else分支
				if (KeyFrame::isBad((*it), pMap))
					it = connectedKeyFrames.erase(it);
				else
					++it;
			}
			//保存关联关键帧的数量
			uint64_t nConnectedKf = connectedKeyFrames.size();
			IoUtils::Write(f, nConnectedKf);
			//保存关联关键帧的id，权重
			for (KeyFrame *connectedKf : connectedKeyFrames)
			{
				// 检查该关联帧是否在关键帧列表内，如果不在则说明系统存在错误：存在 有效关键帧 不在 pMap的关键帧集合内
				if (!skfs.count(connectedKf))
				{
					cout << "Found a valid keyframe not in keyframe list!!! System Error!" << endl;
					throw;
				}

				IoUtils::Write(f, connectedKf->mnId);
				IoUtils::Write(f, kf->GetWeight(connectedKf));
			}
		}
	}
//#pragma optimize("", on)

//#pragma optimize("", off)
	void KeyFrame::LoadKeyFrames(ifstream &f, GlobalData *mpGlobalData, const map<uint64_t, MapPoint*> &mMapPointById, vector<KeyFrame*> &kfs)
	{
		kfs.clear();

		// 读取关键帧数量
		uint64_t nKeyFrames = 0;
		IoUtils::Read(f, nKeyFrames);
		cout << "The number of KeyFrames:" << nKeyFrames << endl;

		//加载关键帧需要两步
		//第一步是初始化关键帧基本信息
		//第二步是设置关键帧之间的关联信息

		//关键帧id 到 关键帧地址的映射
		map<uint64_t, KeyFrame*> mapKeyFrameById;
		//关键帧id 到 关键帧Parent的映射
		map<uint64_t, uint64_t> mapKFParentByKfId;
		//关键帧id 到 关联关键帧ids_weight的映射
		map<uint64_t, vector<pair<uint64_t, int32_t>>> mapConnectedKfIdWeightByKfId;

		// 第一步
		for (uint64_t i = 0; i < nKeyFrames; ++i)
		{
			KeyFrame *kf = new KeyFrame(mpGlobalData);

			//读取当前关键帧的ID，时间戳
			IoUtils::Read(f, kf->mnId);
			IoUtils::Read(f, kf->mnFrameId);
			IoUtils::Read(f, kf->mTimeStamp);
			mapKeyFrameById[kf->mnId] = kf;

			//
			IoUtils::Read(f, kf->mnGridCols);
			IoUtils::Read(f, kf->mnGridRows);
			IoUtils::Read(f, kf->mfGridElementWidthInv);
			IoUtils::Read(f, kf->mfGridElementHeightInv);

			//
			IoUtils::Read(f, kf->fx);
			IoUtils::Read(f, kf->fy);
			IoUtils::Read(f, kf->cx);
			IoUtils::Read(f, kf->cy);
			IoUtils::Read(f, kf->invfx);
			IoUtils::Read(f, kf->invfy);
			IoUtils::Read(f, kf->mbf);
			IoUtils::Read(f, kf->mb);
			IoUtils::Read(f, kf->mThDepth);
			kf->mK = Converter::makeCameraMatrix(kf->fx, kf->fy, kf->cx, kf->cy);

			//读取当前关键帧包含的ORB特征数目
			IoUtils::Read(f, kf->N);

			//读取特征点
			IoUtils::ReadKeyPoints(f, kf->mvKeys);
			IoUtils::ReadKeyPoints(f, kf->mvKeysUn);

			//
			IoUtils::ReadFloatVector(f, kf->mvuRight);
			IoUtils::ReadFloatVector(f, kf->mvDepth);

			//读取特征点的描述符
			IoUtils::ReadMat(f, kf->mDescriptors);

			IoUtils::Read(f, kf->mnScaleLevels);
			IoUtils::Read(f, kf->mfScaleFactor);
			IoUtils::Read(f, kf->mfLogScaleFactor);

			IoUtils::ReadFloatVector(f, kf->mvScaleFactors);
			IoUtils::ReadFloatVector(f, kf->mvLevelSigma2);
			IoUtils::ReadFloatVector(f, kf->mvInvLevelSigma2);

			IoUtils::Read(f, kf->mnMinX);
			IoUtils::Read(f, kf->mnMinY);
			IoUtils::Read(f, kf->mnMaxX);
			IoUtils::Read(f, kf->mnMaxY);
			//IoUtils::ReadMat(f, kf->mK);

			IoUtils::ReadUInt32Vector3s(f, kf->mGrid);

			//读取当前关键帧的位姿矩阵
			cv::Mat pos;
			IoUtils::ReadMat(f, pos);
			kf->SetPose(pos);

			//读取每一个ORB特征点对应的地图点编号
			kf->mvpMapPoints.resize(kf->N, nullptr);
			for (uint64_t i = 0; i < kf->N; ++i)
			{
				//读取当前ORB特征点对应的MapPoint的索引值
				//不是每一个ORB特征点都有对应的MapPoint
				uint64_t mnId;
				IoUtils::Read(f, mnId);
				if (mnId != -1)
				{
					kf->AddMapPoint(mMapPointById.at(mnId), i);

					//设定该地图点 被 此关键帧 观察着
					mMapPointById.at(mnId)->AddObservation(kf, i);
					//如果该地图点没有被任意关键帧引用，则设定被此关键帧引用
					if (!mMapPointById.at(mnId)->GetReferenceKeyFrame())
						mMapPointById.at(mnId)->SetReferenceKeyFrame(kf);
				}
			}

			// 第二步准备
			//保存父关键帧id
			uint64_t parentId;
			IoUtils::Read(f, parentId);
			if (parentId != -1)
			{
				mapKFParentByKfId[kf->mnId] = parentId;
				kf->mbFirstConnection = false;
			}

			//保存关联关键帧的数量
			uint64_t nConnectedKf;
			IoUtils::Read(f, nConnectedKf);
			//保存关联关键帧的id，权重
			for (uint64_t i = 0; i < nConnectedKf; ++i)
			{
				uint64_t conKfId;
				int32_t weight;
				IoUtils::Read(f, conKfId);
				IoUtils::Read(f, weight);
				mapConnectedKfIdWeightByKfId[kf->mnId].push_back(pair<uint64_t, int32_t>(conKfId, weight));
			}
		}

		//第二步

		//设定父id
		for (auto id_parentId : mapKFParentByKfId)
		{
			auto* pKfChild = mapKeyFrameById.at(id_parentId.first);
			auto* pKfParent = mapKeyFrameById.at(id_parentId.second);
			pKfChild->ChangeParent(pKfParent);
			pKfParent->AddChild(pKfChild);
		}

		//设定关联帧
		for (auto id_conKfIdWeight : mapConnectedKfIdWeightByKfId)
		{
			for (auto kfId_weight : id_conKfIdWeight.second)
			{
				mapKeyFrameById.at(id_conKfIdWeight.first)->AddConnection(mapKeyFrameById.at(kfId_weight.first), kfId_weight.second);
			}
		}

		for (auto id_kf : mapKeyFrameById)
		{
			auto kf = id_kf.second;
			kf->ComputeBoW();
			kfs.push_back(kf);
		}

	}
//#pragma optimize("", on)

} //namespace ORB_SLAM
