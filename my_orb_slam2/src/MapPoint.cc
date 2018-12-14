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
#include "MapPoint.h"
#include "ORBmatcher.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "Map.h"
#include "GlobalData.h"

namespace ORB_SLAM2
{

	//uint32_t MapPoint::nNextId = 0;
	//mutex MapPoint::mGlobalMutex;

	MapPoint::MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, GlobalData *pGlobalData) :
		mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), mpRefKF(pRefKF), mpGlobalData(pGlobalData), mpMap(pGlobalData->mpMap)
	{
		Pos.copyTo(mWorldPos);
		mNormalVector = cv::Mat::zeros(3, 1, CV_32F);

		// MapPoints can be created from Tracking and Local Mapping. This recursive_mutex avoid conflicts with id.
		//unique_lock<recursive_mutex> lock(mpMap->mMutexPointCreation);
		mnId = pGlobalData->nMapPointNextId++;

		mpMap->AddExistMapPoint(this);
	}

	MapPoint::MapPoint(const cv::Mat &Pos, GlobalData *pGlobalData, Frame *pFrame, const int32_t idxF) :
		mnFirstFrame(pFrame->mnId), mpGlobalData(pGlobalData), mpMap(pGlobalData->mpMap)
	{
		// 这里用于生成临时点
		Pos.copyTo(mWorldPos);
		cv::Mat Ow = pFrame->GetCameraCenter();
		mNormalVector = mWorldPos - Ow;
		mNormalVector = mNormalVector / cv::norm(mNormalVector);

		cv::Mat PC = Pos - Ow;
		const float dist = cv::norm(PC);
		const int level = pFrame->mvKeysUn[idxF].octave;
		const float levelScaleFactor = pFrame->mvScaleFactors[level];
		const int nLevels = pFrame->mnScaleLevels;

		mfMaxDistance = dist * levelScaleFactor;
		mfMinDistance = mfMaxDistance / pFrame->mvScaleFactors[nLevels - 1];

		pFrame->mDescriptors.row(idxF).copyTo(mDescriptor);

		// MapPoints can be created from Tracking and Local Mapping. This recursive_mutex avoid conflicts with id.
		//unique_lock<recursive_mutex> lock(mpMap->mMutexPointCreation);
		mnId = pGlobalData->nMapPointNextId++;

		mpMap->AddExistMapPoint(this);
	}

	MapPoint::MapPoint(const cv::Mat &Pos, GlobalData *pGlobalData, uint64_t nid) :
		mpGlobalData(pGlobalData), mpMap(pGlobalData->mpMap)
	{
		Pos.copyTo(mWorldPos);
		mnId = nid;
		mNormalVector = cv::Mat::zeros(3, 1, CV_32F);

		// MapPoints can be created from Tracking and Local Mapping. This recursive_mutex avoid conflicts with id.
		//unique_lock<recursive_mutex> lock(mpMap->mMutexPointCreation);

		mpMap->AddExistMapPoint(this);
	}

	MapPoint::~MapPoint()
	{
		mpMap->EraseExistMapPoint(this);
	}

	void MapPoint::SetWorldPos(const cv::Mat &Pos)
	{
		unique_lock<mutex> lock(mpGlobalData->mutexMapPointGlobalPos);
		unique_lock<recursive_mutex> lock2(mMutexPos);
		Pos.copyTo(mWorldPos);
	}

	cv::Mat MapPoint::GetWorldPos()
	{
		unique_lock<recursive_mutex> lock(mMutexPos);
		return mWorldPos.clone();
	}

	cv::Mat MapPoint::GetNormal()
	{
		unique_lock<recursive_mutex> lock(mMutexPos);
		return mNormalVector.clone();
	}

	KeyFrame *MapPoint::GetReferenceKeyFrame()
	{
		unique_lock<recursive_mutex> lock(mMutexFeatures);
		return mpRefKF;
	}

	void MapPoint::SetReferenceKeyFrame(KeyFrame *refKf)
	{
		unique_lock<recursive_mutex> lock(mMutexFeatures);
		mpRefKF = refKf;
	}

	void MapPoint::AddObservation(KeyFrame *pKF, size_t idx)
	{
		unique_lock<recursive_mutex> lock(mMutexFeatures);
		if (KeyFrame::isBad(pKF, mpMap))
		{
			cerr << "AddObservation with bad KeyFrame in MapPoint::AddObservation" << endl;
			return;
		}

		if (mObservations.count(pKF))
			return;
		else
		{
			mObservations[pKF] = idx;

			if (pKF->mvuRight[idx] >= 0)
				nObs += 2;
			else
				nObs++;
		}
	}

	void MapPoint::EraseObservation(KeyFrame *pKF)
	{
		bool bBad = false;
		{
			unique_lock<recursive_mutex> lock(mMutexFeatures);
			if (mObservations.count(pKF))
			{
				int idx = mObservations[pKF];

				if (KeyFrame::isBad(pKF, mpMap))
				{
					cerr << "Found bad KeyFrame Observer in MapPoint::EraseObservation" << endl;
					nObs--;
				}
				else
				{
					if (pKF->mvuRight[idx] >= 0)
						nObs -= 2;
					else
						nObs--;
				}

				mObservations.erase(pKF);

				if (mpRefKF == pKF)
					mpRefKF = mObservations.begin()->first;

				// If only 2 observations or less, discard point
				if (nObs <= 2)
					bBad = true;
			}
		}

		if (bBad)
			SetBadFlag();
	}

	map<KeyFrame*, size_t> MapPoint::GetObservations()
	{
		unique_lock<recursive_mutex> lock(mMutexFeatures);
		return mObservations;
	}

	int MapPoint::Observations()
	{
		unique_lock<recursive_mutex> lock(mMutexFeatures);
		return nObs;
	}

	void MapPoint::SetBadFlag()
	{
		map<KeyFrame*, size_t> obs;
		{
			unique_lock<recursive_mutex> lock1(mMutexFeatures);
			unique_lock<recursive_mutex> lock2(mMutexPos);
			mbBad = true;
			obs = mObservations;
			mObservations.clear();
		}

		int nBadObserver = 0;
		for (map<KeyFrame*, size_t>::iterator mit = obs.begin(); mit != obs.end(); ++mit)
		{
			KeyFrame *pKF = mit->first;
			if (KeyFrame::isBad(pKF, mpMap))
			{
				++nBadObserver;
				continue;
			}
			pKF->EraseMapPointMatch(mit->second);
		}
		if (nBadObserver)
			cerr << "Found " << nBadObserver << " bad KeyFrame in MapPoint::SetBadFlag" << endl;

		mpMap->EraseMapPoint(this);
	}

	MapPoint* MapPoint::GetReplaced()
	{
		unique_lock<recursive_mutex> lock1(mMutexFeatures);
		unique_lock<recursive_mutex> lock2(mMutexPos);
		return mpReplaced;
	}

	void MapPoint::Replace(MapPoint *pMP)
	{
		if (pMP->mnId == this->mnId)
			return;

		int nvisible, nfound;
		map<KeyFrame*, size_t> obs;

		{
			unique_lock<recursive_mutex> lock1(mMutexFeatures);
			unique_lock<recursive_mutex> lock2(mMutexPos);
			obs = mObservations;
			mObservations.clear();
			mbBad = true;
			nvisible = mnVisible;
			nfound = mnFound;
			mpReplaced = pMP;
		}

		int nBadObserver = 0;

		for (map<KeyFrame*, size_t>::iterator mit = obs.begin(); mit != obs.end(); ++mit)
		{
			// Replace measurement in keyframe
			KeyFrame *pKF = mit->first;

			if (KeyFrame::isBad(pKF, mpMap))
			{
				++nBadObserver;
				continue;
			}

			if (!pMP->IsInKeyFrame(pKF))
			{
				pKF->ReplaceMapPointMatch(mit->second, pMP);
				pMP->AddObservation(pKF, mit->second);
			}
			else
			{
				pKF->EraseMapPointMatch(mit->second);
			}
		}

		if (nBadObserver)
			cout << "Found " << nBadObserver << " bad KeyFrame obs in MapPoint::Replace" << endl;

		pMP->IncreaseFound(nfound);
		pMP->IncreaseVisible(nvisible);
		pMP->ComputeDistinctiveDescriptors();

		mpMap->EraseMapPoint(this);
	}

	bool MapPoint::isBad(MapPoint *pMP, Map *pMap)
	{
		return pMap->IsBadMapPoint(pMP);

		//unique_lock<recursive_mutex> lock(mMutexFeatures);
		//unique_lock<recursive_mutex> lock2(mMutexPos);
		//return mbBad;
	}

	void MapPoint::IncreaseVisible(int32_t n)
	{
		unique_lock<recursive_mutex> lock(mMutexFeatures);
		mnVisible += n;
	}

	void MapPoint::IncreaseFound(int32_t n)
	{
		unique_lock<recursive_mutex> lock(mMutexFeatures);
		mnFound += n;
	}

	float MapPoint::GetFoundRatio()
	{
		unique_lock<recursive_mutex> lock(mMutexFeatures);
		return static_cast<float>(mnFound) / mnVisible;
	}

	void MapPoint::ComputeDistinctiveDescriptors()
	{
		// 此函数干了
		// 需要有观察关键帧帧，递归观察关键帧，然后选出最合适的关键帧，然后将 最合适的关键帧 中的 关键点描述子 赋值给自己

		// Retrieve all observed descriptors
		vector<cv::Mat> vDescriptors;

		map<KeyFrame*, size_t> observations;

		{
			unique_lock<recursive_mutex> lock1(mMutexFeatures);
			if (mbBad)
				return;

			int nBadObserver = 0;

			for (auto i = mObservations.begin(); i != mObservations.end();)
				if (KeyFrame::isBad(i->first, mpMap))
				{
					i = mObservations.erase(i);
					++nBadObserver;
				}
				else
					++i;

			if (nBadObserver)
				cerr << "Found " << nBadObserver << " bad KF Observer in MapPoint::ComputeDistinctiveDescriptors" << endl;

			observations = mObservations;
		}

		if (observations.empty())
		{
			cerr << "Found not any KF Observer in MapPoint::ComputeDistinctiveDescriptors" << endl;
			SetBadFlag();
			return;
		}

		vDescriptors.reserve(observations.size());

		for (map<KeyFrame*, size_t>::iterator mit = observations.begin(); mit != observations.end(); ++mit)
		{
			KeyFrame *pKF = mit->first;
			vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
		}

		// Compute distances between them
		const size_t N = vDescriptors.size();

		//float Distances[N][N];
		float ** Distances = new float*[N];
		for (int i = 0; i < N; i++)
			Distances[i] = new float[N];

		for (size_t i = 0; i < N; i++)
		{
			Distances[i][i] = 0;
			for (size_t j = i + 1; j < N; j++)
			{
				int distij = ORBmatcher::DescriptorDistance(vDescriptors[i], vDescriptors[j]);
				Distances[i][j] = distij;
				Distances[j][i] = distij;
			}
		}

		// Take the descriptor with least median distance to the rest
		int BestMedian = INT_MAX;
		int BestIdx = 0;
		for (size_t i = 0; i < N; i++)
		{
			vector<int> vDists(Distances[i], Distances[i] + N);
			sort(vDists.begin(), vDists.end());
			int median = vDists[0.5*(N - 1)];

			if (median < BestMedian)
			{
				BestMedian = median;
				BestIdx = i;
			}
		}

		// 释放 Distances 数组
		for (int i = 0; i < N; i++)
			delete[] Distances[i];
		delete[] Distances;

		{
			unique_lock<recursive_mutex> lock(mMutexFeatures);
			mDescriptor = vDescriptors[BestIdx].clone();
		}
	}

	cv::Mat MapPoint::GetDescriptor()
	{
		unique_lock<recursive_mutex> lock(mMutexFeatures);
		return mDescriptor.clone();
	}

	int MapPoint::GetIndexInKeyFrame(KeyFrame *pKF)
	{
		unique_lock<recursive_mutex> lock(mMutexFeatures);
		if (mObservations.count(pKF))
			return mObservations[pKF];
		else
			return -1;
	}

	bool MapPoint::IsInKeyFrame(KeyFrame *pKF)
	{
		unique_lock<recursive_mutex> lock(mMutexFeatures);
		return mObservations.count(pKF);
	}

	void MapPoint::UpdateNormalAndDepth()
	{
		map<KeyFrame*, size_t> observations;
		KeyFrame *pRefKF;
		cv::Mat Pos;
		{
			unique_lock<recursive_mutex> lock1(mMutexFeatures);
			unique_lock<recursive_mutex> lock2(mMutexPos);
			if (mbBad)
				return;

			int nBadObserver = 0;
			
			// 增加代码，去除mObservations中无效关键帧
			for (auto i = mObservations.begin(); i != mObservations.end();)
			{
				if (KeyFrame::isBad(i->first, mpMap))
				{
					++nBadObserver;
					//throw;
					i = mObservations.erase(i);
				}
				else
					++i;
			}

			if (nBadObserver)
				cerr << "Found " << nBadObserver << " bad KF Observer in MapPoint::UpdateNormalAndDepth" << endl;


			// 增加代码
			if (mObservations.empty())
			{
				cerr << "Found not any KF Observer in MapPoint::UpdateNormalAndDepth" << endl;
				//throw;
				SetBadFlag();
				return;
			}

			// 增加代码，如果ref关键帧是无效的，则从mObservations中选一帧
			if (KeyFrame::isBad(mpRefKF, mpMap))
			{
				cerr << "Found MapPoint ref KeyFrame is bad. now will select new one from KF Observers in MapPoint::UpdateNormalAndDepth" << endl;
				mpRefKF = mObservations.begin()->first;
			}
			
			pRefKF = mpRefKF;
			Pos = mWorldPos.clone();
			observations = mObservations;
		}

		cv::Mat normal = cv::Mat::zeros(3, 1, CV_32F);
		int n = 0;
		for (map<KeyFrame*, size_t>::iterator mit = observations.begin(); mit != observations.end(); ++mit)
		{
			KeyFrame *pKF = mit->first;
			cv::Mat Owi = pKF->GetCameraCenter();
			cv::Mat normali = mWorldPos - Owi;
			normal = normal + normali / cv::norm(normali);
			n++;
		}

		cv::Mat PC = Pos - pRefKF->GetCameraCenter();
		const float dist = cv::norm(PC);
		const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
		const float levelScaleFactor = pRefKF->mvScaleFactors[level];
		const int nLevels = pRefKF->mnScaleLevels;

		{
			unique_lock<recursive_mutex> lock3(mMutexPos);
			mfMaxDistance = dist * levelScaleFactor;
			mfMinDistance = mfMaxDistance / pRefKF->mvScaleFactors[nLevels - 1];
			mNormalVector = normal / n;
		}
	}

	float MapPoint::GetMinDistanceInvariance()
	{
		unique_lock<recursive_mutex> lock(mMutexPos);
		return 0.8f * mfMinDistance;
	}

	float MapPoint::GetMaxDistanceInvariance()
	{
		unique_lock<recursive_mutex> lock(mMutexPos);
		return 1.2f * mfMaxDistance;
	}

	int MapPoint::PredictScale(const float currentDist, KeyFrame *pKF)
	{
		float ratio;
		{
			unique_lock<recursive_mutex> lock(mMutexPos);
			ratio = mfMaxDistance / currentDist;
		}

		// 理论上下面的判断可以屏蔽，因为调用此函数的函数都检查过了
		/*if (KeyFrame::isBad(pKF, mpMap))
		{
			cerr << "Found bad KeyFrame in MapPoint::PredictScale" << endl;
		}*/

		int nScale = ceil(log(ratio) / pKF->mfLogScaleFactor);
		if (nScale < 0)
			nScale = 0;
		else if (nScale >= pKF->mnScaleLevels)
			nScale = pKF->mnScaleLevels - 1;

		return nScale;
	}

	int MapPoint::PredictScale(const float currentDist, Frame *pF)
	{
		float ratio;
		{
			unique_lock<recursive_mutex> lock(mMutexPos);
			ratio = mfMaxDistance / currentDist;
		}

		int nScale = ceil(log(ratio) / pF->mfLogScaleFactor);
		if (nScale < 0)
			nScale = 0;
		else if (nScale >= pF->mnScaleLevels)
			nScale = pF->mnScaleLevels - 1;

		return nScale;
	}

	void MapPoint::SaveMapPoints(ofstream &f, const set<MapPoint*>& _mps, Map *pMap)
	{
		//排除掉所有无效地图点，同时创建 地图点id 到 地图点地址的映射
		set<MapPoint*> mps;
		for (MapPoint *mp : _mps)
		{
			if (MapPoint::isBad(mp, pMap))
				continue;
			if (mp->Observations() == 0)
			{
				// 排除没有与KeyFrame关联的地图点
				cerr << "Found MapPoint have not KF Observer in MapPoint::SaveMapPoints" << endl;
				continue;
			}
			
			// 如果与地图点关联的所有关键帧都是无效的，则跳过
			bool isContinue = true;
			for (auto i : mp->GetObservations())
				isContinue &= KeyFrame::isBad(i.first, pMap);
			if (isContinue)
			{
				cerr << "Found MapPoint all KF Observer is invalid in MapPoint::SaveMapPoints" << endl;
				continue;
			}

			if (KeyFrame::isBad(mp->GetReferenceKeyFrame(), pMap))
			{
				cerr << "Found MapPoint have bad ref KeyFrame in MapPoint::SaveMapPoints" << endl;
				//如果引用帧是无效的，则从obs中选一个作为新的
				mp->SetReferenceKeyFrame(mp->GetObservations().begin()->first);
			}
			mps.insert(mp);
		}
		uint64_t nMapPoints = mps.size();
		IoUtils::Write(f, nMapPoints);

		cout << "The number of MapPoints:" << nMapPoints << endl;

		//保存地图点
		for (MapPoint *mp : mps)
		{
			//保存当前MapPoint的ID和世界坐标值
			IoUtils::Write(f, mp->mnId);
			IoUtils::WriteMat(f, mp->GetWorldPos());
			IoUtils::Write(f, mp->mnFirstKFid);
			IoUtils::Write(f, mp->mnFirstFrame);
		}
	}

	void MapPoint::LoadMapPoints(ifstream &f, map<uint64_t, MapPoint*> &mMapPointById, GlobalData *pGlobalData)
	{
		// 读取地图点数量
		uint64_t nMapPoints = 0;
		IoUtils::Read(f, nMapPoints);
		cout << "The number of MapPoints:" << nMapPoints << endl;

		// 地图点id 到 地图点地址映射
		map<uint64_t, MapPoint*> mapMapPointById;

		// 加载地图点
		for (uint64_t i = 0; i < nMapPoints; ++i)
		{
			uint64_t id;
			cv::Mat pos;
			IoUtils::Read(f, id);
			IoUtils::ReadMat(f, pos);
			//初始化一个MapPoint，并设置其ID和Position
			MapPoint *mp = new MapPoint(pos, pGlobalData, id);

			IoUtils::Read(f, mp->mnFirstKFid);
			IoUtils::Read(f, mp->mnFirstFrame);

			mapMapPointById[mp->mnId] = mp;
		}
		mMapPointById = mapMapPointById;

	}

} //namespace ORB_SLAM
