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
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "GlobalData.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "LoopClosing.h"
#include "LocalMapping.h"

namespace ORB_SLAM2
{

	Map::Map(GlobalData *gd) :
		mpGlobalData(gd)
	{
		mptRecycling = thread(&Map::Recycling, this);
	}

	Map::~Map()
	{
		needRecyclingThreadFinish = true;
		if (mptRecycling.joinable())
			mptRecycling.join();
	}

	void Map::AddKeyFrame(KeyFrame *pKF)
	{
		//unique_lock<mutex> lock(mMutexMap);
		unique_lock<mutex> lock(mMutexMap);

		//// 检查重复id
		//if (!mspKeyFrames.count(pKF))
		//	for (auto i : mspKeyFrames)
		//		if (i->mnId == pKF->mnId)
		//		{
		//			cerr << "repeat id!" << endl;
		//			throw "repeat id!";
		//		}

		mspKeyFrames.insert(pKF);
		//if (pKF->mnId > mnMaxKFid)
		//	mnMaxKFid = pKF->mnId;
	}

	void Map::AddMapPoint(MapPoint *pMP)
	{
		//unique_lock<mutex> lock(mMutexMap);
		unique_lock<mutex> lock(mMutexMap);

		//// 检查重复id
		//if (!mspMapPoints.count(pMP))
		//	for (auto i : mspMapPoints)
		//		if (i->mnId == pMP->mnId)
		//		{
		//			cerr << "repeat id!" << endl;
		//			throw "repeat id!";
		//		}

		mspMapPoints.insert(pMP);
	}

	void Map::EraseMapPoint(MapPoint *pMP)
	{
		//unique_lock<mutex> lock(mMutexMap);
		{
			unique_lock<mutex> lock(mMutexMap);
			mspMapPoints.erase(pMP);
		}

		// TODO: This only erase the pointer.
		// Delete the MapPoint
		unique_lock<recursive_mutex> lock2(mMutexRecycling);
		mspWaitForDeleteMapPoints.insert(pMP);
	}

	void Map::EraseKeyFrame(KeyFrame *pKF)
	{
		//unique_lock<mutex> lock(mMutexMap);
		{
			unique_lock<mutex> lock(mMutexMap);
			mspKeyFrames.erase(pKF);
		}

		// TODO: This only erase the pointer.
		// Delete the MapPoint
		unique_lock<recursive_mutex> lock2(mMutexRecycling);
		mspWaitForDeleteKeyFrames.insert(pKF);
	}

	void Map::SetReferenceMapPoints(const vector<MapPoint*> &vpMPs)
	{
		unique_lock<mutex> lock(mMutexMap);
		mvpReferenceMapPoints = vpMPs;
	}

	void Map::InformNewBigChange()
	{
		// 当检测到回环时，将会调用此函数
		unique_lock<mutex> lock(mMutexMap);
		mnBigChangeIdx++;
	}

	int32_t Map::GetLastBigChangeIdx()
	{
		unique_lock<mutex> lock(mMutexMap);
		return mnBigChangeIdx;
	}

	void Map::AddExistKeyFrame(KeyFrame *pKF)
	{
		unique_lock<recursive_mutex> lock(mMutexExist);
		mspExistKeyFrames.insert(pKF);
	}

	void Map::AddExistMapPoint(MapPoint * pMP)
	{
		unique_lock<recursive_mutex> lock(mMutexExist);
		mspExistMapPoints.insert(pMP);
	}

	void Map::EraseExistKeyFrame(KeyFrame * pKF)
	{
		unique_lock<recursive_mutex> lock(mMutexExist);
		mspExistKeyFrames.erase(pKF);
	}

	void Map::EraseExistMapPoint(MapPoint * pMP)
	{
		unique_lock<recursive_mutex> lock(mMutexExist);
		mspExistMapPoints.erase(pMP);
	}

	bool Map::IsBadKeyFrame(KeyFrame *pKF)
	{
		if (pKF)
		{
			unique_lock<recursive_mutex> lock(mMutexExist);
			return mspExistKeyFrames.count(pKF) ? pKF->mbBad : true;
		}
		else
			return true;
	}

	bool Map::IsBadMapPoint(MapPoint *pMP)
	{
		if (pMP)
		{
			unique_lock<recursive_mutex> lock(mMutexExist);
			return mspExistMapPoints.count(pMP) ? pMP->mbBad : true;
		}
		else
			return true;
	}

	void Map::CheckAndClean()
	{
		// 第一步，检查和清理所有关键帧和地图点的无效引用
		auto nmps = mspMapPoints.size();
		auto nkfs = mspKeyFrames.size();

		set<MapPoint*> mps = mspMapPoints;
		set<KeyFrame*> kfs = mspKeyFrames;

		// 擦除所有观察关键帧少于3的地图点
		for (auto mp : mps)
		{
			for (auto it : mp->GetObservations())
			{
				if (KeyFrame::isBad(it.first, this))
					mp->EraseObservation(it.first);
			}
			if (mp->GetObservations().size() < 3)
				mp->SetBadFlag();
		}

		// 擦除观察到地图点少于50的关键帧
		for (auto kf : kfs)
		{
			kf->UpdateConnections();
			if (kf->GetMapPoints().size() < 50)
				kf->SetBadFlag();
		}

		mps = mspMapPoints;

		// 再擦除所有观察关键帧少于3的地图点
		for (auto mp : mps)
		{
			for (auto it : mp->GetObservations())
			{
				if (KeyFrame::isBad(it.first, this))
					mp->EraseObservation(it.first);
			}
			if (mp->GetObservations().size() < 3)
				mp->SetBadFlag();
		}

		cout << "Clean " << nmps - mspMapPoints.size() << " MapPoint in Map::CheckAndClean" << endl;
		cout << "Clean " << nkfs - mspKeyFrames.size() << " KeyFrame in Map::CheckAndClean" << endl;

	}

	vector<KeyFrame*> Map::GetAllKeyFrames()
	{
		//unique_lock<mutex> lock(mMutexMap);
		unique_lock<mutex> lock(mMutexMap);
		return vector<KeyFrame*>(mspKeyFrames.begin(), mspKeyFrames.end());
	}

	vector<MapPoint*> Map::GetAllMapPoints()
	{
		//unique_lock<mutex> lock(mMutexMap);
		unique_lock<mutex> lock(mMutexMap);
		return vector<MapPoint*>(mspMapPoints.begin(), mspMapPoints.end());
	}

	uint64_t Map::MapPointsInMap()
	{
		//unique_lock<mutex> lock(mMutexMap);
		return mspMapPoints.size();
	}

	uint64_t Map::KeyFramesInMap()
	{
		//unique_lock<mutex> lock(mMutexMap);
		return mspKeyFrames.size();
	}

	vector<MapPoint*> Map::GetReferenceMapPoints()
	{
		unique_lock<mutex> lock(mMutexMap);
		return mvpReferenceMapPoints;
	}

	uint32_t Map::GetMaxKFid()
	{
		//unique_lock<mutex> lock(mMutexMap);
		//return mnMaxKFid;
		return mpGlobalData->nKeyFrameNextId - 1;
	}

	void Map::Clear()
	{
		for (set<MapPoint*>::iterator sit = mspMapPoints.begin(); sit != mspMapPoints.end(); ++sit)
			delete *sit;

		for (set<KeyFrame*>::iterator sit = mspKeyFrames.begin(); sit != mspKeyFrames.end(); ++sit)
			delete *sit;

		mspMapPoints.clear();
		mspKeyFrames.clear();
		//mnMaxKFid = 0;
		mpGlobalData->nKeyFrameNextId = 0;
		mvpReferenceMapPoints.clear();
		mvpKeyFrameOrigins.clear();
	}

//#pragma optimize("", off)
	void Map::Save(ofstream &f)
	{
		//unique_lock<mutex> lock(mMutexMap);
		unique_lock<recursive_mutex> lock2(mMutexRecycling);
		unique_lock<mutex> lock3(mMutexMapUpdate);

		cout << "Map saving..." << endl;

		// 写入文件魔数
		IoUtils::Write(f, *(int32_t*)FILE_MAGIC);

		// 清理数据
		CheckAndClean();

		MapPoint::SaveMapPoints(f, mspMapPoints, this);

		KeyFrame::SaveKeyFrames(f, mspKeyFrames, this);

		cout << "Map saved!" << endl;
	}
//#pragma optimize("", on)

#pragma optimize("", off)
	void Map::Load(ifstream &f, Tracking *pTracker)
	{
		/*
		过程就是根据保存的顺序依次加载地图点的数目、地图点、关键帧的数目、关键帧、生长树和关联关系
		*/
		unique_lock<recursive_mutex> lock2(mMutexRecycling);
		unique_lock<mutex> lock3(mMutexMapUpdate);

		cout << "Loading map..." << endl;
		// 开始加载

		// 检查文件魔数
		int32_t magic = 0;
		IoUtils::Read(f, magic);
		if (magic != *(int32_t*)FILE_MAGIC)
		{
			cerr << "This is not a valid map file!" << endl;
			return;
		}

		// 加载地图点
		map<uint64_t, MapPoint*> mapMapPointById;
		MapPoint::LoadMapPoints(f, mapMapPointById, mpGlobalData);

		// 加载关键帧
		vector<KeyFrame*> kfs;
		KeyFrame::LoadKeyFrames(f, mpGlobalData, mapMapPointById, kfs);

		// 加载完成

		mspKeyFrames.insert(kfs.begin(), kfs.end());

		for (auto i = mapMapPointById.begin(); i != mapMapPointById.end(); ++i)
			mspMapPoints.insert(i->second);

		CheckAndClean();

		for (MapPoint *mp : mspMapPoints)
		{
			mp->ComputeDistinctiveDescriptors();
			mp->UpdateNormalAndDepth();
		}

		kfs.assign(mspKeyFrames.begin(), mspKeyFrames.end());

		// 按照关键帧id进行排序
		KeyFrame *maxIdKF = *max_element(kfs.begin(), kfs.end(), KeyFrame::CompareWithId);
		KeyFrame *minIdKF = *min_element(kfs.begin(), kfs.end(), KeyFrame::CompareWithId);

		uint64_t maxMpId = 0;
		for (auto i = mspMapPoints.begin(); i != mspMapPoints.end(); ++i)
			maxMpId = maxMpId < (*i)->mnId ? (*i)->mnId : maxMpId;

		// 设定id为最大值，避免id冲突
		mpGlobalData->nKeyFrameNextId = maxIdKF->mnId + 1;
		mpGlobalData->nFrameNextId.store(mpGlobalData->nKeyFrameNextId);
		mpGlobalData->nMapPointNextId = maxMpId + 1;

		auto refmps = minIdKF->GetMapPoints();
		mvpReferenceMapPoints = vector<MapPoint*>(refmps.begin(), refmps.end());
		mvpKeyFrameOrigins.push_back(minIdKF);

		// 设定Tracker相关变量
		pTracker->SettingAfterLoadMap(maxIdKF);

		// 将关键帧特征导入关键帧数据库，这样才可以重新导航
		for (auto kf : kfs)
			mpGlobalData->mpKeyFrameDatabase->add(kf);

		cout << "Map loaded!" << endl;
	}
#pragma optimize("", on)

	void Map::Recycling()
	{
		while (true)
		{
			this_thread::sleep_for(chrono::seconds(3));
			if (!mspWaitForDeleteKeyFrames.empty() || !mspWaitForDeleteMapPoints.empty())
			{
				//先让其他线程进入空闲阶段，再锁定自己的mMutexRecycling，不然可能死锁
				bool islock1 = false;
				if (!needRecyclingThreadFinish)
				{
					mpGlobalData->mpLocalMapper->mMutexRecycling.lock();
					mpGlobalData->mpLoopCloser->mMutexRecycling.lock();
					islock1 = true;
				}

				//unique_lock<recursive_mutex> lock4(mMutexExist);
				unique_lock<recursive_mutex> lock(mMutexRecycling);

				for (auto i : mspWaitForDeleteKeyFrames)
					delete i;

				mspWaitForDeleteKeyFrames.clear();

				for (auto i : mspWaitForDeleteMapPoints)
					delete i;

				mspWaitForDeleteMapPoints.clear();

				if (islock1)
				{
					mpGlobalData->mpLocalMapper->mMutexRecycling.unlock();
					mpGlobalData->mpLoopCloser->mMutexRecycling.unlock();
				}
			}
			if (needRecyclingThreadFinish)
				break;
		}
	}

} //namespace ORB_SLAM
