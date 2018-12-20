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

#ifndef MAP_H
#define MAP_H

#include <set>
#include <map>
#include <vector>
#include <mutex>
#include <fstream>
#include <cstdint>

using namespace std;

namespace ORB_SLAM2
{

	class MapPoint;
	class KeyFrame;
	class Tracking;
	class GlobalData;

	const int8_t FILE_MAGIC[] = {'O', 'M', 'A', 'P'};

	class Map
	{
	public:
		Map(GlobalData *gd);
		~Map();

		void AddKeyFrame(KeyFrame *pKF);
		void AddMapPoint(MapPoint *pMP);
		void EraseMapPoint(MapPoint *pMP);
		void EraseKeyFrame(KeyFrame *pKF);
		void SetReferenceMapPoints(const vector<MapPoint*> &vpMPs);
		void InformNewBigChange();
		int32_t GetLastBigChangeIdx();

		void AddExistKeyFrame(KeyFrame *pKF);
		void AddExistMapPoint(MapPoint *pMP);
		void EraseExistKeyFrame(KeyFrame *pKF);
		void EraseExistMapPoint(MapPoint *pMP);
		bool IsBadKeyFrame(KeyFrame *pKF);
		bool IsBadMapPoint(MapPoint *pMP);

		void CheckAndClean();

		vector<KeyFrame*> GetAllKeyFrames();
		vector<MapPoint*> GetAllMapPoints();
		vector<MapPoint*> GetReferenceMapPoints();

		uint64_t MapPointsInMap();
		uint64_t KeyFramesInMap();

		uint64_t GetMaxKFid();

		void Clear();

		// save/load map
		void Save(ofstream &f);
		void Load(ifstream &f, Tracking *pTracker);

		vector<KeyFrame*> mvpKeyFrameOrigins;

		// 这个是争夺Map修改权的互斥体，以前忘记考虑这个东西了
		// 用来控制线程对此类的修改权，要对Map进行较多的时间和较大的修改时需要锁定
		mutex mMutexMapUpdate;

		// This avoid that two points are created simultaneously in separate threads (id conflict)
		mutex mMutexPointCreation;

	protected:

		// 周期性释放内存
		void Recycling();

		thread mptRecycling;

		bool needRecyclingThreadFinish = false;

		set<MapPoint*> mspMapPoints;
		set<KeyFrame*> mspKeyFrames;

		//我加入的等待删除列表，隔一段时间就会删除无效点
		set<MapPoint*> mspWaitForDeleteMapPoints;
		set<KeyFrame*> mspWaitForDeleteKeyFrames;

		//我加入的存在列表，用来跟踪点是否存在内存中
		set<MapPoint*> mspExistMapPoints;
		set<KeyFrame*> mspExistKeyFrames;
		recursive_mutex mMutexExist;

		vector<MapPoint*> mvpReferenceMapPoints;

		//uint32_t mnMaxKFid = 0;

		// Index related to a big change in the map (loop closure, global BA)
		int32_t mnBigChangeIdx = 0;

		GlobalData *mpGlobalData = nullptr;

		mutex mMutexMap;
		//mutex mMutexMapPointsOp;
		//mutex mMutexKeyFramesOp;
		recursive_mutex mMutexRecycling;

	};

} //namespace ORB_SLAM

#endif // MAP_H
