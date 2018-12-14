/*
存放系统全局变量，常量，类静态变量
*/

#ifndef GLOBALDATA_H
#define GLOBALDATA_H

#include <cstdint>
#include <atomic>
#include <opencv2/core.hpp>

using namespace std;

namespace ORB_SLAM2
{

	class ORBVocabulary;
	class KeyFrameDatabase;
	class Map;
	class Tracking;
	class LocalMapping;
	class LoopClosing;
	class Viewer;
	class FrameDrawer;
	class MapDrawer;


	class GlobalData
	{
	public:
		GlobalData();
		~GlobalData();

	public:

		// other class pointer
		ORBVocabulary *mpVocabulary;
		KeyFrameDatabase *mpKeyFrameDatabase;
		Map *mpMap;
		Tracking *mpTracker;
		LocalMapping *mpLocalMapper;
		LoopClosing *mpLoopCloser;
		Viewer *mpViewer;
		FrameDrawer *mpFrameDrawer;
		MapDrawer *mpMapDrawer;



		// for Frame
		atomic<uint64_t> nFrameNextId = 0;
		bool bFrameInitialComputations = true;
		float nFrameMinX = 0, nFrameMinY = 0, nFrameMaxX = 0, nFrameMaxY = 0;
		float fFrameGridElementWidthInv = 0, fFrameGridElementHeightInv = 0;

		// for KeyFrame
		atomic<uint64_t> nKeyFrameNextId = 0;

		// for MapPoint
		atomic<uint64_t> nMapPointNextId = 0;
		mutex mutexMapPointGlobalPos;
	};

}

#endif