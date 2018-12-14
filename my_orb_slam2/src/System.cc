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
#define _ENABLE_EXTENDED_ALIGNED_STORAGE

#include <pangolin/pangolin.h>
#include "System.h"
#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "Viewer.h"
#include "Converter.h"
#include "ORBVocabulary.h"
#include "GlobalData.h"

namespace ORB_SLAM2
{

	System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer) :
		mSensor(sensor)
	{
		// Output welcome message
		cout << endl <<
			"ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
			"This program comes with ABSOLUTELY NO WARRANTY;" << endl <<
			"This is free software, and you are welcome to redistribute it" << endl <<
			"under certain conditions. See LICENSE.txt." << endl << endl;

		cout << "Input sensor was set to: ";

		if (mSensor == MONOCULAR)
			cout << "Monocular" << endl;
		else if (mSensor == STEREO)
			cout << "Stereo" << endl;
		else if (mSensor == RGBD)
			cout << "RGB-D" << endl;

		//Check settings file
		cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
		if (!fsSettings.isOpened())
		{
			cerr << "Failed to open settings file at: " << strSettingsFile << endl;
			exit(-1);
		}
		fsSettings.release();

		mpGlobalData = new GlobalData;
		//mpGlobalData->LoadData(fsSettings);

		//Load ORB Vocabulary
		cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

		mpVocabulary = new ORBVocabulary();
		mpGlobalData->mpVocabulary = mpVocabulary;

		mpVocabulary->load(strVocFile);
		bool bVocLoad = true;

		if (!bVocLoad)
		{
			cerr << "Wrong path to vocabulary. " << endl;
			cerr << "Falied to open at: " << strVocFile << endl;
			exit(-1);
		}
		cout << "Vocabulary loaded!" << endl << endl;

		//Create KeyFrame Database
		mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);
		mpGlobalData->mpKeyFrameDatabase = mpKeyFrameDatabase;

		//Create the Map
		mpMap = new Map(mpGlobalData);
		mpGlobalData->mpMap = mpMap;

		//Create Drawers. These are used by the Viewer
		mpFrameDrawer = new FrameDrawer(mpMap);
		mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);
		mpGlobalData->mpFrameDrawer = mpFrameDrawer;
		mpGlobalData->mpMapDrawer = mpMapDrawer;

		//Initialize the Tracking thread
		//(it will live in the main thread of execution, the one that called this constructor)
		mpTracker = new Tracking(this, mpVocabulary, mpGlobalData, mpFrameDrawer, mpMapDrawer, mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);
		mpGlobalData->mpTracker = mpTracker;

		//Initialize the Local Mapping thread and launch
		mpLocalMapper = new LocalMapping(mpGlobalData, mpMap, mSensor == MONOCULAR);
		mpGlobalData->mpLocalMapper = mpLocalMapper;

		//Initialize the Loop Closing thread and launch
		mpLoopCloser = new LoopClosing(mpGlobalData, mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR);
		mpGlobalData->mpLoopCloser = mpLoopCloser;

		//Initialize the Viewer thread and launch
		if (bUseViewer)
		{
			mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile);
			mpTracker->SetViewer(mpViewer);
			mpGlobalData->mpViewer = mpViewer;
		}

		//Set pointers between threads
		mpTracker->SetLocalMapper(mpLocalMapper);
		mpTracker->SetLoopClosing(mpLoopCloser);

		mpLocalMapper->SetTracker(mpTracker);
		mpLocalMapper->SetLoopCloser(mpLoopCloser);

		mpLoopCloser->SetTracker(mpTracker);
		mpLoopCloser->SetLocalMapper(mpLocalMapper);
	}

	cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
	{
		if (mSensor != STEREO)
		{
			cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
			exit(-1);
		}

		// Check mode change
		{
			unique_lock<mutex> lock(mMutexMode);
			if (mbActivateLocalizationMode)
			{
				mpLocalMapper->RequestStop();

				// Wait until Local Mapping has effectively stopped
				while (!mpLocalMapper->isStopped())
				{
					//usleep(1000);
					this_thread::sleep_for(std::chrono::microseconds(1000));
				}

				mpTracker->InformOnlyTracking(true);
				mbActivateLocalizationMode = false;
			}
			if (mbDeactivateLocalizationMode)
			{
				mpTracker->InformOnlyTracking(false);
				mpLocalMapper->Release();
				mbDeactivateLocalizationMode = false;
			}
		}

		// Check reset
		{
			unique_lock<mutex> lock(mMutexReset);
			if (mbReset)
			{
				mpTracker->Reset();
				mbReset = false;
			}
		}

		cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight, timestamp);

		unique_lock<mutex> lock2(mMutexState);
		mTrackingState = mpTracker->mState;
		mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
		mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
		return Tcw;
	}

	cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
	{
		if (mSensor != RGBD)
		{
			cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
			exit(-1);
		}

		// Check mode change
		{
			unique_lock<mutex> lock(mMutexMode);
			if (mbActivateLocalizationMode)
			{
				mpLocalMapper->RequestStop();

				// Wait until Local Mapping has effectively stopped
				while (!mpLocalMapper->isStopped())
				{
					//usleep(1000);
					this_thread::sleep_for(std::chrono::microseconds(1000));
				}

				mpTracker->InformOnlyTracking(true);
				mbActivateLocalizationMode = false;
			}
			if (mbDeactivateLocalizationMode)
			{
				mpTracker->InformOnlyTracking(false);
				mpLocalMapper->Release();
				mbDeactivateLocalizationMode = false;
			}
		}

		// Check reset
		{
			unique_lock<mutex> lock(mMutexReset);
			if (mbReset)
			{
				mpTracker->Reset();
				mbReset = false;
			}
		}

		cv::Mat Tcw = mpTracker->GrabImageRGBD(im, depthmap, timestamp);

		unique_lock<mutex> lock2(mMutexState);
		mTrackingState = mpTracker->mState;
		mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
		mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
		return Tcw;
	}

	cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
	{
		if (mSensor != MONOCULAR)
		{
			cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
			exit(-1);
		}

		// Check mode change
		{
			unique_lock<mutex> lock(mMutexMode);
			if (mbActivateLocalizationMode)
			{
				mpLocalMapper->RequestStop();

				// Wait until Local Mapping has effectively stopped
				while (!mpLocalMapper->isStopped())
				{
					//usleep(1000);
					this_thread::sleep_for(std::chrono::microseconds(1000));
				}

				mpTracker->InformOnlyTracking(true);
				mbActivateLocalizationMode = false;
			}
			if (mbDeactivateLocalizationMode)
			{
				mpTracker->InformOnlyTracking(false);
				mpLocalMapper->Release();
				mbDeactivateLocalizationMode = false;
			}
		}

		// Check reset
		{
			unique_lock<mutex> lock(mMutexReset);
			if (mbReset)
			{
				mpTracker->Reset();
				mbReset = false;
			}
		}

		cv::Mat Tcw = mpTracker->GrabImageMonocular(im, timestamp);

		unique_lock<mutex> lock2(mMutexState);
		mTrackingState = mpTracker->mState;
		mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
		mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

		return Tcw;
	}

	void System::ActivateLocalizationMode()
	{
		unique_lock<mutex> lock(mMutexMode);
		mbActivateLocalizationMode = true;
	}

	void System::DeactivateLocalizationMode()
	{
		unique_lock<mutex> lock(mMutexMode);
		mbDeactivateLocalizationMode = true;
	}

	bool System::MapChanged()
	{
		int curn = mpMap->GetLastBigChangeIdx();
		if (mnLastBigChangeId < curn)
		{
			mnLastBigChangeId = curn;
			return true;
		}
		else
			return false;
	}

	void System::Reset()
	{
		unique_lock<mutex> lock(mMutexReset);
		mbReset = true;
	}

	void System::Shutdown()
	{
		// 先停止所有子线程
		mpLocalMapper->RequestFinish();
		mpLoopCloser->RequestFinish();
		if (mpViewer)
		{
			mpViewer->RequestFinish();
			pangolin::QuitAll();
			while (!mpViewer->isFinished())
				this_thread::sleep_for(std::chrono::milliseconds(10));
		}

		// Wait until all thread have effectively stopped
		while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || !mpLoopCloser->isFinishedGBA())
		{
			this_thread::sleep_for(std::chrono::milliseconds(10));
		}


		// 销毁指针
		if (mpViewer)
			delete mpViewer;

		delete mpLoopCloser;
		delete mpLocalMapper;
		delete mpTracker;
		delete mpMapDrawer;
		delete mpFrameDrawer;
		delete mpMap;
		delete mpKeyFrameDatabase;
		delete mpVocabulary;
		delete mpGlobalData;
	}

	void System::SaveTrajectoryTUM(const string &filename)
	{
		cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
		if (mSensor == MONOCULAR)
		{
			cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
			return;
		}

		vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
		sort(vpKFs.begin(), vpKFs.end(), KeyFrame::CompareWithId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		cv::Mat Two = vpKFs[0]->GetPoseInverse();

		ofstream f;
		f.open(filename.c_str());
		f << fixed;

		// Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
		// We need to get first the keyframe pose and then concatenate the relative transformation.
		// Frames not localized (tracking failure) are not saved.

		// For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
		// which is true when tracking failed (lbL).
		list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
		list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
		list<bool>::iterator lbL = mpTracker->mlbLost.begin();
		for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(),
			lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++, lbL++)
		{
			if (*lbL)
				continue;

			KeyFrame *pKF = *lRit;

			cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

			// If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
			while (KeyFrame::isBad(pKF, mpMap))
			{
				Trw = Trw * pKF->mTcp;
				pKF = pKF->GetParent();
			}

			Trw = Trw * pKF->GetPose()*Two;

			cv::Mat Tcw = (*lit)*Trw;
			cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
			cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

			vector<float> q = Converter::toQuaternion(Rwc);

			f << setprecision(6) << *lT << " " << setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
		}
		f.close();
		cout << endl << "trajectory saved!" << endl;
	}


	void System::SaveKeyFrameTrajectoryTUM(const string &filename)
	{
		cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

		vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
		sort(vpKFs.begin(), vpKFs.end(), KeyFrame::CompareWithId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		//cv::Mat Two = vpKFs[0]->GetPoseInverse();

		ofstream f;
		f.open(filename.c_str());
		f << fixed;

		for (size_t i = 0; i < vpKFs.size(); i++)
		{
			KeyFrame *pKF = vpKFs[i];

			// pKF->SetPose(pKF->GetPose()*Two);

			if (KeyFrame::isBad(pKF, mpMap))
				continue;

			cv::Mat R = pKF->GetRotation().t();
			vector<float> q = Converter::toQuaternion(R);
			cv::Mat t = pKF->GetCameraCenter();
			f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
				<< " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

		}

		f.close();
		cout << endl << "trajectory saved!" << endl;
	}

	void System::SaveTrajectoryKITTI(const string &filename)
	{
		cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
		if (mSensor == MONOCULAR)
		{
			cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
			return;
		}

		vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
		sort(vpKFs.begin(), vpKFs.end(), KeyFrame::CompareWithId);

		// Transform all keyframes so that the first keyframe is at the origin.
		// After a loop closure the first keyframe might not be at the origin.
		cv::Mat Two = vpKFs[0]->GetPoseInverse();

		ofstream f;
		f.open(filename.c_str());
		f << fixed;

		// Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
		// We need to get first the keyframe pose and then concatenate the relative transformation.
		// Frames not localized (tracking failure) are not saved.

		// For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
		// which is true when tracking failed (lbL).
		list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
		list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
		for (list<cv::Mat>::iterator lit = mpTracker->mlRelativeFramePoses.begin(), lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++)
		{
			ORB_SLAM2::KeyFrame *pKF = *lRit;

			cv::Mat Trw = cv::Mat::eye(4, 4, CV_32F);

			while (KeyFrame::isBad(pKF, mpMap))
			{
				//  cout << "bad parent" << endl;
				Trw = Trw * pKF->mTcp;
				pKF = pKF->GetParent();
			}

			Trw = Trw * pKF->GetPose()*Two;

			cv::Mat Tcw = (*lit)*Trw;
			cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
			cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

			f << setprecision(9) << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1) << " " << Rwc.at<float>(0, 2) << " " << twc.at<float>(0) << " " <<
				Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " " << Rwc.at<float>(1, 2) << " " << twc.at<float>(1) << " " <<
				Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " " << Rwc.at<float>(2, 2) << " " << twc.at<float>(2) << endl;
		}
		f.close();
		cout << endl << "trajectory saved!" << endl;
	}

	bool System::SaveMap(const string &filename)
	{
		cout << "Prepare to save the map" << endl;

		if (mpViewer)
		{
			mpViewer->RequestStop();
			while (!mpViewer->isStopped())
			{
				this_thread::sleep_for(chrono::milliseconds(5));
			}
		}
		mpLocalMapper->RequestStop();
		while (!mpLocalMapper->isStopped())
		{
			this_thread::sleep_for(chrono::milliseconds(5));
		}

		ofstream f(filename, ios::binary);
		if (!f.is_open())
		{
			cout << "Save map failure! file " + filename + " can't open!" << endl;
			return false;
		}
		mpMap->Save(f);
		f.close();

		if (mpViewer)
			mpViewer->Release();
		mpLocalMapper->Release();
		cout << "Save map success!" << endl;
		return true;
	}

	bool System::LoadMap(const string &filename)
	{
		cout << "Prepare to load the map" << endl;

		ifstream f(filename, ios::binary);
		if (!f.is_open())
		{
			cout << "Load map failure! file " + filename + " can't open!" << endl;
			return false;
		}

		{
			unique_lock<mutex> lock(mMutexReset);
			mpTracker->Reset();
		}

		if (mpViewer)
		{
			mpViewer->RequestStop();
			while (!mpViewer->isStopped())
			{
				this_thread::sleep_for(chrono::milliseconds(5));
			}
		}
		mpLocalMapper->RequestStop();
		while (!mpLocalMapper->isStopped())
		{
			this_thread::sleep_for(chrono::milliseconds(5));
		}

		mpMap->Load(f, mpTracker);

		f.close();

		if (mpViewer)
			mpViewer->Release();
		mpLocalMapper->Release();

		//ActivateLocalizationMode();

		cout << "Load map success!" << endl;
		return true;
	}

	int System::GetTrackingState()
	{
		unique_lock<mutex> lock(mMutexState);
		return mTrackingState;
	}

	vector<MapPoint*> System::GetTrackedMapPoints()
	{
		unique_lock<mutex> lock(mMutexState);
		return mTrackedMapPoints;
	}

	vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
	{
		unique_lock<mutex> lock(mMutexState);
		return mTrackedKeyPointsUn;
	}

} //namespace ORB_SLAM
