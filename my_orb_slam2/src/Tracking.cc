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

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include "Tracking.h"
#include "ORBmatcher.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "Initializer.h"
#include "Optimizer.h"
#include "PnPsolver.h"
#include "Frame.h"
#include "System.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LocalMapping.h"
#include "MapDrawer.h"
#include "ORBextractor.h"
#include "Viewer.h"
#include "KeyFrameDatabase.h"
#include "GlobalData.h"

namespace ORB_SLAM2
{

	Tracking::Tracking(System *pSys, ORBVocabulary *pVoc, GlobalData *pGlobalData, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase *pKFDB, const string &strSettingPath, const int sensor) :
		mState(NO_IMAGES_YET), mSensor(sensor), mpORBVocabulary(pVoc), mpGlobalData(pGlobalData), mpKeyFrameDB(pKFDB), mpSystem(pSys),
		mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap)
	{
		// Load camera parameters from settings file

		cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
		float fx = fSettings["Camera.fx"];
		float fy = fSettings["Camera.fy"];
		float cx = fSettings["Camera.cx"];
		float cy = fSettings["Camera.cy"];

		cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
		K.at<float>(0, 0) = fx;
		K.at<float>(1, 1) = fy;
		K.at<float>(0, 2) = cx;
		K.at<float>(1, 2) = cy;
		K.copyTo(mK);

		cv::Mat DistCoef(4, 1, CV_32F);
		DistCoef.at<float>(0) = fSettings["Camera.k1"];
		DistCoef.at<float>(1) = fSettings["Camera.k2"];
		DistCoef.at<float>(2) = fSettings["Camera.p1"];
		DistCoef.at<float>(3) = fSettings["Camera.p2"];
		const float k3 = fSettings["Camera.k3"];
		if (k3 != 0)
		{
			DistCoef.resize(5);
			DistCoef.at<float>(4) = k3;
		}
		DistCoef.copyTo(mDistCoef);

		mbf = fSettings["Camera.bf"];

		float fps = fSettings["Camera.fps"];
		if (fps == 0)
			fps = 30;

		// Max/Min Frames to insert keyframes and to check relocalisation
		mMinFrames = 0;
		mMaxFrames = fps;

		cout << endl << "Camera Parameters: " << endl;
		cout << "- fx: " << fx << endl;
		cout << "- fy: " << fy << endl;
		cout << "- cx: " << cx << endl;
		cout << "- cy: " << cy << endl;
		cout << "- k1: " << DistCoef.at<float>(0) << endl;
		cout << "- k2: " << DistCoef.at<float>(1) << endl;
		if (DistCoef.rows == 5)
			cout << "- k3: " << DistCoef.at<float>(4) << endl;
		cout << "- p1: " << DistCoef.at<float>(2) << endl;
		cout << "- p2: " << DistCoef.at<float>(3) << endl;
		cout << "- fps: " << fps << endl;


		int nRGB = fSettings["Camera.RGB"];
		mbRGB = nRGB;

		if (mbRGB)
			cout << "- color order: RGB (ignored if grayscale)" << endl;
		else
			cout << "- color order: BGR (ignored if grayscale)" << endl;

		// Load ORB parameters
		int nFeatures = fSettings["ORBextractor.nFeatures"];
		float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
		int nLevels = fSettings["ORBextractor.nLevels"];
		int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
		int fMinThFAST = fSettings["ORBextractor.minThFAST"];

		mpORBextractorLeft = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

		if (sensor == System::STEREO)
			mpORBextractorRight = new ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

		if (sensor == System::MONOCULAR)
			mpIniORBextractor = new ORBextractor(2 * nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);

		cout << endl << "ORB Extractor Parameters: " << endl;
		cout << "- Number of Features: " << nFeatures << endl;
		cout << "- Scale Levels: " << nLevels << endl;
		cout << "- Scale Factor: " << fScaleFactor << endl;
		cout << "- Initial Fast Threshold: " << fIniThFAST << endl;
		cout << "- Minimum Fast Threshold: " << fMinThFAST << endl;

		if (sensor == System::STEREO || sensor == System::RGBD)
		{
			mThDepth = mbf * (float)fSettings["ThDepth"] / fx;
			cout << endl << "Depth Threshold (Close/Far Points): " << mThDepth << endl;
		}

		if (sensor == System::RGBD)
		{
			mDepthMapFactor = fSettings["DepthMapFactor"];
			if (fabs(mDepthMapFactor) < 1e-5)
				mDepthMapFactor = 1;
			else
				mDepthMapFactor = 1.0f / mDepthMapFactor;
		}

	}

	void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
	{
		mpLocalMapper = pLocalMapper;
	}

	void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
	{
		mpLoopClosing = pLoopClosing;
	}

	void Tracking::SetViewer(Viewer *pViewer)
	{
		mpViewer = pViewer;
	}

	void Tracking::SettingAfterLoadMap(KeyFrame *pReferenceKF)
	{
		mnLastKeyFrameId = pReferenceKF->mnId;
		mpReferenceKF = pReferenceKF;
		mpLastKeyFrame = pReferenceKF;
		mLastFrame.mpReferenceKF = pReferenceKF;
		mState = LOST;
	}

	cv::Mat Tracking::GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double timestamp)
	{
		mImGray = imRectLeft;
		cv::Mat imGrayRight = imRectRight;

		if (mImGray.channels() == 3)
		{
			if (mbRGB)
			{
				cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
				cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGB2GRAY);
			}
			else
			{
				cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
				cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGR2GRAY);
			}
		}
		else if (mImGray.channels() == 4)
		{
			if (mbRGB)
			{
				cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
				cvtColor(imGrayRight, imGrayRight, cv::COLOR_RGBA2GRAY);
			}
			else
			{
				cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
				cvtColor(imGrayRight, imGrayRight, cv::COLOR_BGRA2GRAY);
			}
		}

		mCurrentFrame = Frame(mImGray, imGrayRight, timestamp, mpORBextractorLeft, mpORBextractorRight, mpORBVocabulary, mpGlobalData, mK, mDistCoef, mbf, mThDepth);

		Track();

		return mCurrentFrame.mTcw.clone();
	}


	cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double timestamp)
	{
		mImGray = imRGB;
		cv::Mat imDepth = imD;

		if (mImGray.channels() == 3)
		{
			if (mbRGB)
				cv::cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
			else
				cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
		}
		else if (mImGray.channels() == 4)
		{
			if (mbRGB)
				cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
			else
				cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
		}

		if ((fabs(mDepthMapFactor - 1.0f) > 1e-5) || imDepth.type() != CV_32F)
			imDepth.convertTo(imDepth, CV_32F, mDepthMapFactor);

		mCurrentFrame = Frame(mImGray, imDepth, timestamp, mpORBextractorLeft, mpORBVocabulary, mpGlobalData, mK, mDistCoef, mbf, mThDepth);

		Track();

		return mCurrentFrame.mTcw.clone();
	}


	cv::Mat Tracking::GrabImageMonocular(const cv::Mat &im, const double timestamp)
	{
		mImGray = im;

		if (mImGray.channels() == 3)
		{
			if (mbRGB)
				cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);
			else
				cvtColor(mImGray, mImGray, cv::COLOR_BGR2GRAY);
		}
		else if (mImGray.channels() == 4)
		{
			if (mbRGB)
				cvtColor(mImGray, mImGray, cv::COLOR_RGBA2GRAY);
			else
				cvtColor(mImGray, mImGray, cv::COLOR_BGRA2GRAY);
		}

		if (mState == NOT_INITIALIZED || mState == NO_IMAGES_YET)
			mCurrentFrame = Frame(mImGray, timestamp, mpIniORBextractor, mpORBVocabulary, mpGlobalData, mK, mDistCoef, mbf, mThDepth);
		else
			mCurrentFrame = Frame(mImGray, timestamp, mpORBextractorLeft, mpORBVocabulary, mpGlobalData, mK, mDistCoef, mbf, mThDepth);

		Track();

		return mCurrentFrame.mTcw.clone();
	}

	void Tracking::Track()
	{
		if (mState == NO_IMAGES_YET)
			mState = NOT_INITIALIZED;

		mLastProcessedState = mState;

		// Get Map Mutex -> Map cannot be changed
		unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

		if (mState == NOT_INITIALIZED)
		{
			if (mSensor == System::STEREO || mSensor == System::RGBD)
				StereoInitialization();
			else
				MonocularInitialization();

			mpFrameDrawer->Update(this);

			if (mState != OK)
				return;
		}
		else
		{
			// System is initialized. Track Frame.
			bool bOK;

			// 如果 mpReferenceKF 是bad，则在pMap中有效关键帧中选一个最新的
			if (KeyFrame::isBad(mpReferenceKF, mpMap))
			{
				auto kfs = mpMap->GetAllKeyFrames();
				auto it = max_element(kfs.begin(), kfs.end(), KeyFrame::CompareWithId);
				mpReferenceKF = *it;
				cerr << "mpReferenceKF is bad!" << endl;
			}

			// 如果 mLastFrame.mpReferenceKF 是bad，则在pMap中有效关键帧中选一个最新的
			if (KeyFrame::isBad(mLastFrame.mpReferenceKF, mpMap))
			{
				mLastFrame.mpReferenceKF = mpReferenceKF;
				cerr << "mLastFrame.mpReferenceKF is bad!" << endl;
			}

			// Initial camera pose estimation using motion model or relocalization (if tracking is lost)
			if (!mbOnlyTracking)
			{
				// Local Mapping is activated. This is the normal behaviour, unless
				// you explicitly activate the "only tracking" mode.

				if (mState == OK)
				{
					// Local Mapping might have changed some MapPoints tracked in last frame
					// 目前不进行replace，只是单纯将无效地图点替换为nullptr
					CheckReplacedInLastFrame();

					if (mVelocity.empty() || mCurrentFrame.mnId < mnLastRelocFrameId + 2)
					{
						bOK = TrackReferenceKeyFrame();
					}
					else
					{
						// 根据上一帧进行跟踪，消耗资源较小
						bOK = TrackWithMotionModel();
						if (!bOK)
							// 上面函数失败，则对最近关键帧进行跟踪
							bOK = TrackReferenceKeyFrame();
					}
				}
				else
				{
					bOK = Relocalization();
				}
			}
			else
			{
				// Localization Mode: Local Mapping is deactivated

				if (mState == LOST)
				{
					bOK = Relocalization();
				}
				else
				{
					if (!mbVO)
					{
						// In last frame we tracked enough MapPoints in the map

						if (!mVelocity.empty())
						{
							bOK = TrackWithMotionModel();
						}
						else
						{
							bOK = TrackReferenceKeyFrame();
						}
					}
					else
					{
						// In last frame we tracked mainly "visual odometry" points.

						// We compute two camera poses, one from motion model and one doing relocalization.
						// If relocalization is sucessfull we choose that solution, otherwise we retain
						// the "visual odometry" solution.

						bool bOKMM = false;
						bool bOKReloc = false;
						vector<MapPoint*> vpMPsMM;
						vector<bool> vbOutMM;
						cv::Mat TcwMM;
						if (!mVelocity.empty())
						{
							bOKMM = TrackWithMotionModel();
							vpMPsMM = mCurrentFrame.mvpMapPoints;
							vbOutMM = mCurrentFrame.mvbOutlier;
							TcwMM = mCurrentFrame.mTcw.clone();
						}
						bOKReloc = Relocalization();

						if (bOKMM && !bOKReloc)
						{
							mCurrentFrame.SetPose(TcwMM);
							mCurrentFrame.mvpMapPoints = vpMPsMM;
							mCurrentFrame.mvbOutlier = vbOutMM;

							if (mbVO)
							{
								for (int i = 0; i < mCurrentFrame.N; i++)
								{
									if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
									{
										mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
									}
								}
							}
						}
						else if (bOKReloc)
						{
							mbVO = false;
						}

						bOK = bOKReloc || bOKMM;
					}
				}
			}
			
			mCurrentFrame.mpReferenceKF = mpReferenceKF;

			// If we have an initial estimation of the camera pose and matching. Track the local map.
			if (!mbOnlyTracking)
			{
				if (bOK)
					bOK = TrackLocalMap();
			}
			else
			{
				// mbVO true means that there are few matches to MapPoints in the map. We cannot retrieve
				// a local map and therefore we do not perform TrackLocalMap(). Once the system relocalizes
				// the camera we will use the local map again.
				if (bOK && !mbVO)
					bOK = TrackLocalMap();
			}

			if (bOK)
				mState = OK;
			else
				mState = LOST;

			// Update drawer
			mpFrameDrawer->Update(this);

			// If tracking were good, check if we insert a keyframe
			if (bOK)
			{
				// Update motion model
				if (!mLastFrame.mTcw.empty())
				{
					cv::Mat LastTwc = cv::Mat::eye(4, 4, CV_32F);
					mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0, 3).colRange(0, 3));
					mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0, 3).col(3));
					mVelocity = mCurrentFrame.mTcw*LastTwc;
				}
				else
					mVelocity = cv::Mat();

				mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

				// Clean VO matches
				// 清除VO点，所以这里的点不能 SetBadFlag 而是要给后面进行 Delete
				for (int i = 0; i < mCurrentFrame.N; i++)
				{
					MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
					if (pMP)
						if (pMP->Observations() < 1)
						{
							mCurrentFrame.mvbOutlier[i] = false;
							mCurrentFrame.mvpMapPoints[i] = nullptr;
							//pMP->SetBadFlag();
						}
				}

				// Delete temporal MapPoints
				for (list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(); lit != mlpTemporalPoints.end(); ++lit)
				{
					// 这些临时地图点可以直接删除，因为他们并没有与现存的关键帧和地图点建立关系
					MapPoint *pMP = *lit;
					//pMP->SetBadFlag();
					delete pMP;
				}
				mlpTemporalPoints.clear();

				// Check if we need to insert a new keyframe
				if (NeedNewKeyFrame())
					CreateNewKeyFrame();

				// We allow points with high innovation (considererd outliers by the Huber Function)
				// pass to the new keyframe, so that bundle adjustment will finally decide
				// if they are outliers or not. We don't want next frame to estimate its position
				// with those points so we discard them in the frame.
				for (int i = 0; i < mCurrentFrame.N; i++)
				{
					if (mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
						mCurrentFrame.mvpMapPoints[i] = nullptr;
				}
			}

			// Reset if the camera get lost soon after initialization
			if (mState == LOST)
			{
				if (mpMap->KeyFramesInMap() <= 5)
				{
					cout << "Track lost soon after initialisation, reseting..." << endl;
					mpSystem->Reset();
					return;
				}
			}

			if (!mCurrentFrame.mpReferenceKF)
				mCurrentFrame.mpReferenceKF = mpReferenceKF;

			mLastFrame = Frame(mCurrentFrame);
		}

		// Store frame pose information to retrieve the complete camera trajectory afterwards.
		if (!mCurrentFrame.mTcw.empty())
		{
			cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
			mlRelativeFramePoses.push_back(Tcr);
			mlpReferences.push_back(mpReferenceKF);
			mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
			mlbLost.push_back(mState == LOST);
		}
		else
		{
			// This can happen if tracking is lost
			// 当加载地图时，这里的已有轨迹均为0，此时跳过下面的操作
			if (mlRelativeFramePoses.size())
				mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
			if (mlpReferences.size())
				mlpReferences.push_back(mlpReferences.back());
			if (mlFrameTimes.size())
				mlFrameTimes.push_back(mlFrameTimes.back());
			if (mlbLost.size())
				mlbLost.push_back(mState == LOST);
		}

		// calc fps
		++mnTrackedFrames;
		chrono::milliseconds nowTime = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now().time_since_epoch());
		if (nowTime - mLastClearTime > chrono::milliseconds(1000))
		{
			float interval = (nowTime - mLastClearTime).count() / 1000.;
			mTrackingFps = mnTrackedFrames / interval;
			mnTrackedFrames = 0;
			mLastClearTime = nowTime;
		}
	}


	void Tracking::StereoInitialization()
	{
		if (mCurrentFrame.N > 500)
		{
			// Set Frame pose to the origin
			mCurrentFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));

			// Create KeyFrame
			KeyFrame *pKFini = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

			// Insert KeyFrame in the map
			mpMap->AddKeyFrame(pKFini);

			// Create MapPoints and asscoiate to KeyFrame
			for (int i = 0; i < mCurrentFrame.N; i++)
			{
				float z = mCurrentFrame.mvDepth[i];
				if (z > 0)
				{
					cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
					MapPoint *pNewMP = new MapPoint(x3D, pKFini, mpGlobalData);
					pNewMP->AddObservation(pKFini, i);
					pKFini->AddMapPoint(pNewMP, i);
					pNewMP->ComputeDistinctiveDescriptors();
					pNewMP->UpdateNormalAndDepth();
					mpMap->AddMapPoint(pNewMP);

					mCurrentFrame.mvpMapPoints[i] = pNewMP;
				}
			}

			cout << "New map created with " << mpMap->MapPointsInMap() << " points" << endl;

			mpLocalMapper->InsertKeyFrame(pKFini);

			mLastFrame = Frame(mCurrentFrame);
			mnLastKeyFrameId = mCurrentFrame.mnId;
			mpLastKeyFrame = pKFini;

			mvpLocalKeyFrames.push_back(pKFini);
			mvpLocalMapPoints = mpMap->GetAllMapPoints();
			mpReferenceKF = pKFini;
			mCurrentFrame.mpReferenceKF = pKFini;

			mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

			mpMap->mvpKeyFrameOrigins.push_back(pKFini);

			mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);

			mState = OK;
		}
	}

	void Tracking::MonocularInitialization()
	{

		if (!mpInitializer)
		{
			// Set Reference Frame
			if (mCurrentFrame.mvKeys.size() > 100)
			{
				mInitialFrame = Frame(mCurrentFrame);
				mLastFrame = Frame(mCurrentFrame);
				mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
				for (size_t i = 0; i < mCurrentFrame.mvKeysUn.size(); i++)
					mvbPrevMatched[i] = mCurrentFrame.mvKeysUn[i].pt;

				if (mpInitializer)
					delete mpInitializer;

				mpInitializer = new Initializer(mCurrentFrame, 1.0, 200);

				fill(mvIniMatches.begin(), mvIniMatches.end(), -1);

				return;
			}
		}
		else
		{
			// Try to initialize
			if ((int)mCurrentFrame.mvKeys.size() <= 100)
			{
				delete mpInitializer;
				mpInitializer = nullptr;
				fill(mvIniMatches.begin(), mvIniMatches.end(), -1);
				return;
			}

			// Find correspondences
			ORBmatcher matcher(mpGlobalData, 0.9, true);
			int nmatches = matcher.SearchForInitialization(mInitialFrame, mCurrentFrame, mvbPrevMatched, mvIniMatches, 100);

			// Check if there are enough correspondences
			if (nmatches < 100)
			{
				delete mpInitializer;
				mpInitializer = nullptr;
				return;
			}

			cv::Mat Rcw; // Current Camera Rotation
			cv::Mat tcw; // Current Camera Translation
			vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

			if (mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
			{
				for (size_t i = 0, iend = mvIniMatches.size(); i < iend; i++)
				{
					if (mvIniMatches[i] >= 0 && !vbTriangulated[i])
					{
						mvIniMatches[i] = -1;
						nmatches--;
					}
				}

				// Set Frame Poses
				mInitialFrame.SetPose(cv::Mat::eye(4, 4, CV_32F));
				cv::Mat Tcw = cv::Mat::eye(4, 4, CV_32F);
				Rcw.copyTo(Tcw.rowRange(0, 3).colRange(0, 3));
				tcw.copyTo(Tcw.rowRange(0, 3).col(3));
				mCurrentFrame.SetPose(Tcw);

				CreateInitialMapMonocular();
			}
		}
	}

	void Tracking::CreateInitialMapMonocular()
	{
		// Create KeyFrames
		KeyFrame *pKFini = new KeyFrame(mInitialFrame, mpMap, mpKeyFrameDB);
		KeyFrame *pKFcur = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);


		pKFini->ComputeBoW();
		pKFcur->ComputeBoW();

		// Insert KFs in the map
		mpMap->AddKeyFrame(pKFini);
		mpMap->AddKeyFrame(pKFcur);

		// Create MapPoints and asscoiate to keyframes
		for (size_t i = 0; i < mvIniMatches.size(); i++)
		{
			if (mvIniMatches[i] < 0)
				continue;

			//Create MapPoint.
			cv::Mat worldPos(mvIniP3D[i]);

			MapPoint *pMP = new MapPoint(worldPos, pKFcur, mpGlobalData);

			pKFini->AddMapPoint(pMP, i);
			pKFcur->AddMapPoint(pMP, mvIniMatches[i]);

			pMP->AddObservation(pKFini, i);
			pMP->AddObservation(pKFcur, mvIniMatches[i]);

			pMP->ComputeDistinctiveDescriptors();
			pMP->UpdateNormalAndDepth();

			//Fill Current Frame structure
			mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
			mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

			//Add to Map
			mpMap->AddMapPoint(pMP);
		}

		// Update Connections
		pKFini->UpdateConnections();
		pKFcur->UpdateConnections();

		// Bundle Adjustment
		cout << "New Map created with " << mpMap->MapPointsInMap() << " points" << endl;

		Optimizer::GlobalBundleAdjustemnt(mpMap, 20);

		// Set median depth to 1
		float medianDepth = pKFini->ComputeSceneMedianDepth(2);
		float invMedianDepth = 1.0f / medianDepth;

		if (medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 100)
		{
			cout << "Wrong initialization, reseting..." << endl;
			Reset();
			return;
		}

		// Scale initial baseline
		cv::Mat Tc2w = pKFcur->GetPose();
		Tc2w.col(3).rowRange(0, 3) = Tc2w.col(3).rowRange(0, 3)*invMedianDepth;
		pKFcur->SetPose(Tc2w);

		// Scale points
		vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
		for (size_t iMP = 0; iMP < vpAllMapPoints.size(); iMP++)
		{
			if (vpAllMapPoints[iMP])
			{
				MapPoint *pMP = vpAllMapPoints[iMP];
				pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
			}
		}

		mpLocalMapper->InsertKeyFrame(pKFini);
		mpLocalMapper->InsertKeyFrame(pKFcur);

		mCurrentFrame.SetPose(pKFcur->GetPose());
		mnLastKeyFrameId = mCurrentFrame.mnId;
		mpLastKeyFrame = pKFcur;

		mvpLocalKeyFrames.push_back(pKFcur);
		mvpLocalKeyFrames.push_back(pKFini);
		mvpLocalMapPoints = mpMap->GetAllMapPoints();
		mpReferenceKF = pKFcur;
		mCurrentFrame.mpReferenceKF = pKFcur;

		mLastFrame = Frame(mCurrentFrame);

		mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

		mpMapDrawer->SetCurrentCameraPose(pKFcur->GetPose());

		mpMap->mvpKeyFrameOrigins.push_back(pKFini);

		mState = OK;
	}

	void Tracking::CheckReplacedInLastFrame()
	{
		// 暂时不处理replace，目前自身单纯的将无效地图点置为nullptr
		for (auto& mp : mLastFrame.mvpMapPoints)
			if (MapPoint::isBad(mp, mpMap))
				mp = nullptr;

		//for (int i = 0; i < mLastFrame.N; i++)
		//{
		//	MapPoint *pMP = mLastFrame.mvpMapPoints[i];
		//	// 因为一旦replaced，就会setbad，然后就会被删除，所以这里函数不会执行。。需要继续考虑和处理
		//	if (pMP)
		//	{
		//		MapPoint *pRep = pMP->GetReplaced();
		//		if (pRep)
		//		{
		//			mLastFrame.mvpMapPoints[i] = pRep;
		//		}
		//	}
		//}
	}

	bool Tracking::TrackReferenceKeyFrame()
	{
		// Compute Bag of Words vector
		mCurrentFrame.ComputeBoW();

		// We perform first an ORB matching with the reference keyframe
		// If enough matches are found we setup a PnP solver
		ORBmatcher matcher(mpGlobalData, 0.7, true);
		vector<MapPoint*> vpMapPointMatches;

		// 通过BoW向量查找当前帧与关键帧中相似的地图点
		int nmatches = matcher.SearchByBoW(mpReferenceKF, mCurrentFrame, vpMapPointMatches);

		if (nmatches < 15)
			return false;

		mCurrentFrame.mvpMapPoints = vpMapPointMatches;
		mCurrentFrame.SetPose(mLastFrame.mTcw);

		Optimizer::PoseOptimization(mpGlobalData, &mCurrentFrame);

		// Discard outliers
		int nmatchesMap = 0;
		for (int i = 0; i < mCurrentFrame.N; i++)
		{
			if (mCurrentFrame.mvpMapPoints[i])
			{
				if (mCurrentFrame.mvbOutlier[i])
				{
					MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

					mCurrentFrame.mvpMapPoints[i] = nullptr;
					mCurrentFrame.mvbOutlier[i] = false;
					pMP->mbTrackInView = false;
					pMP->mnLastFrameSeen = mCurrentFrame.mnId;
					nmatches--;
				}
				else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
					nmatchesMap++;
			}
		}

		return nmatchesMap >= 10;
	}

	void Tracking::UpdateLastFrame()
	{
		// Update pose according to reference keyframe
		KeyFrame *pRef = mLastFrame.mpReferenceKF;
		cv::Mat Tlr = mlRelativeFramePoses.back();

		mLastFrame.SetPose(Tlr*pRef->GetPose());

		if (mnLastKeyFrameId == mLastFrame.mnId || mSensor == System::MONOCULAR || !mbOnlyTracking)
			return;

		// Create "visual odometry" MapPoints
		// We sort points according to their measured depth by the stereo/RGB-D sensor
		vector<pair<float, int> > vDepthIdx;
		vDepthIdx.reserve(mLastFrame.N);
		for (int i = 0; i < mLastFrame.N; i++)
		{
			float z = mLastFrame.mvDepth[i];
			if (z > 0)
			{
				vDepthIdx.push_back(make_pair(z, i));
			}
		}

		if (vDepthIdx.empty())
			return;

		sort(vDepthIdx.begin(), vDepthIdx.end());

		// We insert all close points (depth<mThDepth)
		// If less than 100 close points, we insert the 100 closest ones.
		int nPoints = 0;
		for (size_t j = 0; j < vDepthIdx.size(); j++)
		{
			int i = vDepthIdx[j].second;

			bool bCreateNew = false;

			MapPoint *pMP = mLastFrame.mvpMapPoints[i];
			if (!pMP)
				bCreateNew = true;
			else if (pMP->Observations() < 1)
			{
				bCreateNew = true;
			}

			if (bCreateNew)
			{
				cv::Mat x3D = mLastFrame.UnprojectStereo(i);
				MapPoint *pNewMP = new MapPoint(x3D, mpGlobalData, &mLastFrame, i);

				mLastFrame.mvpMapPoints[i] = pNewMP;

				mlpTemporalPoints.push_back(pNewMP);
				nPoints++;
			}
			else
			{
				nPoints++;
			}

			if (vDepthIdx[j].first > mThDepth && nPoints > 100)
				break;
		}
	}

	bool Tracking::TrackWithMotionModel()
	{
		ORBmatcher matcher(mpGlobalData, 0.9, true);

		// Update last frame pose according to its reference keyframe
		// Create "visual odometry" points if in Localization Mode
		UpdateLastFrame();

		mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

		fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), nullptr);

		// Project points seen in previous frame
		int th;
		if (mSensor != System::STEREO)
			th = 15;
		else
			th = 7;
		int nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, th, mSensor == System::MONOCULAR);

		// If few matches, uses a wider window search
		if (nmatches < 20)
		{
			fill(mCurrentFrame.mvpMapPoints.begin(), mCurrentFrame.mvpMapPoints.end(), nullptr);
			nmatches = matcher.SearchByProjection(mCurrentFrame, mLastFrame, 2 * th, mSensor == System::MONOCULAR);
		}

		if (nmatches < 20)
			return false;

		// Optimize frame pose with all matches
		Optimizer::PoseOptimization(mpGlobalData, &mCurrentFrame);

		// Discard outliers
		int nmatchesMap = 0;
		for (int i = 0; i < mCurrentFrame.N; i++)
		{
			if (mCurrentFrame.mvpMapPoints[i])
			{
				if (mCurrentFrame.mvbOutlier[i])
				{
					MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

					mCurrentFrame.mvpMapPoints[i] = nullptr;
					mCurrentFrame.mvbOutlier[i] = false;
					pMP->mbTrackInView = false;
					pMP->mnLastFrameSeen = mCurrentFrame.mnId;
					nmatches--;
				}
				else if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
					nmatchesMap++;
			}
		}

		if (mbOnlyTracking)
		{
			mbVO = nmatchesMap < 10;
			return nmatches > 20;
		}

		return nmatchesMap >= 10;
	}

	bool Tracking::TrackLocalMap()
	{
		// We have an estimation of the camera pose and some map points tracked in the frame.
		// We retrieve the local map and try to find matches to points in the local map.

		UpdateLocalMap();

		SearchLocalPoints();

		// Optimize Pose
		Optimizer::PoseOptimization(mpGlobalData, &mCurrentFrame);
		mnMatchesInliers = 0;

		// Update MapPoints Statistics
		for (int i = 0; i < mCurrentFrame.N; i++)
		{
			if (mCurrentFrame.mvpMapPoints[i])
			{
				if (!mCurrentFrame.mvbOutlier[i])
				{
					mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
					if (!mbOnlyTracking)
					{
						if (mCurrentFrame.mvpMapPoints[i]->Observations() > 0)
							mnMatchesInliers++;
					}
					else
						mnMatchesInliers++;
				}
				else if (mSensor == System::STEREO)
					mCurrentFrame.mvpMapPoints[i] = nullptr;

			}
		}

		// Decide if the tracking was succesful
		// More restrictive if there was a relocalization recently
		if (mCurrentFrame.mnId < mnLastRelocFrameId + mMaxFrames && mnMatchesInliers < 50)
			return false;

		if (mnMatchesInliers < 30)
			return false;
		else
			return true;
	}


	bool Tracking::NeedNewKeyFrame()
	{
		if (mbOnlyTracking)
			return false;

		// If Local Mapping is freezed by a Loop Closure do not insert keyframes
		if (mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
			return false;

		const int nKFs = mpMap->KeyFramesInMap();

		// Do not insert keyframes if not enough frames have passed from last relocalisation
		if (mCurrentFrame.mnId<mnLastRelocFrameId + mMaxFrames && nKFs>mMaxFrames)
			return false;

		// Tracked MapPoints in the reference keyframe
		int nMinObs = 3;
		if (nKFs <= 2)
			nMinObs = 2;
		int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

		// Local Mapping accept keyframes?
		bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

		// Check how many "close" points are being tracked and how many could be potentially created.
		int nNonTrackedClose = 0;
		int nTrackedClose = 0;
		if (mSensor != System::MONOCULAR)
		{
			for (int i = 0; i < mCurrentFrame.N; i++)
			{
				if (mCurrentFrame.mvDepth[i] > 0 && mCurrentFrame.mvDepth[i] < mThDepth)
				{
					if (mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
						nTrackedClose++;
					else
						nNonTrackedClose++;
				}
			}
		}

		bool bNeedToInsertClose = (nTrackedClose < 100) && (nNonTrackedClose > 70);

		// Thresholds
		float thRefRatio = 0.75f;
		if (nKFs < 2)
			thRefRatio = 0.4f;

		if (mSensor == System::MONOCULAR)
			thRefRatio = 0.9f;

		// Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
		const bool c1a = mCurrentFrame.mnId >= mnLastKeyFrameId + mMaxFrames;
		// Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
		const bool c1b = (mCurrentFrame.mnId >= mnLastKeyFrameId + mMinFrames && bLocalMappingIdle);
		//Condition 1c: tracking is weak
		const bool c1c = mSensor != System::MONOCULAR && (mnMatchesInliers < nRefMatches*0.25 || bNeedToInsertClose);
		// Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
		const bool c2 = ((mnMatchesInliers < nRefMatches*thRefRatio || bNeedToInsertClose) && mnMatchesInliers > 15);

		if ((c1a || c1b || c1c) && c2)
		{
			// If the mapping accepts keyframes, insert keyframe.
			// Otherwise send a signal to interrupt BA
			if (bLocalMappingIdle)
			{
				return true;
			}
			else
			{
				mpLocalMapper->InterruptBA();
				if (mSensor != System::MONOCULAR)
				{
					if (mpLocalMapper->KeyframesInQueue() < 3)
						return true;
					else
						return false;
				}
				else
					return false;
			}
		}
		else
			return false;
	}

	void Tracking::CreateNewKeyFrame()
	{
		if (!mpLocalMapper->SetNotStop(true))
			return;

		KeyFrame *pKF = new KeyFrame(mCurrentFrame, mpMap, mpKeyFrameDB);

		mpReferenceKF = pKF;
		mCurrentFrame.mpReferenceKF = pKF;

		if (mSensor != System::MONOCULAR)
		{
			mCurrentFrame.UpdatePoseMatrices();

			// We sort points by the measured depth by the stereo/RGBD sensor.
			// We create all those MapPoints whose depth < mThDepth.
			// If there are less than 100 close points we create the 100 closest.
			vector<pair<float, int> > vDepthIdx;
			vDepthIdx.reserve(mCurrentFrame.N);
			for (int i = 0; i < mCurrentFrame.N; i++)
			{
				float z = mCurrentFrame.mvDepth[i];
				if (z > 0)
				{
					vDepthIdx.push_back(make_pair(z, i));
				}
			}

			if (!vDepthIdx.empty())
			{
				sort(vDepthIdx.begin(), vDepthIdx.end());

				int nPoints = 0;
				for (size_t j = 0; j < vDepthIdx.size(); j++)
				{
					int i = vDepthIdx[j].second;

					bool bCreateNew = false;

					MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
					if (!pMP)
						bCreateNew = true;
					else if (pMP->Observations() < 1)
					{
						bCreateNew = true;
						mCurrentFrame.mvpMapPoints[i] = nullptr;
					}

					if (bCreateNew)
					{
						cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
						MapPoint *pNewMP = new MapPoint(x3D, pKF, mpGlobalData);
						pNewMP->AddObservation(pKF, i);
						pKF->AddMapPoint(pNewMP, i);
						pNewMP->ComputeDistinctiveDescriptors();
						pNewMP->UpdateNormalAndDepth();
						mpMap->AddMapPoint(pNewMP);

						mCurrentFrame.mvpMapPoints[i] = pNewMP;
						nPoints++;
					}
					else
					{
						nPoints++;
					}

					if (vDepthIdx[j].first > mThDepth && nPoints > 100)
						break;
				}
			}
		}

		mpLocalMapper->InsertKeyFrame(pKF);

		mpLocalMapper->SetNotStop(false);

		mnLastKeyFrameId = mCurrentFrame.mnId;
		mpLastKeyFrame = pKF;
	}

	void Tracking::SearchLocalPoints()
	{
		// Do not search map points already matched
		for (vector<MapPoint*>::iterator vit = mCurrentFrame.mvpMapPoints.begin(); vit != mCurrentFrame.mvpMapPoints.end(); ++vit)
		{
			MapPoint *pMP = *vit;
			if (pMP)
			{
				if (MapPoint::isBad(pMP, mpMap))
				{
					*vit = nullptr;
				}
				else
				{
					pMP->IncreaseVisible();
					pMP->mnLastFrameSeen = mCurrentFrame.mnId;
					pMP->mbTrackInView = false;
				}
			}
		}

		int nToMatch = 0;

		// Project points in frame and check its visibility
		for (vector<MapPoint*>::iterator vit = mvpLocalMapPoints.begin(); vit != mvpLocalMapPoints.end(); ++vit)
		{
			MapPoint *pMP = *vit;

			if (MapPoint::isBad(pMP, mpMap))
				continue;

			if (pMP->mnLastFrameSeen == mCurrentFrame.mnId)
				continue;

			// Project (this fills MapPoint variables for matching)
			if (mCurrentFrame.isInFrustum(pMP, 0.5))
			{
				pMP->IncreaseVisible();
				nToMatch++;
			}
		}

		if (nToMatch > 0)
		{
			ORBmatcher matcher(mpGlobalData, 0.8);
			int th = 1;
			if (mSensor == System::RGBD)
				th = 3;
			// If the camera has been relocalised recently, perform a coarser search
			if (mCurrentFrame.mnId < mnLastRelocFrameId + 2)
				th = 5;
			matcher.SearchByProjection(mCurrentFrame, mvpLocalMapPoints, th);
		}
	}

	void Tracking::UpdateLocalMap()
	{
		// This is for visualization
		mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

		// Update
		UpdateLocalKeyFrames();
		UpdateLocalPoints();
	}

	void Tracking::UpdateLocalPoints()
	{
		mvpLocalMapPoints.clear();

		for (vector<KeyFrame*>::const_iterator itKF = mvpLocalKeyFrames.begin(); itKF != mvpLocalKeyFrames.end(); ++itKF)
		{
			KeyFrame *pKF = *itKF;
			const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();

			for (vector<MapPoint*>::const_iterator itMP = vpMPs.begin(); itMP != vpMPs.end(); ++itMP)
			{
				MapPoint *pMP = *itMP;
				if (!pMP)
					continue;
				if (pMP->mnTrackReferenceForFrame == mCurrentFrame.mnId)
					continue;
				if (!MapPoint::isBad(pMP, mpMap))
				{
					mvpLocalMapPoints.push_back(pMP);
					pMP->mnTrackReferenceForFrame = mCurrentFrame.mnId;
				}
			}
		}
	}


	void Tracking::UpdateLocalKeyFrames()
	{
		// Each map point vote for the keyframes in which it has been observed
		map<KeyFrame*, int> keyframeCounter;
		for (int i = 0; i < mCurrentFrame.N; i++)
		{
			if (mCurrentFrame.mvpMapPoints[i])
			{
				MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
				if (!MapPoint::isBad(pMP, mpMap))
				{
					const map<KeyFrame*, size_t> observations = pMP->GetObservations();
					for (map<KeyFrame*, size_t>::const_iterator it = observations.begin(); it != observations.end(); ++it)
						keyframeCounter[it->first]++;
				}
				else
				{
					mCurrentFrame.mvpMapPoints[i] = nullptr;
				}
			}
		}

		if (keyframeCounter.empty())
			return;

		int max = 0;
		KeyFrame *pKFmax = nullptr;

		mvpLocalKeyFrames.clear();
		mvpLocalKeyFrames.reserve(3 * keyframeCounter.size());

		// All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
		for (map<KeyFrame*, int>::const_iterator it = keyframeCounter.begin(); it != keyframeCounter.end(); ++it)
		{
			KeyFrame *pKF = it->first;

			if (KeyFrame::isBad(pKF, mpMap))
				continue;

			if (it->second > max)
			{
				max = it->second;
				pKFmax = pKF;
			}

			mvpLocalKeyFrames.push_back(it->first);
			pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
		}

		//for (auto itKF = mvpLocalKeyFrames.begin(); itKF != mvpLocalKeyFrames.end(); itKF++)
		//{
		//	if (KeyFrame::isBad(*itKF, mpMap))
		//		throw;
		//}

		// Include also some not-already-included keyframes that are neighbors to already-included keyframes
		//for (vector<KeyFrame*>::const_iterator itKF = mvpLocalKeyFrames.begin(), itEndKF = mvpLocalKeyFrames.end(); itKF != itEndKF; itKF++)
		for (auto itKF = mvpLocalKeyFrames.begin(); itKF != mvpLocalKeyFrames.end(); itKF++)
		{
			// Limit the number of keyframes
			if (mvpLocalKeyFrames.size() > 80)
				break;

			KeyFrame *pKF = *itKF;
			if (KeyFrame::isBad(pKF, mpMap))
				continue;

			const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

			for (vector<KeyFrame*>::const_iterator itNeighKF = vNeighs.begin(); itNeighKF != vNeighs.end(); ++itNeighKF)
			{
				KeyFrame *pNeighKF = *itNeighKF;
				if (!KeyFrame::isBad(pNeighKF, mpMap))
				{
					if (pNeighKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
					{
						mvpLocalKeyFrames.push_back(pNeighKF);
						pNeighKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
						break;
					}
				}
			}

			const set<KeyFrame*> spChilds = pKF->GetChilds();
			for (set<KeyFrame*>::const_iterator sit = spChilds.begin(); sit != spChilds.end(); ++sit)
			{
				KeyFrame *pChildKF = *sit;
				if (!KeyFrame::isBad(pChildKF, mpMap))
				{
					if (pChildKF->mnTrackReferenceForFrame != mCurrentFrame.mnId)
					{
						mvpLocalKeyFrames.push_back(pChildKF);
						pChildKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
						break;
					}
				}
			}

			KeyFrame *pParent = pKF->GetParent();
			if (pParent)
			{
				if (pParent->mnTrackReferenceForFrame != mCurrentFrame.mnId)
				{
					mvpLocalKeyFrames.push_back(pParent);
					pParent->mnTrackReferenceForFrame = mCurrentFrame.mnId;
					break;
				}
			}

		}

		if (pKFmax)
		{
			mpReferenceKF = pKFmax;
			mCurrentFrame.mpReferenceKF = mpReferenceKF;
		}
	}

	bool Tracking::Relocalization()
	{
		// Compute Bag of Words Vector
		mCurrentFrame.ComputeBoW();

		// Relocalization is performed when tracking is lost
		// Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
		// 找到所有与当前帧有关的关键帧然后当作候选帧
		vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

		if (vpCandidateKFs.empty())
			return false;

		const int nKFs = vpCandidateKFs.size();

		// We perform first an ORB matching with each candidate
		// If enough matches are found we setup a PnP solver
		ORBmatcher matcher(mpGlobalData, 0.75, true);

		// 储存着每一个候选帧的pnp求解器
		vector<PnPsolver*> vpPnPsolvers;
		vpPnPsolvers.resize(nKFs);
		// 每一个候选帧的与当前帧有关联的地图点集合
		vector<vector<MapPoint*> > vvpMapPointMatches;
		vvpMapPointMatches.resize(nKFs);
		// 长度为候选帧数量，对应位置设置为true代表该候选帧将不再考虑
		vector<bool> vbDiscarded;
		vbDiscarded.resize(nKFs);

		int nCandidates = 0;

		for (int i = 0; i < nKFs; i++)
		{
			KeyFrame *pKF = vpCandidateKFs[i];
			// 如果该候选帧已经无效，直接丢弃
			if (KeyFrame::isBad(pKF, mpMap))
				vbDiscarded[i] = true;
			else
			{
				// 对当前帧和当前候选帧进行比对，将同时存在当前帧和当前候选帧的地图点储存在 vvpMapPointMatches[i]
				int nmatches = matcher.SearchByBoW(pKF, mCurrentFrame, vvpMapPointMatches[i]);
				// 如果找到的地图点过少，直接丢弃此候选帧
				if (nmatches < 15)
				{
					vbDiscarded[i] = true;
					continue;
				}
				else
				{
					// 新建一个pnp求解器，用于求解在当前帧(2D点集合)与地图点(3D点集合)约束下的相机位姿
					PnPsolver* pSolver = new PnPsolver(mpMap, mCurrentFrame, vvpMapPointMatches[i]);

					// pnpsolver have been change
					/*pSolver->SetRansacParameters(0.99, 10, 300, 4, 0.5, 5.991);*/

					vpPnPsolvers[i] = pSolver;
					// 有效候选帧+1
					nCandidates++;
				}
			}
		}

		// Alternatively perform some iterations of P4P RANSAC
		// Until we found a camera pose supported by enough inliers
		bool bMatch = false;
		ORBmatcher matcher2(mpGlobalData, 0.9, true);

		// 该循环处理每个与当前帧有共享bow向量的关键帧
		// 继续循环条件为 有效候选帧数量不为0 和 尚未找到匹配
		// 下面两层循环原理是，设置solver最大迭代次数为300，但每次for循环只迭代5次
		// 这样就可以让所有候选帧都可以平均的获得求解机会，最先获得符合要求的求解结果的候选帧将选为最佳候选帧
		// 然后将最佳候选帧送入optimizer进行优化，如果返回值nGood大于10，该最佳候选帧将被选为关键帧
		while (nCandidates > 0 && !bMatch)
		{
			// 使用 nKFs 作为循环是因为这里是使用桶原理
			for (int i = 0; i < nKFs; i++)
			{
				if (vbDiscarded[i])
					continue;

				// Perform 5 Ransac Iterations

				// 哪些内点匹配成功了
				vector<bool> vbInliers;
				// 匹配成功的内点数量
				int nInliers;
				// 执行超过最大次数迭代仍然未满足条件时设置为true
				bool bNoMore;

				PnPsolver* pSolver = vpPnPsolvers[i];

				// 返回相机矩阵(4x4或空)，执行了5次Ransac迭代
				// pnpsolver have been change
				//cv::Mat Tcw = pSolver->iterate(5, bNoMore, vbInliers, nInliers);
				cv::Mat Tcw = pSolver->iterate(20, bNoMore, vbInliers, nInliers);


				// If Ransac reachs max. iterations discard keyframe
				// 如果达到了最大迭代次数，就丢弃该候选帧
				if (bNoMore)
				{
					vbDiscarded[i] = true;
					nCandidates--;
				}

				// If a Camera Pose is computed, optimize
				// 如果找到了一个相机矩阵，则对其进行优化
				if (!Tcw.empty())
				{
					Tcw.copyTo(mCurrentFrame.mTcw);

					set<MapPoint*> sFound;

					const int np = vbInliers.size();

					for (int j = 0; j < np; j++)
					{
						if (vbInliers[j])
						{
							mCurrentFrame.mvpMapPoints[j] = vvpMapPointMatches[i][j];
							sFound.insert(vvpMapPointMatches[i][j]);
						}
						else
							mCurrentFrame.mvpMapPoints[j] = nullptr;
					}

					int nGood = Optimizer::PoseOptimization(mpGlobalData, &mCurrentFrame);

					if (nGood < 10)
					{
						//// pnpsolver have been change
						//vbDiscarded[i] = true;
						//nCandidates--;

						continue;
					}

					for (int io = 0; io < mCurrentFrame.N; io++)
						if (mCurrentFrame.mvbOutlier[io])
							mCurrentFrame.mvpMapPoints[io] = nullptr;

					// If few inliers, search by projection in a coarse window and optimize again
					if (nGood < 50)
					{
						int nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 10, 100);

						if (nadditional + nGood >= 50)
						{
							nGood = Optimizer::PoseOptimization(mpGlobalData, &mCurrentFrame);

							// If many inliers but still not enough, search by projection again in a narrower window
							// the camera has been already optimized with many points
							if (nGood > 30 && nGood < 50)
							{
								sFound.clear();
								for (int ip = 0; ip < mCurrentFrame.N; ip++)
									if (mCurrentFrame.mvpMapPoints[ip])
										sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
								nadditional = matcher2.SearchByProjection(mCurrentFrame, vpCandidateKFs[i], sFound, 3, 64);

								// Final optimization
								if (nGood + nadditional >= 50)
								{
									nGood = Optimizer::PoseOptimization(mpGlobalData, &mCurrentFrame);

									for (int io = 0; io < mCurrentFrame.N; io++)
										if (mCurrentFrame.mvbOutlier[io])
											mCurrentFrame.mvpMapPoints[io] = nullptr;
								}
							}
						}
					}


					// If the pose is supported by enough inliers stop ransacs and continue
					if (nGood >= 50)
					{
						bMatch = true;
						break;
					}
				}
			}
		}

		if (!bMatch)
		{
			return false;
		}
		else
		{
			mnLastRelocFrameId = mCurrentFrame.mnId;
			return true;
		}

	}

	void Tracking::Reset()
	{

		cout << "System Reseting" << endl;
		if (mpViewer)
		{
			mpViewer->RequestStop();
			while (!mpViewer->isStopped())
				//usleep(3000);
				this_thread::sleep_for(std::chrono::microseconds(3000));
		}

		// Reset Local Mapping
		cout << "Reseting Local Mapper...";
		mpLocalMapper->RequestReset();
		cout << " done" << endl;

		// Reset Loop Closing
		cout << "Reseting Loop Closing...";
		mpLoopClosing->RequestReset();
		cout << " done" << endl;

		// Clear BoW Database
		cout << "Reseting Database...";
		mpKeyFrameDB->clear();
		cout << " done" << endl;

		// Clear Map (this erase MapPoints and KeyFrames)
		mpMap->Clear();

		//KeyFrame::nNextId = 0;
		mpGlobalData->nKeyFrameNextId = 0;
		//Frame::nNextId = 0;
		mpGlobalData->nFrameNextId = 0;
		mState = NO_IMAGES_YET;

		if (mpInitializer)
		{
			delete mpInitializer;
			mpInitializer = nullptr;
		}

		mlRelativeFramePoses.clear();
		mlpReferences.clear();
		mlFrameTimes.clear();
		mlbLost.clear();

		if (mpViewer)
			mpViewer->Release();
	}

	void Tracking::ChangeCalibration(const string &strSettingPath)
	{
		cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
		float fx = fSettings["Camera.fx"];
		float fy = fSettings["Camera.fy"];
		float cx = fSettings["Camera.cx"];
		float cy = fSettings["Camera.cy"];

		cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
		K.at<float>(0, 0) = fx;
		K.at<float>(1, 1) = fy;
		K.at<float>(0, 2) = cx;
		K.at<float>(1, 2) = cy;
		K.copyTo(mK);

		cv::Mat DistCoef(4, 1, CV_32F);
		DistCoef.at<float>(0) = fSettings["Camera.k1"];
		DistCoef.at<float>(1) = fSettings["Camera.k2"];
		DistCoef.at<float>(2) = fSettings["Camera.p1"];
		DistCoef.at<float>(3) = fSettings["Camera.p2"];
		const float k3 = fSettings["Camera.k3"];
		if (k3 != 0)
		{
			DistCoef.resize(5);
			DistCoef.at<float>(4) = k3;
		}
		DistCoef.copyTo(mDistCoef);

		mbf = fSettings["Camera.bf"];

		//Frame::mbInitialComputations = true;
		mpGlobalData->bFrameInitialComputations = true;
	}

	void Tracking::InformOnlyTracking(bool flag)
	{
		mbOnlyTracking = flag;
	}



} //namespace ORB_SLAM
