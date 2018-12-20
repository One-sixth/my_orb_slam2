#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <opencv2/opencv.hpp>

#include <System.h>

using namespace std;


int main(int argc, char **argv)
{
	cv::VideoCapture cam1(cv::CAP_DSHOW + 0);
	cv::VideoCapture cam2(cv::CAP_DSHOW + 1);

	if (!cam1.isOpened() || !cam2.isOpened())
	{
		cout << "打开摄像头失败" << endl;
		system("pause");
		return -1;
	}

	bool b = cam1.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	b &= cam1.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	b &= cam1.set(cv::CAP_PROP_FPS, 30);
	b &= cam2.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	b &= cam2.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	b &= cam2.set(cv::CAP_PROP_FPS, 30);
	if (b)
		cout << "设定640x480，成功" << endl;
	else
	{
		cout << "设定640x480，失败" << endl;
		system("pause");
		return -1;
	}
	
	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	const char *orb_voc = R"(..\DBow3-master\orbvoc.dbow3)";
	const char *cam_data = "stereo.yaml";

	ORB_SLAM2::System SLAM(orb_voc, cam_data, ORB_SLAM2::System::STEREO, true);

	SLAM.SetDontSaveTrack(true);


	// Read rectification parameters
	cv::FileStorage fsSettings(cam_data, cv::FileStorage::READ);
	if (!fsSettings.isOpened())
	{
		cerr << "ERROR: Wrong path to settings" << endl;
		return -1;
	}

	cv::Mat LM, LD, RM, RD, T, R;
	fsSettings["LM"] >> LM;
	fsSettings["LD"] >> LD;

	fsSettings["RM"] >> RM;
	fsSettings["RD"] >> RD;

	fsSettings["T"] >> T;
	fsSettings["R"] >> R;

	int rows_l = fsSettings["LEFT.height"];
	int cols_l = fsSettings["LEFT.width"];
	int rows_r = fsSettings["RIGHT.height"];
	int cols_r = fsSettings["RIGHT.width"];

	if (LM.empty() || LD.empty() || RM.empty() || RD.empty() || T.empty() || R.empty() ||
		rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
	{
		cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
		return -1;
	}

	cv::Size img_wh(cols_l, rows_l);

	cv::Mat R1, R2, P1, P2, Q;
	cv::Rect validPixROI1, validPixROI2;

	cv::stereoRectify(LM, LD, RM, RD, img_wh, R, T, R1, R2, P1, P2, Q, cv::CALIB_ZERO_DISPARITY, 0, img_wh, &validPixROI1, &validPixROI2);

	cv::Mat Lmap1, Lmap2, Rmap1, Rmap2;
	cv::initUndistortRectifyMap(LM, LD, R1, P1, img_wh, CV_32F, Lmap1, Lmap2);
	cv::initUndistortRectifyMap(RM, RD, R2, P2, img_wh, CV_32F, Rmap1, Rmap2);

	auto startTime = chrono::steady_clock::now();

	cv::Mat imL;
	cv::Mat imR;
	cv::Mat imCat;

	for (;;)
	{
		auto ret = cam1.grab();
		ret &= cam2.grab();
		ret &= cam1.retrieve(imL);
		ret &= cam2.retrieve(imR);
		if (ret)
		{
			double frameTime = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - startTime).count();

			cv::remap(imL, imL, Lmap1, Lmap2, cv::INTER_CUBIC);
			cv::remap(imR, imR, Rmap1, Rmap2, cv::INTER_CUBIC);

			cv::hconcat(imL, imR, imCat);

			cv::imshow("viewer", imCat);

			SLAM.TrackStereo(imL, imR, frameTime);
		}
		else
		{
			cout << "load img failure" << endl;
		}
		if (cv::waitKey(1) == 'q')
			break;
	}

	SLAM.Shutdown();

	return 0;
}