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

	if (!cam1.isOpened())
	{
		cout << "打开摄像头失败" << endl;
		system("pause");
		return -1;
	}

	bool b = cam1.set(cv::CAP_PROP_FRAME_WIDTH, 640);
	b &= cam1.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	b &= cam1.set(cv::CAP_PROP_FPS, 30);
	cam1.set(cv::CAP_PROP_SETTINGS, 0);
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
	const char *cam_data = "mono.yaml";

	ORB_SLAM2::System SLAM(orb_voc, cam_data, ORB_SLAM2::System::MONOCULAR, true);

	SLAM.SetDontSaveTrack(true);

	auto startTime = chrono::steady_clock::now();

	cv::Mat im;
	for (;;)
	{
		auto ret = cam1.read(im);
		if (ret)
		{
			double frameTime = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - startTime).count();

			SLAM.TrackMonocular(im, frameTime);
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