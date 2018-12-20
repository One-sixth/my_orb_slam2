#ifndef PRESET_PARAMETERS_H
#define PRESET_PARAMETERS_H

namespace ORB_SLAM2
{
	// for class Frame
	constexpr auto FRAME_GRID_ROWS = 48;
	constexpr auto FRAME_GRID_COLS = 64;

	// for class ORBmatcher
	constexpr int TH_LOW = 50;
	constexpr int TH_HIGH = 100;
	constexpr int HISTO_LENGTH = 30;

	// for map
	// 每隔多少秒进行一次回收内存
	constexpr int RecyclingInterval = 3;

	// for PnPsolver
	// 最大迭代次数
	constexpr int PnpMaxIterationsCount = 300;
	// 重投影误差，重投影误差小于该值的点对的编号会加入inliers
	// 该值太小，会引起重跟踪变得困难，太大，误差也会变大
	constexpr float PnpReprojectionError = 5.f;
	constexpr float PnpConfidence = 0.99;
	// 至少有这个数量的点对的重投影误差小于 reprojectionError，否则返回空矩阵
	constexpr int PnpMinInliers = 50;
}

#endif