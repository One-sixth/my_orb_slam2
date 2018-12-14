# my_orb_slam2

Original repository.<br>
https://github.com/raulmur/ORB_SLAM2<br><br>

# What is the difference
I made some simple modifications relative to the original repository.<br><br>
1.Clean up code, style changes.<br>
2.Migrate DBow2 to DBow3.<br>
3.Replace Dense and Eigen Solver to CSparse Solver.<br>
4.Replace PnpSolver to cv::solvePnPRansac.<br>
5.Migrate to opencv 4.0.<br>
6.Migrate g2o to the latest version.<br>
7.Added map reading and saving functions. I have tested it in monocular and stereo mode. referenced from https://www.cnblogs.com/mafuqiang/p/6972342.html.<br>
8.Fix a bug (maybe is not bug) in ORBmatcher.cc . referenced from https://www.zhihu.com/question/35116055.<br>
9.Fix a bug in Frame::Frame (Stereo). In the original implementation, the variable mb was used without initialization.<br>
10.Add keyframes and map points to erase from memory. It's very difficult to add this feature, and maybe there are still bugs.<br>

# How to build
//TODO
