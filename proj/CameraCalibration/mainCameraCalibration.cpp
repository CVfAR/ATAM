/*!
@file		mainCameraCalibration.cpp
@brief		camera calibration with OpenCV
*/

#include <stdio.h>
#include <vector>

#include "opencv2/opencv.hpp"
#include "Cam.h"
#include "Calibration.h"

#include "DirectoryConfig.h"



/*!
@brief		capture images
@param[out]	vimg	captured images
*/
void captureImages(std::vector<cv::Mat> &vimg)
{
	CCam cam;

	int width, height, channel;
	if (!cam.Open(width, height, channel)) {
		printf("camera error\n");
		exit(1);
	}

	cv::Mat im = cv::Mat(cv::Size(width, height), channel == 1 ? CV_8UC1 : CV_8UC3);

	const char window_name[128] = "camera calibration";
	cv::namedWindow(window_name);

	printf("space: save image\n");
	printf("esc: finish\n");

	CCalibration calib;

	while (1) {

		// capture images
		cam.Get(im);
		cv::Mat detect = im.clone();

		cv::imshow(window_name, detect);

		int key = cv::waitKey(10);
		if (key == ' ') {	// save image
							// extract corners
			std::vector< cv::Point2f > tmp;
			cv::Mat gDetect;
			cv::cvtColor(detect, gDetect, cv::COLOR_BGR2GRAY);

			bool found = calib.DetectCorners(gDetect, tmp);
			if (found) {
				calib.DrawCorners(detect, tmp);
				vimg.push_back(im.clone());
				printf("image:%02d saved\n", int(vimg.size()));
				cv::imshow(window_name, detect);
				cv::waitKey(1000);
			}
		}
		else if (key == 'q') {	// quit
			break;
		}
	}

	cv::destroyWindow(window_name);
}

/*!
@brief			calibrate camera
@param[in,out]	vimg		captured images
@param[out]		intrinsic	3 x 3 intrinsic parameters
@param[out]		distortion	1 x 4 distortion coefficients
@retval			true		calibrated
@retval			false		failed
*/
bool calibrate(std::vector<cv::Mat> &vimg, cv::Mat &intrinsic, cv::Mat &distortion)
{
	const char window_name[128] = "calibration";
	cv::namedWindow(window_name);
	cv::moveWindow(window_name, 0, 0);

	std::vector< std::vector< cv::Point2f > > imagePoints;

	CCalibration calib;

	// for each image
	for (int i = 0, iend = int(vimg.size()); i < iend; ++i) {

		// extract corners
		std::vector< cv::Point2f > tmp;
		cv::Mat gIm;
		cv::cvtColor(vimg[i], gIm, cv::COLOR_BGR2GRAY);
		bool found = calib.DetectCorners(gIm, tmp);
		if (found) {
			imagePoints.push_back(tmp);

			calib.DrawCorners(vimg[i], tmp);
			cv::imshow(window_name, vimg[i]);
			cv::waitKey(200);
		}
	}

	cv::destroyWindow(window_name);

	if (imagePoints.size() == 0) {
		printf("calibration failed\n");
		return false;
	}

	calib.Calibrate(imagePoints, vimg[0].size(), intrinsic, distortion);

	return true;
}

/*!
@brief		main function
*/
int main(int argc, char **argv)
{
	std::vector<cv::Mat> vimg;

	captureImages(vimg);

	cv::Mat intrinsic, distortion;
	if (calibrate(vimg, intrinsic, distortion)) {
		CCam camera;
		camera.A = intrinsic;
		camera.D = distortion;
		camera.SaveParameters(strData + "camera");
	}

	return 0;
}
