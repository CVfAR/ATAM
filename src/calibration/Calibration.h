/*!
@file		Calibration.h
@brief		header for CCalibration
*/

#pragma once

#include "opencv2/opencv.hpp"

/*!
@class		CCalibration
@brief		calibration
*/
class CCalibration
{
public:
	CCalibration();
	~CCalibration();

	bool PoseEstimation(const cv::Mat &img, const cv::Mat &A, const cv::Mat &D, cv::Mat &rvec, cv::Mat &tvec) const;
	void Calibrate(const std::vector< std::vector< cv::Point2f > > &imagePoints, const cv::Size &imageSize, cv::Mat &intrinsic, cv::Mat &distortion) const;
	bool DetectCorners(const cv::Mat &img, std::vector<cv::Point2f> &vcorner) const;
	void DrawCorners(cv::Mat &img, const std::vector<cv::Point2f> &vcorner) const;
	void SetSize(const float data);
	float GetSize(void) const;
	int GetShortSide(void) const;

private:
	cv::Size mPattern;		//!< calibration pattern
	float mSize;			//!< square size or dot interval
	std::vector<cv::Point3f> mVpt3d;	//!< 3D coordinates of corners
};


