/*!
@file		PointDetector.h
@brief		header of CPointDetector
*/
#pragma once

#include "opencv2/opencv.hpp"

/*!
@class		CPointDetector
@brief		keypoint detector, descriptor and matcher
*/
class CPointDetector
{
public:
	CPointDetector();
	~CPointDetector();

	void Init(const int numpts, const int numlevel);
	void Match(const cv::Mat &query, const cv::Mat &train, std::vector<cv::DMatch> &vmatch) const;
	void Describe(const cv::Mat &img, std::vector<cv::KeyPoint> &vkpt, cv::Mat &vdesc) const;
	void Detect(const cv::Mat &img, std::vector<cv::KeyPoint> &vkpt) const;

private:
	cv::Ptr<cv::ORB> mDetector;					//!< detector
	cv::Ptr<cv::DescriptorMatcher> mMatcher;	//!< matcher
};


