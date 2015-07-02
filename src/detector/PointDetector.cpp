/*!
@file		PointDetector.cpp
@brief		functions in CPointDetector
*/

#include "PointDetector.h"

/*!
@brief		constructor
*/
CPointDetector::CPointDetector()
{
	// initialize detector and matcher
	mDetector = cv::ORB::create();
	mMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming(2)");
}

/*!
@brief		destructor
*/
CPointDetector::~CPointDetector()
{
}

/*!
@brief		initialize parameters
@param[in]	numpts		maximum number of keypoints
@param[in]	numlevel	number of pyramid levels
*/
void CPointDetector::Init(
	const int numpts,
	const int numlevel
	)
{
	mDetector = cv::ORB::create(numpts, 1.2f, numlevel);
}

/*!
@brief		match descriptors
@param[in]	query		descriptors1
@param[in]	train		descriptors2
@param[out]	matches		matches
*/
void CPointDetector::Match(
	const cv::Mat &query,
	const cv::Mat &train,
	std::vector<cv::DMatch> &vmatch
	) const
{
	mMatcher->match(query, train, vmatch);
}

/*!
@brief		compute descriptors
@param[in]	img		gray scale image
@param[in]	vkpt	keypoints
@param[out]	desc	descriptors
*/
void CPointDetector::Describe(
	const cv::Mat &img,
	std::vector<cv::KeyPoint> &vkpt,
	cv::Mat &vdesc
	) const
{
	mDetector->compute(img, vkpt, vdesc);
}

/*!
@brief		detect keypoints
@param[in]	img		gray scale image
@param[out]	vkpt	keypoints
*/
void CPointDetector::Detect(
	const cv::Mat &img,
	std::vector<cv::KeyPoint> &vkpt
	) const
{
	mDetector->detect(img, vkpt);
}