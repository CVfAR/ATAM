/*!
@file		Calibration.cpp
@brief		functions in CCalibration
*/

#include "Calibration.h"

/*!
@brief		constructor
*/
CCalibration::CCalibration()
{
	// define size of calibration board and its pattern
#ifdef CHESSBOARD
	mPattern = cv::Size(9, 6);
	mSize = 100.f;
#else
	mPattern = cv::Size(4, 11);
	mSize = 34.f;
#endif

	// set 3D coordinates of corners
	for (int i = 0; i < mPattern.height; ++i){
		for (int j = 0; j < mPattern.width; j++){
#ifdef CHESSBOARD
			mVpt3d.push_back(cv::Point3f(float(i)*mSize, float(j)*mSize, 0));
#else
			float size = mSize / 2.0f;
			mVpt3d.push_back(cv::Point3f(float((2 * j + i % 2))*size, float(i)*size, 0));
#endif
		}
	}
}

/*!
@brief		pose estimation with respect to calibration board
@param[in]	img			image
@param[in]	A			camera parameters
@param[in]	D			distortion parameters
@param[out]	rvec		rotation vector
@param[out]	tvec		translation vector
*/
bool CCalibration::EstimatePose(
	const cv::Mat &img,
	const cv::Mat &A,
	const cv::Mat &D,
	cv::Mat &rvec,
	cv::Mat &tvec
	) const
{
	// detect corners
	std::vector<cv::Point2f> vPt2d;
	bool found = DetectCorners(img, vPt2d);

	if (found){
		// compute camera pose with respect to calibration board
		cv::solvePnP(mVpt3d, vPt2d, A, D, rvec, tvec);
	}

	return found;
}

/*!
@brief		calibrate camera parameters
@param[in]	imagePoints		a list of points
@param[in]	imageSize		image size
@param[out]	intrinsic		camera paramters
@param[out]	distortion		distortion parameters
*/
void CCalibration::Calibrate(
	const std::vector< std::vector< cv::Point2f > > &imagePoints,
	const cv::Size &imageSize,
	cv::Mat &intrinsic,
	cv::Mat &distortion
	) const
{
	// set 3D points of corners
	std::vector< std::vector< cv::Point3f > > objectPoints;
	objectPoints.resize(imagePoints.size(), mVpt3d);

	// calibrate camera
	std::vector< cv::Mat > rvecs, tvecs;
	cv::calibrateCamera(objectPoints, imagePoints, imageSize, intrinsic, distortion, rvecs, tvecs);
}

/*!
@brief		detect corners
@param[in]	gImg		gray scale image
@param[out]	vcorner		corners
*/
bool CCalibration::DetectCorners(const cv::Mat &gImg, std::vector<cv::Point2f> &vCorner) const
{
	bool found = false;

#ifdef CHESSBOARD
	found = cv::findChessboardCorners(gImg, mPattern, vCorner);
	if (found){
		cv::cornerSubPix(gImg, vCorner, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
	}
#else
	found = cv::findCirclesGrid(img, mPattern, vcorner, cv::CALIB_CB_ASYMMETRIC_GRID);
#endif

	return found;
}

/*!
@brief		draw corners
@param[out]	img			image
@param[in]	vcorner		corners
*/
void CCalibration::DrawCorners(cv::Mat &img, const std::vector<cv::Point2f> &vCorner) const
{
	cv::drawChessboardCorners(img, mPattern, vCorner, true);
}
