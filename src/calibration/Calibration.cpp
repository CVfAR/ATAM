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
@brief		destructor
*/
CCalibration::~CCalibration()
{
}

/*!
@brief		pose estimation with respect to calibration board
@param[in]	img			image
@param[in]	A			camera parameters
@param[in]	D			distortion parameters
@param[out]	rvec		rotation vector
@param[out]	tvec		translation vector
*/
bool CCalibration::PoseEstimation(
	const cv::Mat &img,
	const cv::Mat &A,
	const cv::Mat &D,
	cv::Mat &rvec,
	cv::Mat &tvec
	) const
{
	// detect corners
	std::vector<cv::Point2f> vpt2d;
	bool found = DetectCorners(img, vpt2d);

	if (found){

		// compute camera pose with respect to calibration board
		cv::solvePnP(mVpt3d, vpt2d, A, D, rvec, tvec);

		// check reprojection error
		std::vector<cv::Point2f> vprojpt2d;
		cv::projectPoints(mVpt3d, rvec, tvec, A, D, vprojpt2d);

		double projerr = 0.0;
		for (int i = 0, iend = int(vpt2d.size()); i < iend; ++i){
			projerr += cv::norm(vpt2d[i] - vprojpt2d[i]);
		}
		projerr /= double(vpt2d.size());

		if (projerr > 1.0){		// if error is large
			found = false;
		}
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
@param[in]	img			image
@param[out]	vcorner		corners
*/
bool CCalibration::DetectCorners(const cv::Mat &img, std::vector<cv::Point2f> &vcorner) const
{
	bool found = false;

#ifdef CHESSBOARD
	found = cv::findChessboardCorners(img, mPattern, vcorner);
	if (found){
		cv::Mat gimg;

		if (img.channels() == 3){
			cv::cvtColor(img, gimg, cv::COLOR_BGR2GRAY);
		}
		else{
			gimg = img.clone();
		}
		cv::cornerSubPix(gimg, vcorner, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
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
void CCalibration::DrawCorners(cv::Mat &img, const std::vector<cv::Point2f> &vcorner) const
{
	cv::drawChessboardCorners(img, mPattern, vcorner, true);
}

/*!
@brief		set pattern size
@param[in]	size	size
*/
void CCalibration::SetSize(const float size)
{
	mSize = size;
}

/*!
@brief		return pattern size
@retval		size
*/
float CCalibration::GetSize(void) const
{
	return mSize;
}

/*!
@brief		return short side length
@retval		size
*/
int CCalibration::GetShortSide(void) const
{
	return std::min(mPattern.width, mPattern.height);
}