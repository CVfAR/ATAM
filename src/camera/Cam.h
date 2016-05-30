/*!
@file		Cam.h
@brief		header for CCam
*/

#pragma once

#include "opencv2/opencv.hpp"

#ifdef USEPGR
#include <FlyCapture2.h>
#endif

/*!
@class		CCam
@brief		camera interface
*/
class CCam
{
public:
	CCam();
	~CCam();

	bool SaveVideo(const std::string &name, const float fps = 30.f);
	bool OpenVideo(const std::string &name, int &w, int &h, int &c);
	bool Open(int &w, int &h, int &c);
	void Close(void);
	bool Get(cv::Mat &img);
	void SaveImage(void);
	bool LoadParameters(const std::string &name, const bool undist = true);
	bool SaveParameters(const std::string &name) const;

public:
	cv::Mat A;		//!< camera parameters
	cv::Mat D;		//!< distortion parameters

private:

#ifdef USEPGR
	FlyCapture2::Camera mPGRCap;	//!< camera class in flycaptures
#endif

	cv::VideoCapture mCap;	//!< camera class in opencv

	int mDeviceID;	//!< devide ID

	cv::Mat mImg;	//!< image

	cv::Mat mMapx, mMapy;	//!< map for undistortion
	bool mUndist;			//!< use undistorted image or not

	int mWidth;		//!< image width
	int mHeight;	//!< image height
	int mChannel;	//!< number of channels
	int mSize;		//!< image data size (width*height*channels)

	cv::VideoWriter mWriter;	//!< opencv video class
	std::string mName;			//!< file name
	bool mSave;		//!< save or not
	bool mVideo;	//!< video or not
};


