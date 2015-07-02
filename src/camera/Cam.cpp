/*!
@file		Cam.cpp
@brief		functions in CCam
*/

#include "Cam.h"

/*!
@brief		constructor
*/
CCam::CCam()
{
	mDeviceID = 0;		// normally 0, but may be 1 or 2 for USB cam on laptop
	mSave = false;
	mVideo = false;
	mUndist = false;
}

/*!
@brief		destructor
*/
CCam::~CCam()
{
	Close();
}

/*!
@brief		save video
@param[in]	name		file name
@param[in]	fps			fps for recording
@retval		succeeded or not
*/
bool CCam::SaveVideo(
	const std::string &name,
	const float fps
	)
{
#ifdef USEPGR
	bool isopen = mPGRCap.IsConnected();
#else
	bool isopen = mCap.isOpened();
#endif

	if (isopen){	// if camera opened

		mSave = true;
		mName = name;

		bool isColor;

		if (mChannel == 1){
			isColor = false;
		}
		else{
			isColor = true;
		}

		return mWriter.open(mName, int(NULL), fps, cv::Size(mWidth, mHeight), isColor);
	}
	else{
		return isopen;
	}
}

/*!
@brief		open video
@param[in]	name		file name
@param[out]	width		image width
@param[out]	height		image height
@param[out]	channel		num of channels
@retval		successed or not
*/
bool CCam::OpenVideo(
	const std::string &name,
	int &w,
	int &h,
	int &c
	)
{
	mVideo = true;
	mName = name;

	bool isopen = mCap.open(mName);

	if (isopen){
		mCap >> mImg;

		mWidth = w = mImg.cols;
		mHeight = h = mImg.rows;
		mChannel = c = mImg.channels();

		mSize = mHeight*(int)mImg.step;
	}

	return isopen;
}

/*!
@brief		open camera
@param[out]	width		image width
@param[out]	height		image height
@param[out]	channel		num of channels
@retval		successed or not
*/
bool CCam::Open(
	int &w,
	int &h,
	int &c
	)
{
#ifdef USEPGR
	if (!mPGRCap.IsConnected()){

		FlyCapture2::Error error;

		// check cameras
		FlyCapture2::BusManager busMgr;
		unsigned int numCameras;
		error = busMgr.GetNumOfCameras(&numCameras);
		if (error != FlyCapture2::PGRERROR_OK || numCameras != 1){
			fprintf(stderr, "0 or multiple cameras found\n");
			return false;
		}

		// open one camera
		FlyCapture2::PGRGuid guid;
		error = busMgr.GetCameraFromIndex(0, &guid);
		if (error != FlyCapture2::PGRERROR_OK){
			return false;
		}

		// connect to a camera
		error = mPGRCap.Connect(&guid);
		if (error != FlyCapture2::PGRERROR_OK){
			return false;
		}

		// start capturing
		error = mPGRCap.StartCapture();
		if (error != FlyCapture2::PGRERROR_OK){
			return false;
		}

		// get image
		FlyCapture2::Image raw;
		error = mPGRCap.RetrieveBuffer(&raw);
		if (error != FlyCapture2::PGRERROR_OK){
			return false;
		}

		// VGA
		w = mWidth = 640;		// image width
		h= mHeight = 480;		// image height
		c= mChannel = 3;		// color camera

		mSize = mWidth*mHeight*mChannel;

		mImg = cv::Mat(cv::Size(mWidth, mHeight), CV_8UC3);

		return true;
	}
	else{
		return false;
	}
#else
	if (!mCap.isOpened()){

		if (!mCap.open(mDeviceID)){		// if opening failed
			fprintf(stderr, "Cam ID %d not found\n", mDeviceID);
			return false;
		}

		// VGA
		mCap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
		mCap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
	
		// check image
		int count = 0;
		while (mImg.data == NULL){

			mCap >> mImg;
			++count;

			if (count > 50){		// if retrieval failed
				fprintf(stderr, "Cannot retrieve images\n");
				return false;
			}
		}

		mWidth = w = mImg.cols;
		mHeight = h = mImg.rows;
		mChannel = c = mImg.channels();

		mSize = mHeight*(int)mImg.step;

		return true;
	}
	else{
		return false;
	}
#endif
}

/*!
@brief		close camera
*/
void CCam::Close(void)
{
#ifdef USEPGR
	if (mPGRCap.IsConnected()){

		FlyCapture2::Error error;

		error = mPGRCap.StopCapture();
		if (error != FlyCapture2::PGRERROR_OK){
			fprintf(stderr, "error at%s\n", __FUNCTION__);
		}

		error = mPGRCap.Disconnect();
		if (error != FlyCapture2::PGRERROR_OK){
			fprintf(stderr, "error at%s\n", __FUNCTION__);
		}
	}
#endif
	if (mCap.isOpened()){
		mCap.release();
	}
}

/*!
@brief		get image
@param[out]	img		image
@retval		successed or not
*/
bool CCam::Get(cv::Mat &img)
{
#ifdef USEPGR
	if (mVideo){
		mCap >> mImg;

		if (mImg.empty()){
			mCap.open(mName);
			mCap >> mImg;
		}
	}
	else if (mPGRCap.IsConnected()){

		FlyCapture2::Error error;

		FlyCapture2::Image raw;
		error = mPGRCap.RetrieveBuffer(&raw);

		FlyCapture2::Image img;
		error = raw.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &img);

		cv::Mat cvImg(cv::Size(raw.GetCols(), raw.GetRows()), CV_8UC3);
		int buffer_size = img.GetCols() * img.GetRows() * 3;
		memcpy_s(cvImg.data, buffer_size, img.GetData(), buffer_size);
		cv::resize(cvImg, mImg, mImg.size());

		memcpy_s(mImg.data, mSize, img.GetData(), mSize);

	}
	else{
		return false;
	}
#else
	if (mCap.isOpened()){
		mCap >> mImg;
		
		if (mVideo & mImg.empty()){		// for video
			mCap.open(mName);
			mCap >> mImg;
		}
	}
	else{
		fprintf(stderr, "Failed to capture\n");
		return false;
	}
#endif

	if (mUndist){
		cv::remap(mImg.clone(), mImg, mMapx, mMapy, cv::INTER_LINEAR);
	}

	img = mImg.clone();

	return true;
}

/*!
@brief		save image to video
*/
void CCam::SaveImage(void)
{
	if (mSave){
		mWriter << mImg;
	}
}

/*!
@brief		load camera parameters (call after Open)
@param[in]	name		file name
@param[in]	undist		undistort image or not
@retval		successed or not
*/
bool CCam::LoadParameters(
	const std::string &name,
	const bool undist
	)
{
	cv::FileStorage fs;
	if (!fs.open(name, cv::FileStorage::READ)){
		fprintf(stderr, "Cannot load camera parameters\n");
		return false;
	}

	cv::FileNode node(fs.fs, NULL);

	cv::read(node["A"], A);
	cv::read(node["D"], D);

	mUndist = undist;

	if (mUndist){
		cv::initUndistortRectifyMap(A, D, cv::Mat(), A, mImg.size(), CV_32FC1, mMapx, mMapy);
		D = cv::Mat_<double>::zeros(5, 1);
	}

	return true;
}

/*!
@brief		save camera parameters
@param[in]	name		file name
@retval		successed or not
*/
bool CCam::SaveParameters(const std::string &name) const
{
	if (A.empty() || D.empty()){
		fprintf(stderr, "Empty parameters\n");
		return false;
	}

	cv::FileStorage fs;
	fs.open(name + ".xml", cv::FileStorage::WRITE);

	cv::write(fs, "A", A);
	cv::write(fs, "D", D);

	fs.release();

	return true;
}
