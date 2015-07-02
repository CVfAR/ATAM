/*!
@file		ATAM.h
@brief		header for CATAM
*/

#pragma once

#include <thread>
#include <string>

#include "opencv2/opencv.hpp"

#include "Cam.h"
#include "ATAMData.h"
#include "PointDetector.h"
#include "Calibration.h"

#ifdef WITHBA
#include "cvsba.h"
#endif

/*!
@class		CATAM
@brief		ATAM
*/
class CATAM
{
public:
	CATAM();
	~CATAM();

public:
	void Start(void);

private:
	// setting
	bool init(void);

	// interface
	void drawButton(cv::Mat &img);
	void checkButton(const int x, const int y);
	void generateButton(void);
	void mouse(int event, int x, int y, int flags);
	static void mousedummy(int event, int x, int y, int flags, void* param);
	bool operation(void);

	// process
	void mainLoop(void);
	void process(void);
	void draw(cv::Mat &img);
	void drawProcess(cv::Mat &img) const;
	void startInit(void);
	void startTAM(void);
	void changeState(void);
	void reset(void);

	// tracking
	bool setKeyframe(void);
	void projectMap(void);
	bool checkInsideImage(const cv::Point2f &pt) const;
	int trackFrame(void);
	void drawTrack(cv::Mat &img) const;
	bool matchKeyframe(void);
	bool computePose(void);

	// mapping
	void computePosefromE(const std::vector<cv::Point2f> &vpt1, const std::vector<cv::Point2f> &vpt2, cv::Mat &rvec, cv::Mat &tvec) const;
	void triangulate(const std::vector<cv::Point2f> &vpt1, const std::vector<cv::Point2f> &vpt2, const sPose &pose1, const sPose &pose2, std::vector<cv::Point3f> &vpt3d) const;
	bool initialBA(std::vector<cv::Point3f> &vpt3d,	const std::vector<cv::Point2f> &vundist1, const std::vector<cv::Point2f> &vundist2, sPose &pose1, sPose &pose2);
	bool makeMap(void);
	void drawMap(cv::Mat &img) const;
	void drawGrid(cv::Mat &img) const;
	bool mappingCriteria(void) const;
	void mapping(void);
	void trackAndMap(void);
	void initialize(void);

	// about challenge points for competition
	void loadChallenge(const std::string &name);
	void drawChallenge(cv::Mat &img);
	
	// registration with world coordinate system
	void transformToWorld(const sPose &local, sPose &world) const;
	bool getWorldCoordinate(sPose &pose) const;
	void registerWorld(void);

	// local BA
	void localBA(void);

	// relocalization
	void changeRelocalImage(void);
	void relocalize(void);
	void drawView(cv::Mat &img);

private:
	CCam mCam;			//!< camera
	cv::Mat mImg;		//!< current image (from camera)
	cv::Mat mGimg;		//!< current image (gray scale)
	sPose mPose;		//!< current local camera pose
	sPose mWPose;		//!< current world camera pose
	int mFrameNumber;	//!< frame number
	int mChallengeNumber;		//!< challenge point number

	CCalibration mCalibrator;	//!< calibration
	CPointDetector mDetector;	//!< keypoint detector

	std::string mText;	//!< instruction on image
	double mFPS;			//!< frame per second in main process

	enum STATE{
		STOP, INIT, TAM, RELOCAL, CLOSE
	};
	STATE mState;		//!< state in ATAM

	sATAMData mData;	//!< all data in ATAM
	
#ifdef WITHBA
	cvsba::Sba mBA;		//!< bundle adjustment (BA)
#endif
	bool mDoingBA;			//!< doing BA or not
	std::mutex mBAMutex;	//!< for BA

	/*!
	@struct		button
	@brief		button shape
	*/
	struct button{
		void set(int ix, int iy, int iw, int ih){ r.x = ix, r.y = iy, r.width = iw, r.height = ih; }
		bool in(int ix, int iy) { return r.x < ix && ix < r.x + r.width && r.y < iy && iy < r.y + r.height ? true : false; }
		cv::Rect r;
		int key;
		std::string name;
	};
	std::vector<button> mvButton;	//!< button information
	int mMouse;						//!< for mouse input
};
