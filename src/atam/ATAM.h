/*!
@file		ATAM.h
@brief		header for CATAM
*/

#pragma once

#include <thread>
#include <string>

#include "opencv2/opencv.hpp"

#include "ATAMData.h"
#include "Cam.h"
#include "PointDetector.h"
#include "Calibration.h"
#include "cvsba.h"

/*!
@class		CATAM
@brief		ATAM
*/
class CATAM
{
public:
	CATAM();

public:
	void Start(void);

private:
	// setting
	bool init(void);

	// process
	void mainLoop(void);
	void process(void);
	void startInit(void);
	void startTAM(void);
	void changeState(void);
	void reset(void);

	// tracking
	bool setKeyframe(void);
	bool checkInsideImage(const cv::Point2f &pt) const;
	int trackFrame(void);
	bool matchKeyframe(void);
	bool computePose(void);

	// mapping
	void computePosefromE(const std::vector<cv::Point2f> &vUnPt1, const std::vector<cv::Point2f> &vUnPt2, cv::Mat &rvec, cv::Mat &tvec) const;
	void triangulate(const std::vector<cv::Point2f> &vUnPt1, const std::vector<cv::Point2f> &vUnPt2, const sPose &pose1, const sPose &pose2, std::vector<cv::Point3f> &vpt3d) const;
	bool initialBA(std::vector<cv::Point3f> &vPt3d,	const std::vector<cv::Point2f> &vDist1, const std::vector<cv::Point2f> &vDist2, sPose &pose1, sPose &pose2);
	bool makeMap(void);
	bool mappingCriteria(void) const;
	void mapping(void);
	void trackAndMap(void);
	void whileInitialize(void);
	void BA(void);

	// registration with world coordinate system
	void transformToWorld(const sPose &local, sPose &world) const;
	bool getWorldCoordinate(sPose &pose) const;
	void registerWorld(void);

	// relocalization
	void changeRelocalImage(void);
	void relocalize(void);
	void drawView(cv::Mat &img);

	// challenge points for competition
	void loadChallenge(const std::string &name);
	void drawChallenge(cv::Mat &img);

	// GUI
	void drawButton(cv::Mat &img);
	void checkButton(const int x, const int y);
	void generateButton(void);
	void mouse(int event, int x, int y, int flags);
	static void mousedummy(int event, int x, int y, int flags, void* param);
	bool operation(const int key);
	void draw(cv::Mat &img);
	void drawProcess(cv::Mat &img) const;
	void drawMap(cv::Mat &img) const;
	void drawGrid(cv::Mat &img) const;
	void drawTrack(cv::Mat &img) const;

private:
	CCam mCam;			//!< camera
	cv::Mat mImg;		//!< current image (from camera)
	cv::Mat mGImg;		//!< current image (gray scale)
	sPose mPose;		//!< current local camera pose
	sPose mWPose;		//!< current world camera pose
	int mFrameNumber;	//!< frame number
	int mChallengeNumber;		//!< challenge point number

	CCalibration mCalibrator;	//!< calibration
	CPointDetector mDetector;	//!< keypoint detector

	std::string mText;	//!< instruction on image
	double mFPS;		//!< frame per second in main process

	enum STATE{
		STOP, INIT, TAM, RELOCAL, CLOSE
	};
	STATE mState;		//!< state in ATAM

	sATAMData mData;	//!< all data in ATAM
	
	cvsba::Sba mBA;			//!< bundle adjustment (BA)
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
	std::vector<button> mvButton;	//!< for button input
	int mMouse;						//!< for mouse input
};
