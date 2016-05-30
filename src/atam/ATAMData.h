/*!
@file		ATAMData.h
@brief		All data for ATAM
*/

#pragma once

#include "DirectoryConfig.h"
#include "opencv2/opencv.hpp"

#include <vector>
#include <list>
#include <map>
#include <mutex>

#ifdef SHOWLOG
#define LOGOUT(fmt, ...) printf(fmt, ##__VA_ARGS__) 
#else
#define LOGOUT(fmt, ...)
#endif

const int NOID = -1;		//!< NO ID
const int DISCARD = -2;		//!< point that will be discarded


/*!
@struct		sPose
@brief		Pose parameters
*/
struct sPose
{
	sPose(void);
	sPose(const sPose& r);

	sPose& operator=(const sPose& r);

	void getM(cv::Mat &M) const;
	void getR(cv::Mat &R) const;
	void print(void) const;

	cv::Mat rvec;		//!< 3 x 1 rotation vector
	cv::Mat tvec;		//!< 3 x 1 translation vector
};

/*!
@struct		sTrack
@brief		point track
*/
struct sTrack
{
	sTrack(void);

	std::vector<cv::Point2f> vPt;	//!< list of points
	cv::KeyPoint kpt;				//!< keypoint at first frame
	int ptID;						//!< point ID
};

/*!
@struct		sKeyframe
@brief		keyframe data for mapping
*/
struct sKeyframe
{
	sKeyframe(void);
	sKeyframe(const sKeyframe& r);
	void clear(void);

	cv::Mat img;	//!< image
	sPose pose;		//!< pose

	// for relocalization
	std::vector<int> vKptID;			//!< point ID 
	std::vector<cv::KeyPoint> vKpt;		//!< keypoints
	cv::Mat vDesc;						//!< descriptors

	// for ba
	std::vector<int> vPtID;			//!< point ID
	std::vector<cv::Point2f> vPt;	//!< points

	int ID;		//!< keyframe ID
};

/*!
@struct		sBAData
@brief		data for BA
*/
struct sBAData
{
	std::vector<cv::Point3f> vPt3d;		//!< mapped points
	std::vector<int> vVisibleID;		//!< IDs of visible mapped points
	std::vector<sKeyframe> vKeyframe;	//!< keyframes
	std::vector<int> vKeyframeID;		//!< IDs of keyframes
};

/*!
@class		CMapData
@brief		map data
*/
class CMapData
{
public:
	CMapData(void);

	void Clear(void);
	bool CopytoBA(sBAData &data);
	void CopyfromBA(const sBAData &data);
	void AddKeyframe(const sKeyframe &kf);
	sKeyframe& GetLastKeyframe(void);
	void UpdateLastKeyframe(const std::vector<cv::Point3f> &vPt3d, const std::vector<cv::KeyPoint> &vKpt, const cv::Mat &vDesc, std::vector<int> &vID);
	const cv::Point3f& GetPoint(const int id) const;
	const sKeyframe& GetNearestKeyframe(const sPose &pose) const;
	void GetRandomKeyFramePose(sPose &pose) const;

private:
	std::vector<cv::Point3f> mvPt;	//!< 3D points in map
	std::vector<sKeyframe> mvKf;	//!< keyframes
	std::mutex mMapMutex;				//!< mutex
	bool mAdded;					//!< keyframe added and not used in BA
};

/*!
@struct		sATAMParams
@brief		parameters for ATAM
*/
struct sATAMParams
{
	sATAMParams();

	void loadParams(const std::string &name);

	int MAXPTS;					//!< maximum number of points
	int LEVEL;					//!< pyramid level for detector
	float DESCDIST;				//!< max distance for correct matches

	double BASEANGLE;			//!< angle (degree) for adding new keyframe
	double BASETAN;				//!< tangent of BASEANGLE
	int BAKEYFRAMES;			//!< number of keyframes for local bundle adjustment
	
	float PROJERR;				//!< tolerance for reprojection error
	int MINPTS;					//!< minimum number of points for tracking and mapping
	int PATCHSIZE;				//!< patch size for KLT	
	float MATCHKEYFRAME;		//!< inlier ratio for matching keyframe
	int RELOCALHIST;			//!< check last n frames for relocalization

	bool USEVIDEO;				//!< use video or not
	std::string VIDEONAME;		//!< video file name

	std::string CAMERANAME;		//!< camera parameter file name	
};

/*!
@struct		sATAMData
@brief		ATAM Data
*/
struct sATAMData
{
	sATAMData(void);

	void clear(void);
	void clearAllTrack(void);
	void clearTrack(const int ID);
	void addTrack(const sTrack &in);

	cv::Mat prevGImg;						//!< previous gray scale image
	std::list<sTrack> vTrack;				//!< all tracks	
	std::vector<cv::Point2f> vPrevPt;		//!< tracked points in previous image

	CMapData map;		//!< map data
	
	cv::Mat A, D;		//!< camera parameters
	double focal;		//!< focal length

	double scale;		//!< scale (world/local)
	cv::Mat transMat;	//!< transformation from local to world
	bool haveScale;		//!< scale is computed or not

	std::vector<std::pair<sPose, sPose>> vPosePair;	//!< set of world and local coordinates
	std::map<int, cv::Point3f> vChallenge;			//!< challenge points

	std::vector<cv::KeyPoint> vKpt;			//!< keypoints in current image
};
