/*!
@file		ATAMData.cpp
@brief		functions in ATAMData.h
*/

#include "ATAMData.h"

sATAMParams PARAMS;		//!< parameters in ATAM

/*!
@brief		constructor
*/
sPose::sPose(void)
{
	rvec = cv::Mat::eye(cvSize(1, 3), CV_64F);
	tvec = cv::Mat::eye(cvSize(1, 3), CV_64F);
}

/*!
@brief		copy constructor
@param[in]	r	right side
*/
sPose::sPose(const sPose& r)
{
	r.rvec.copyTo(rvec);
	r.tvec.copyTo(tvec);
}

/*!
@brief		= operator
@param[in]	r	right side
*/
sPose& sPose::operator=(const sPose& r)
{
	r.rvec.copyTo(rvec);
	r.tvec.copyTo(tvec);
	return *this;
}

/*!
@brief		set rvec and tvec in M
@param[out]	M	4 x 4 transformation matrix
*/
void sPose::getM(cv::Mat &M) const
{
	M = cv::Mat::eye(cvSize(4, 4), CV_64F);

	// 3x1 rotation vector to 3x3 rotation matrix
	cv::Mat r;
	cv::Rodrigues(rvec, r);

	r.copyTo(M(cv::Rect(0, 0, 3, 3)));
	tvec.copyTo(M(cv::Rect(3, 0, 1, 3)));
}

/*!
@brief		set  rotation matrix
@param[out]	R	3 x 3 transformation matrix
*/
void sPose::getR(cv::Mat &R) const
{
	cv::Rodrigues(rvec, R);
}


/*!
@brief	for debugging
*/
void sPose::print(void) const
{
	std::cout << rvec.t() << std::endl;
	std::cout << tvec.t() << std::endl;
}

/*!
@brief	constructor
*/
sTrack::sTrack(void)
{
	ptID = NOID;
}

/*!
@brief	constructor
*/
sKeyframe::sKeyframe(void)
{
	clear();
}

/*!
@brief	copy constructor
*/
sKeyframe::sKeyframe(const sKeyframe& r)
{
	img = r.img.clone();
	pose = r.pose;
	vKptID = r.vKptID;
	vKpt = r.vKpt;
	vDesc = r.vDesc.clone();
	vPtID = r.vPtID;
	vPt = r.vPt;
	ID = r.ID;
}

/*!
@brief	clear data
*/
void sKeyframe::clear(void)
{
	vKptID.clear();
	vKpt.clear();
	vDesc.release();

	vPtID.clear();
	vPt.clear();

	ID = NOID;
}

/*!
@brief	constructor
*/
CMapData::CMapData(void)
{
	Clear();
}

/*!
@brief	clear data
*/
void CMapData::Clear(void)
{
	mMapMutex.lock();

	mvPt.clear();
	mvKf.clear();
	mAdded = false;
	
	mMapMutex.unlock();
}

/*!
@brief			copy data to BA thread
@param[out]		data	BA data
*/
bool CMapData::CopytoBA(sBAData &data)
{
	if (mAdded){

		mMapMutex.lock();

		if (mvPt.size() == 0){
			return false;
		}

		// set 3d points
		int prevSize = int(data.vPt3d.size());
		data.vPt3d.resize(mvPt.size());
		for (int i = prevSize, iend = int(mvPt.size()); i < iend; ++i){
			data.vPt3d[i] = mvPt[i];
		}

		int size = int(mvKf.size()) > PARAMS.BAKEYFRAMES ? PARAMS.BAKEYFRAMES : int(mvKf.size());

		data.vKeyframe.clear();
		data.vKeyframeID.clear();

		data.vKeyframe.resize(size);
		data.vKeyframeID.resize(size);
		
		// set keyframe data
		for (int i = 0; i < size; ++i){
			int id = int(mvKf.size()) - size + i;
			data.vKeyframeID[i] = id;
			data.vKeyframe[i].pose = mvKf[id].pose;
			data.vKeyframe[i].vPt = mvKf[id].vPt;
			data.vKeyframe[i].vPtID = mvKf[id].vPtID;
		}

		mAdded = false;

		mMapMutex.unlock();

		return true;
	}
	else{
		return false;
	}
}

/*!
@brief		copy data from BA
@param[in]	data	BA data
*/
void CMapData::CopyfromBA(const sBAData &data)
{
	if (mvPt.size() != 0){

		mMapMutex.lock();
		for (int i = 0, iend = int(data.vVisibleID.size()); i < iend; ++i){
			int id = data.vVisibleID[i];
			mvPt[id] = data.vPt3d[id];
		}

		for (int i = 0, iend = int(data.vKeyframeID.size()); i < iend; ++i){
			int id = data.vKeyframeID[i];
			mvKf[id].pose = data.vKeyframe[i].pose;
		}
		mMapMutex.unlock();
	}
}

/*!
@brief	add keyframe
@param[in]	kf		keyframe
*/
void CMapData::AddKeyframe(const sKeyframe &kf)
{
	mMapMutex.lock();

	mvKf.push_back(kf);

	sKeyframe &lkf = mvKf.back();
	lkf.ID = int(mvKf.size()) - 1;		// set keyframe ID

	mAdded = true;

	LOGOUT("%d points in keyframes %d at %s\n", int(lkf.vKpt.size()), lkf.ID, __FUNCTION__);

	mMapMutex.unlock();
}

/*!
@brief		return last keyframe
@retval		sKeyframe
*/
sKeyframe& CMapData::GetLastKeyframe(void)
{
	return mvKf.back();
}

/*!
@brief		update last keyframe
@param[in]	vpt3d	mapped 3d points
@param[in]	vkpt	corresponding keypoints
@param[in]	vdesc	vkpt's descriptors
@param[out]	vid		vkpt's keypoint ID
*/
void CMapData::UpdateLastKeyframe(
	const std::vector<cv::Point3f> &vPt3d,
	const std::vector<cv::KeyPoint> &vKpt,
	const cv::Mat &vDesc,
	std::vector<int> &vID
	)
{
	mMapMutex.lock();

	// get last keyframe
	sKeyframe &lKf = mvKf.back();

	// get point ID
	int pointID = int(mvPt.size());

	// set point ID to points in the keyframe
	for (int i = 0, iend = int(vPt3d.size()); i < iend; ++i, ++pointID){
		mvPt.push_back(vPt3d[i]);

		lKf.vKpt.push_back(vKpt[i]);
		lKf.vKptID.push_back(pointID);

		lKf.vPt.push_back(vKpt[i].pt);
		lKf.vPtID.push_back(pointID);

		vID.push_back(pointID);
	}

	// set descriptor
	lKf.vDesc.push_back(vDesc);

	LOGOUT("Added %d points Total keypoints %d in keyframe %d\n", int(vPt3d.size()), int(lKf.vKpt.size()), lKf.ID);
	
	mMapMutex.unlock();
}

/*!
@brief		return point
@param[in]	id		point ID
@retval		point
*/
const cv::Point3f& CMapData::GetPoint(const int id) const
{
	return mvPt[id];
}

/*!
@brief		return nearest keyframe from all keyframes
@param[in]	pose	pose
@retval		keyframe
*/
const sKeyframe& CMapData::GetNearestKeyframe(const sPose &pose) const
{
	double dist = DBL_MAX;
	int ID = NOID;
	int ID2 = NOID;	// considering view direction

	for (int i = 0, iend = int(mvKf.size()); i < iend; ++i){

		// check distance
		double tmp = cv::norm(mvKf[i].pose.tvec - pose.tvec);

		// check direction of optical axis
		cv::Mat pR, kfR;
		mvKf[i].pose.getR(kfR);
		pose.getR(pR);

		cv::Mat pOptAx = pR.inv().col(2);
		cv::Mat kfOptAx = kfR.inv().col(2);

		double innerProduct = pOptAx.dot(kfOptAx);

		if (tmp < dist){
			dist = tmp;
			ID = i;

			if (innerProduct > 0.0) {
				ID2 = ID;
			}
		}
	}

	if (ID2 == NOID) {
		ID2 = ID;
	}

	return mvKf[ID2];
}

/*!
@brief			return randomly selected keyframe pose for relocalization
@param[out]		pose	pose
*/
void CMapData::GetRandomKeyFramePose(sPose &pose) const
{
	int size = PARAMS.RELOCALHIST > int(mvKf.size()) ? int(mvKf.size()) : PARAMS.RELOCALHIST;
	int val = rand() % size;
	int index = int(mvKf.size()) - 1 - val;

	pose = mvKf[index].pose;
}


/*!
@brief		constructor (with default parameters)
*/
sATAMParams::sATAMParams()
{
	MAXPTS = 400;	
	LEVEL = 2;
	DESCDIST = 50.f;

	BASEANGLE = 5.0;
	BAKEYFRAMES = 20;

	PROJERR = 2.0;
	MINPTS = 20;
	PATCHSIZE = 15;
	MATCHKEYFRAME = 0.3f;

	RELOCALHIST = 3;

	VIDEONAME = strData + "movie.avi";
	USEVIDEO = false;

	CAMERANAME = strData + "camera.xml";
}

/*!
@brief		load parameters
@param[in]	file name
*/
void sATAMParams::loadParams(const std::string &name)
{
	cv::FileStorage fs;

	if (!fs.open(name, cv::FileStorage::READ)){
		fs.open(name, cv::FileStorage::WRITE);

		// about keypoint
		cv::write(fs, "MAXPTS", MAXPTS);
		cv::write(fs, "LEVEL", LEVEL);
		cv::write(fs, "DESCDIST", DESCDIST);

		// about mapping
		cv::write(fs, "BASEANGLE", BASEANGLE);
		cv::write(fs, "BAKEYFRAMES", BAKEYFRAMES);

		// about tracking
		cv::write(fs, "PROJERR", PROJERR);
		cv::write(fs, "MINPTS", MINPTS);
		cv::write(fs, "PATCHSIZE", PATCHSIZE);
		cv::write(fs, "MATCHKEYFRAME", MATCHKEYFRAME);
		cv::write(fs, "RELOCALHIST", RELOCALHIST);

		cv::write(fs, "USEVIDEO", int(USEVIDEO));
		cv::write(fs, "VIDEONAME", VIDEONAME);
		
		cv::write(fs, "CAMERANAME", CAMERANAME);
	}
	else{
		cv::FileNode node(fs.fs, NULL);

		MAXPTS = node["MAXPTS"];
		LEVEL = node["LEVEL"];
		DESCDIST = node["DESCDIST"];

		BASEANGLE = node["BASEANGLE"];
		BAKEYFRAMES = node["BAKEYFRAMES"];

		PROJERR = node["PROJERR"];
		MINPTS = node["MINPTS"];
		PATCHSIZE = node["PATCHSIZE"];
		MATCHKEYFRAME = node["MATCHKEYFRAME"];
		RELOCALHIST = node["RELOCALHIST"];
		
		USEVIDEO = int(node["USEVIDEO"]) == 1 ? true : false;
		VIDEONAME = std::string(node["VIDEONAME"]);

		CAMERANAME = std::string(node["CAMERANAME"]);
	}

	BASETAN = tan(BASEANGLE / 180.0*CV_PI);
}

/*!
@brief	constructor
*/
sATAMData::sATAMData(void)
{
	clear();
}

/*!
@brief	clear data
*/
void sATAMData::clear(void)
{
	clearAllTrack();

	map.Clear();
	vKpt.clear();

	transMat = cv::Mat::eye(cvSize(4, 4), CV_64F);

	vPosePair.clear();

	haveScale = false;
}

/*!
@brief	clear tracking data
*/
void sATAMData::clearAllTrack(void)
{
	vTrack.clear();
	vPrevPt.clear();
}

/*!
@brief		clear track
@param[in]	ID		point ID
*/
void sATAMData::clearTrack(const int ID)
{
	// remove point if its ID is ID
	for (std::list<sTrack>::iterator it = vTrack.begin(); it != vTrack.end();){
		if (it->ptID == ID){
			it = vTrack.erase(it);
		}
		else{
			++it;
		}
	}

	// set points in previous image
	vPrevPt.clear();
	for (std::list<sTrack>::iterator it = vTrack.begin(),
		itend = vTrack.end(); it != itend; ++it){
		vPrevPt.push_back(it->vPt.back());
	}
}

/*!
@brief		set point for tracking (set kpt and ptID beforehand)
@param[in]	in	 track
*/
void sATAMData::addTrack(const sTrack &in)
{
	const cv::Point2f &pt = in.kpt.pt;
	vTrack.push_back(in);
	vTrack.back().vPt.push_back(pt);
	vPrevPt.push_back(pt);
}