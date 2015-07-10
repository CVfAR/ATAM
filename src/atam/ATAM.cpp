/*!
@file		ATAM.cpp
@brief		functions in CATAM
*/

#include "ATAM.h"
#include "Timer.h"

#include <numeric> 
#include <fstream>

extern sATAMParams PARAMS;		//!< parameters in ATAM

/*!
@brief		constructor
*/
CATAM::CATAM()
{
	mDoingBA = false;
	reset();
}

/*!
@brief		destructor
*/
CATAM::~CATAM()
{
}

/*!
@brief		start ATAM
*/
void CATAM::Start(void)
{
	// initialization
	if (!init()){
		return;
	}

	// start
#ifdef MULTITHREAD
	std::thread BA(&CATAM::localBA, this);

	mainLoop();

	BA.join();
#else
	mainLoop();
#endif
}


/*!
@brief		initialization
@retval		OK or not
*/
bool CATAM::init(void)
{
	// load parameters
	PARAMS.loadParams(strData + "params.xml");

	// open camera
	int width, height, channel;
	if (PARAMS.USEVIDEO){
		if (!mCam.OpenVideo(PARAMS.VIDEONAME, width, height, channel)){
			LOGOUT("Cannot open %s\n", PARAMS.VIDEONAME.c_str());
			return false;
		}
	}
	else{
		if (!mCam.Open(width, height, channel)){
			LOGOUT("Cannot open camera\n");
			return false;
		}
	}

	// load camera parameters
	if (!mCam.LoadParameters(PARAMS.CAMERANAME)){
		LOGOUT("Cannot open %s\n", PARAMS.CAMERANAME.c_str());
		return false;
	}

	// set camera parameters to ATAM
	mCam.A.copyTo(mData.A);					// camera paramters	
	mCam.D.copyTo(mData.D);					// distortion parameters
	mData.focal = mCam.A.at<double>(0, 0);	// focal length

	// image
	mImg = cv::Mat(cv::Size(width, height), CV_8UC3);
	mGimg = cv::Mat(cv::Size(width, height), CV_8UC1);

	// load challenge points
	loadChallenge(strData + "challenge.txt");

	// initialize keypoint detector
	mDetector.Init(PARAMS.MAXPTS, PARAMS.LEVEL);

	// generate button
	generateButton();

	return true;
}



/*!
@brief			draw button
@param[out]		img		color image
*/
void CATAM::drawButton(cv::Mat &img)
{
	for (int i = 0, iend = int(mvButton.size()); i < iend; ++i){	// for each button
		cv::Scalar col(0, 255, 0);
		int thickness = 1;

		// show botton
		cv::rectangle(img, mvButton[i].r, col, thickness);

		// show text on button
		cv::putText(img, mvButton[i].name, cv::Point(mvButton[i].r.x, mvButton[i].r.y + mvButton[i].r.height / 2), cv::FONT_HERSHEY_SIMPLEX, 0.5, col, thickness);
	}
}

/*!
@brief		check button is pressed or not
@param[in]	x		x coordinate
@param[in]	y		y coordinate
*/
void CATAM::checkButton(const int x, const int y)
{
	for (int i = 0, iend = int(mvButton.size()); i < iend; ++i){		// for each button
		if (mvButton[i].in(x, y)){		// check inside button or not
			mMouse = mvButton[i].key;	
			break;
		}
	}
}

/*!
@brief		generate buttoin
*/
void CATAM::generateButton(void)
{
	// define button size
	const int ratio = 13;
	const int w = mImg.cols / ratio;
	const int h = mImg.rows / ratio;

	button tmp;

	// space
	tmp.set(mImg.cols - w, mImg.rows - h, w, h);
	tmp.key = ' ';
	tmp.name = "space";
	mvButton.push_back(tmp);

	// c
	tmp.set(mImg.cols - w, mImg.rows - 2 * h, w, h);
	tmp.key = 'c';
	tmp.name = "  c  ";
	mvButton.push_back(tmp);

	// n
	tmp.set(mImg.cols - w, mImg.rows - 3 * h, w, h);
	tmp.key = 'n';
	tmp.name = "  n  ";
	mvButton.push_back(tmp);

	// r
	tmp.set(mImg.cols - w, mImg.rows - 4 * h, w, h);
	tmp.key = 'r';
	tmp.name = "  r  ";
	mvButton.push_back(tmp);


	// esc
	tmp.set(mImg.cols - w, mImg.rows - 5 * h, w, h);	
	tmp.key = 0x1b;
	tmp.name = " ESC ";
	mvButton.push_back(tmp);
}


/*!
@brief		OpenCV mouse operation
*/
void CATAM::mouse(int event, int x, int y, int flags)
{
	switch (event){
	case cv::EVENT_LBUTTONDOWN:
		checkButton(x, y);
		break;
	default:
		break;
	}
}

/*!
@brief		mouse dummy for OpenCV mouse callback
*/
void CATAM::mousedummy(int event, int x, int y, int flags, void* param)
{
	CATAM *patam = reinterpret_cast<CATAM*>(param);
	patam->mouse(event, x, y, flags);
}

/*!
@brief		user interface loop
*/
void CATAM::mainLoop(void)
{
	// create window
	const char windowName[128] = "ATAM";
	cv::namedWindow(windowName);

	// set mouse callback
	cv::setMouseCallback(windowName, mousedummy, this);

	// timer
	CTimer timer;

	while (1){

		timer.Push(__FUNCTION__);	// start measuring time

		// get color image
		mCam.Get(mImg);

		// convert to gray scale
		cv::cvtColor(mImg, mGimg, cv::COLOR_BGR2GRAY);

		// process
		process();

		// get FPS
		mFPS = 1.0 / double(timer.Pop()) * 1000.0;

		// get keyboard input
		int mouse = mMouse;

		LOGOUT("--frame %d--\n", mFrameNumber);
		++mFrameNumber;

		// user's operation
		if (operation(cv::waitKey(1))){
			break;
		}

		// show image
		draw(mImg);

		// show image
		cv::imshow(windowName, mImg);
	}

	// close
	cv::destroyWindow(windowName);
	mState = STATE::CLOSE;
}


/*!
@brief		user's operation
@param[in]	input key
@retval		ESC is pressed or not
*/
bool CATAM::operation(const int key)
{
	if (key == ' ' || mMouse == ' '){		// change state
		changeState();
	}
	else if (key == 'r' || mMouse == 'r'){	// reset
		reset();
	}
	else if (key == 'n' || mMouse == 'n'){ 
		if (mState == STATE::RELOCAL){		// change image for relocalization
			changeRelocalImage();
		}
		else if (mState == STATE::TAM){		// add keyframe
			mapping();
		}
	}
	else if (key == 'c' || mMouse == 'c'){
		++mChallengeNumber;
	}
	else if (key == 0x1b || mMouse == 0x1b){	// exit
		return true;
	}

	mMouse = -1;	// clear mouse data
	
	return false;
}

/*!
@brief			main process
*/
void CATAM::process(void)
{
	// detect keypoint
	mData.vKpt.clear();
	mDetector.Detect(mGimg, mData.vKpt);

	// each process
	switch (mState){
	case STATE::INIT:
		initialize();
		break;
	case STATE::TAM:
		trackAndMap();
		break;
	case STATE::RELOCAL:
		relocalize();
		break;
	default:
		break;
	}

	// keep image for tracking
	mGimg.copyTo(mData.previmg);
}

/*!
@brief			draw on images
@param[in,out]	img		color image
*/
void CATAM::draw(cv::Mat &img)
{
	// each process
	switch (mState){
	case STATE::INIT:
		drawTrack(img);
		break;
	case STATE::TAM:
		drawTrack(img);
		drawMap(img);
		drawGrid(img);
		drawChallenge(img);
		break;
	case STATE::RELOCAL:
		drawView(img);
		break;
	default:
		break;
	}

	drawProcess(img);
	drawButton(img);
}

/*!
@brief			draw process
@param[in,out]	img		color image
*/
void CATAM::drawProcess(cv::Mat &img) const
{
	// draw process at the bottom
	cv::Scalar textcol(0, 255, 0);
	cv::putText(img, mText, cv::Point(0, img.rows - 5), cv::FONT_HERSHEY_SIMPLEX, 0.8, textcol, 2);

	// draw fps	at the top
	cv::Scalar fpscol(0, 255, 0);
	cv::putText(img, std::to_string(int(mFPS)) + " FPS", cv::Point(0, 15), cv::FONT_HERSHEY_SIMPLEX, 0.5, fpscol, 1);
}

/*!
@brief		start initialization
*/
void CATAM::startInit(void)
{
	// set initial pose
	mPose.rvec.setTo(0);
	mPose.tvec.setTo(0);

	// start keyframe
	setKeyframe();

	// change state   
	mState = STATE::INIT;
	mText = "Translate camera and press space";
}

/*!
@brief		start tracking and mapping
*/
void CATAM::startTAM(void)
{
	if (makeMap()){	// if map is generated
		// set keyframe
		setKeyframe();

		// change state
		mState = STATE::TAM;
		mText = "Capture calibration board and press space";
	}
}

/*!
@brief		change state
*/
void CATAM::changeState(void)
{
	if (mState == STATE::STOP){
		startInit();		// initializate
	}
	else if (mState == STATE::INIT){
		startTAM();			// start tracking and mapping
	}
	else if (mState == STATE::TAM){
		registerWorld();	// registration with world coordinate system
	}
}

/*!
@brief		reset
*/
void CATAM::reset(void)
{
	mState = STATE::STOP;

	mFrameNumber = 0;
	mFPS = 0.0;
	mChallengeNumber = 0;

	bool tmp = true;

	while (tmp){
		mBAMutex.lock();
		tmp = mDoingBA;
		mBAMutex.unlock();
	}

	mData.clear();

	// change state
	mText = "Press space to start";
	LOGOUT("-------------RESET-------------\n");
}

/*!
@brief		project map to image and recover points
*/
void CATAM::projectMap(void)
{
	// select mapped points that are not in tracks
	std::vector<int> vmapID(mData.map.GetSize());
	for (int i = 0, iend = int(mData.map.GetSize()); i < iend; ++i){
		vmapID[i] = i;
	}

	for (std::list<sTrack>::iterator it = mData.vtrack.begin(),
		itend = mData.vtrack.end();	it != itend; ++it) {
		if (it->ptID != NOID){
			vmapID[it->ptID] = NOID;
		}
	}
	
	std::vector<int> vaddID;
	std::vector<cv::Point3f> vaddpt3d;
	
	for (int i = 0, iend = int(mData.map.GetSize()); i < iend; ++i){
		if (vmapID[i] != NOID){
			vaddID.push_back(i);
			vaddpt3d.push_back(mData.map.GetPoint(i));
		}
	}

	if (vaddpt3d.size() > 0){

		// project points to image
		std::vector<cv::Point2f> vaddpt2d;
		cv::projectPoints(vaddpt3d, mPose.rvec, mPose.tvec, mData.A, mData.D, vaddpt2d);

		std::vector<cv::KeyPoint> &vKpt = mData.vKpt;

		int numrecovered = 0;
		for (int i = 0, iend = int(vaddpt2d.size()); i < iend; ++i){

			if (checkInsideImage(vaddpt2d[i])){		// if inside image

				sTrack track;
				bool added = false;
				double mindist = double(PARAMS.PROJERR);

				// check projected point is close to a keypoint
				for (int j = 0, jend = int(vKpt.size()); j < jend; ++j){
					double dist = cv::norm(vKpt[j].pt - vaddpt2d[i]);
					if (dist < mindist){
						track.kpt = vKpt[j];
						track.ptID = vaddID[i];
						added = true;
						mindist = dist;
					}
				}

				if (added){

					mData.addTrack(track);
					++numrecovered;

					// remove new tracks close to the point
					for (std::list<sTrack>::iterator it = mData.vtrack.begin(),
						itend = mData.vtrack.end();	it != itend; ++it) {

						if (it->ptID == NOID && cv::norm(track.kpt.pt - it->vpt.back()) < PARAMS.PROJERR*2.0f){
							it->ptID = DISCARD;
						}
					}
				}
			}
		}

		mData.clearTrack(DISCARD);

		if (numrecovered){
			LOGOUT("Recovered %d points at %s\n", numrecovered, __FUNCTION__);
		}
	}
}


/*!
@brief		set keyframe for tracking
@retval		set or not
*/
bool CATAM::setKeyframe(void)
{
	mData.clearTrack(NOID);

	sKeyframe tmpKf;
	tmpKf.pose = mPose;		// keyframe pose

	// set mapped points for BA
	for (std::list<sTrack>::iterator it = mData.vtrack.begin(),
		itend = mData.vtrack.end(); it != itend; ++it){
		tmpKf.vpt.push_back(it->vpt.back());
		tmpKf.vptID.push_back(it->ptID);
	}

	// select keypoints as new tracks
	std::vector<int> vnewptID;
	std::vector<cv::KeyPoint> &vKpt = mData.vKpt;

	for (int i = 0, iend = int(vKpt.size()); i < iend; ++i){	// for keypoints

		// check already in track
		bool foundsame = false;
		double mindist = double(PARAMS.PROJERR* 2.0f);
		int ID = NOID;

		for (std::list<sTrack>::iterator it = mData.vtrack.begin(),
			itend = mData.vtrack.end(); it != itend; ++it){

			double dist = cv::norm(vKpt[i].pt - it->vpt.back());

			if (it->ptID != NOID && dist < mindist){	// if mapped track exists
				ID = it->ptID;
				mindist = dist;
				foundsame = true;
			}
		}

		if (!foundsame){		// if keypoints not in mapped tracks
			vnewptID.push_back(i);
		}
		else{					// if keypoints in mapped tracks
			tmpKf.vkpt.push_back(vKpt[i]);
			tmpKf.vkptID.push_back(ID);
		}
	}

	if (int(vnewptID.size()) < PARAMS.MINPTS){	// not suitable if new keypoints are less
		return false;
	}

	// add new points in new tracks
	for (int i = 0, iend = int(vnewptID.size()); i < iend; ++i){
		sTrack tmpTrack;
		tmpTrack.kpt = vKpt[vnewptID[i]];
		tmpTrack.ptID = NOID;
		mData.addTrack(tmpTrack);
	}

	// compute descriptor
	mGimg.copyTo(tmpKf.img);
	cv::Mat vdesc;
	if (tmpKf.vkpt.size() != 0){
		mDetector.Describe(tmpKf.img, tmpKf.vkpt, vdesc);
	}

	mData.map.AddKeyframe(tmpKf, vdesc);

	return true;
}

/*!
@brief		check inside image or not
@param[in]	pt	point
@retval		inside or not
*/
bool CATAM::checkInsideImage(const cv::Point2f &pt) const
{
	if (pt.x < PARAMS.PATCHSIZE * 2
		|| mGimg.cols - PARAMS.PATCHSIZE * 2 < pt.x
		|| pt.y < PARAMS.PATCHSIZE * 2
		|| mGimg.rows - PARAMS.PATCHSIZE * 2 < pt.y){
		return false;
	}
	else{
		return true;
	}
}


/*!
@brief		track frame using KLT
@retval		number of tracked mapped points
*/
int CATAM::trackFrame(void)
{
	// KLT
	std::vector<cv::Point2f> vtracked;
	std::vector<unsigned char> vstatus;
	std::vector<float> verror;
	const cv::Size patch(PARAMS.PATCHSIZE, PARAMS.PATCHSIZE);
	const int level = 1;

	cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 10, 0.1);
	cv::calcOpticalFlowPyrLK(mData.previmg, mGimg, mData.vprevpt, vtracked, vstatus, verror);

	int count = 0;
	mData.vprevpt.clear();
	std::list<sTrack>::iterator it = mData.vtrack.begin();
	for (size_t i = 0; i < vstatus.size(); ++i){
		if (!vstatus[i] || !checkInsideImage(vtracked[i])){	// remove if point not tracked
			it = mData.vtrack.erase(it);
		}
		else{
			mData.vprevpt.push_back(vtracked[i]);
			it->vpt.push_back(vtracked[i]);

			if (it->ptID != NOID){
				++count;
			}
			++it;
		}
	}

	return count;
}

/*!
@brief		draw tracked points
@param[in]	img		image
*/
void CATAM::drawTrack(cv::Mat &img) const
{
	const int pointsize = 5;

	const std::list<sTrack> &vtrack = mData.vtrack;

	cv::Scalar border(0, 0, 0);			// circle border
	cv::Scalar mapped(255, 255, 255);	// mapped
	cv::Scalar newpt(255, 0, 0);		// not mapped

	for (std::list<sTrack>::const_iterator it = vtrack.begin(), itend = vtrack.end();
		it != itend; ++it){
		if (it->ptID != NOID){
			cv::circle(img, cv::Point(it->vpt.back()), pointsize, border, -1);
			cv::circle(img, cv::Point(it->vpt.back()), pointsize - 1, mapped, -1);
		}
		else{
			cv::circle(img, cv::Point(it->vpt.back()), pointsize, border, -1);
			cv::circle(img, cv::Point(it->vpt.back()), pointsize - 1, newpt, -1);
		}
	}
}

/*!
@brief		match with keyframe near mPose amd recover points
@retval		matched or not
*/
bool CATAM::matchKeyframe(void)
{
	// compute descriptors of current frame
	std::vector<cv::KeyPoint> &vKpt = mData.vKpt;
	if (vKpt.size() < PARAMS.MINPTS){
		return false;
	}
	cv::Mat vdesc;
	mDetector.Describe(mGimg, vKpt, vdesc);

	// get nearest keyframe
	const sKeyframe& kf = mData.map.GetNearestKeyframe(mPose);
	if (kf.vkpt.size() < PARAMS.MINPTS){		// not enough keypoint in keyframe
		return false;
	}

	// matching
	std::vector<cv::DMatch> vmatch;
	mDetector.Match(vdesc, kf.vdesc, vmatch);

	std::vector<cv::Point3f> vpt3d;
	std::vector<cv::Point2f> vpt2d;
	std::vector<int> vID;

	// select good matches
	for (int i = 0, iend = int(vmatch.size()); i < iend; ++i){
		if (vmatch[i].distance < PARAMS.DESCDIST){	
			vpt2d.push_back(vKpt[vmatch[i].queryIdx].pt);

			int ID = vmatch[i].trainIdx;
			vpt3d.push_back(mData.map.GetPoint(kf.vkptID[ID]));
			vID.push_back(kf.vkptID[ID]);
		}
	}

	if (vpt2d.size() > PARAMS.MINPTS){	// if enough good correspondences

		// compute camera pose
		sPose tmpPose = mPose;
		const int iteration = 100;
		const double confidence = 0.98;
		std::vector<int> vinliers;

		cv::solvePnPRansac(vpt3d, vpt2d, mData.A, mData.D, tmpPose.rvec, tmpPose.tvec, true, iteration, PARAMS.PROJERR, confidence, vinliers);

		// check number of inliers and inlier ratio (inlier / all)
		if (int(vinliers.size()) > PARAMS.MINPTS
			&& float(vinliers.size()) / float(vpt2d.size()) > PARAMS.MATCHKEYFRAME){

			// add as a mapped track
			int numrecovered = 0;
			for (int i = 0, iend = int(vinliers.size()); i < iend; ++i){

				// check already in mapped tracks
				bool found = false;
				int pos = vinliers[i];
				for (std::list<sTrack>::iterator it = mData.vtrack.begin(),
					itend = mData.vtrack.end();	it != itend; ++it) {

					if (it->ptID == vID[pos]){
						found = true;
						break;
					}
				}

				if (!found){
					sTrack track;
					track.kpt.pt = vpt2d[pos];
					track.ptID = vID[pos];
					mData.addTrack(track);
					++numrecovered;
				}
			}

			if (mState == STATE::RELOCAL){
				mPose = tmpPose;
			}

			LOGOUT("Recovered %d points with keyframe %d at %s\n", numrecovered, kf.ID, __FUNCTION__);
			return true;
		}
	}

	return false;
}

/*!
@brief		compute camera pose
@retval		computed or not
*/
bool CATAM::computePose(void)
{
	// pose estimation
	std::vector<cv::Point2f> vpt2d;
	std::vector<cv::Point3f> vpt3d;

	for (std::list<sTrack>::iterator it = mData.vtrack.begin(),
		itend = mData.vtrack.end();	it != itend; ++it) {
		if (it->ptID != NOID){
			vpt2d.push_back(it->vpt.back());
			vpt3d.push_back(mData.map.GetPoint(it->ptID));
		}
	}

	cv::solvePnP(vpt3d, vpt2d, mData.A, mData.D, mPose.rvec, mPose.tvec, true);

	// check reprojection error and discard points if error is large
	std::vector< cv::Point2f > vrepropt;
	cv::projectPoints(vpt3d, mPose.rvec, mPose.tvec, mData.A, mData.D, vrepropt);

	int numall = 0;
	int numdiscard = 0;

	for (std::list<sTrack>::iterator it = mData.vtrack.begin(),
		itend = mData.vtrack.end();	it != itend; ++it) {
		if (it->ptID != NOID){

			double dist = cv::norm(vrepropt[numall] - vpt2d[numall]);

			if (dist > PARAMS.PROJERR){
				it->ptID = DISCARD;
				++numdiscard;
			}
			++numall;
		}
	}

	mData.clearTrack(DISCARD);
	LOGOUT("Discarded:%d used:%d at %s\n", numdiscard, numall - numdiscard, __FUNCTION__);

	mData.quality = double(numall - numdiscard) / double(numall);		// inlier ratio

	if (mData.havescale){
		transformToWorld(mPose, mWPose);
	}

	if (numall - numdiscard > PARAMS.MINPTS){
		return true;
	}
	else{
		return false;
	}
}


/*!
@brief		compute pose from essential matrix
@param[in]	vpt1	undistorted point list1
@param[in]	vpt2	undistorted point list2
@param[out]	rvec	rotation vector
@param[out]	tvec	translation vector
*/
void CATAM::computePosefromE(
	const std::vector<cv::Point2f> &vpt1,
	const std::vector<cv::Point2f> &vpt2,
	cv::Mat &rvec,
	cv::Mat &tvec
	) const
{
	double focal = 1.0;
	cv::Point2d pp = cv::Point2d(0, 0);
	int method = cv::LMEDS;
	double prob = 0.99;
	double th = 1.0 / mData.focal;

	cv::Mat mask;
	cv::Mat E = cv::findEssentialMat(vpt1, vpt2, focal, pp, method, prob, th, mask);

	cv::Mat R;
	cv::recoverPose(E, vpt1, vpt2, R, tvec, focal, pp, mask);

	cv::Rodrigues(R, rvec);
}

/*!
@brief		triangulation
@param[in]	vpt1	undistorted points1
@param[in]	vpt2	undistorted points2
@param[in,out]	pose1	camera pose of vpt1
@param[in,out]	pose2	camera pose of vpt2
@param[out]	vpt3d		triangulated points
@retval		true or false
*/
void CATAM::triangulate(
	const std::vector<cv::Point2f> &vpt1,
	const std::vector<cv::Point2f> &vpt2,
	const sPose &pose1,
	const sPose &pose2,
	std::vector<cv::Point3f> &vpt3d
	) const
{
	cv::Mat R1, R2;

	cv::Rodrigues(pose1.rvec, R1);
	cv::Rodrigues(pose2.rvec, R2);

	cv::Mat P1(3, 4, R1.type()), P2(3, 4, R2.type());
	R1.copyTo(P1(cv::Rect(0, 0, 3, 3)));
	R2.copyTo(P2(cv::Rect(0, 0, 3, 3)));

	pose1.tvec.copyTo(P1(cv::Rect(3, 0, 1, 3)));
	pose2.tvec.copyTo(P2(cv::Rect(3, 0, 1, 3)));

	cv::Mat triangulated;
	cv::triangulatePoints(P1, P2, vpt1, vpt2, triangulated);

	vpt3d.resize(vpt1.size());

	for (int i = 0, iend = int(vpt1.size()); i < iend; ++i){

		float x = triangulated.at < float >(0, i);
		float y = triangulated.at < float >(1, i);
		float z = triangulated.at < float >(2, i);
		float w = triangulated.at < float >(3, i);

		vpt3d[i].x = x / w;
		vpt3d[i].y = y / w;
		vpt3d[i].z = z / w;
	}
}

/*!
@brief		BA for initial map
@param[in,out]		vpt3d		3d points
@param[in]			vundist1	undistorted points1
@param[in]			vundist2	undistorted points2
@param[in,out]		pose1		pose 1
@param[in,out]		pose2		pose 2
*/
bool CATAM::initialBA(
	std::vector<cv::Point3f> &vpt3d,
	const std::vector<cv::Point2f> &vundist1,
	const std::vector<cv::Point2f> &vundist2,
	sPose &pose1,
	sPose &pose2
	)
{
#ifdef WITHBA
	std::vector< std::vector<cv::Point2f> > imagePoints(2);
	imagePoints[0] = vundist1;
	imagePoints[1] = vundist2;

	std::vector< std::vector<int> > visibility(2);
	std::vector<int> vvis(vundist1.size(), 1);
	visibility[0] = vvis;
	visibility[1] = vvis;

	std::vector<cv::Mat> cameraMatrix(2);
	cameraMatrix[0] = cv::Mat::eye(cv::Size(3, 3), CV_64F);
	cameraMatrix[1] = cv::Mat::eye(cv::Size(3, 3), CV_64F);

	std::vector<cv::Mat> R(2);
	pose1.rvec.copyTo(R[0]);
	pose2.rvec.copyTo(R[1]);

	std::vector<cv::Mat> T(2);
	pose1.tvec.copyTo(T[0]);
	pose2.tvec.copyTo(T[1]);

	std::vector<cv::Mat> distCoeffs(2);
	distCoeffs[0] = cv::Mat::zeros(cv::Size(1, 5), CV_64F);
	distCoeffs[1] = cv::Mat::zeros(cv::Size(1, 5), CV_64F);

	LOGOUT("initial BA with %d points\n", vpt3d.size());
	double val = mBA.run(vpt3d, imagePoints, visibility, cameraMatrix, R, T, distCoeffs);
	LOGOUT("Initial error = \t %f\n", mBA.getInitialReprjError());
	LOGOUT("Final error = \t %f\n", mBA.getFinalReprjError());

	if (val < 0.0){
		return false;
	}

	R[0].copyTo(pose1.rvec);
	R[1].copyTo(pose2.rvec);

	T[0].copyTo(pose1.tvec);
	T[1].copyTo(pose2.tvec);
#endif
	return true;
}

/*!
@brief		mapping
@retval		new map is generated or not
*/
bool CATAM::makeMap(void)
{
	// select start and end points from new tracks
	std::vector<cv::Point2f> vstart;
	std::vector<cv::Point2f> vend;

	std::vector<sTrack*> vnewtrack;
	for (std::list<sTrack>::iterator it = mData.vtrack.begin(),
		itend = mData.vtrack.end();	it != itend; ++it) {
		if (it->ptID == NOID){
			vstart.push_back(it->vpt[0]);
			vend.push_back(it->vpt.back());
			vnewtrack.push_back(&(*it));
		}
	}

	// if not enough points
	if (vstart.size() < PARAMS.MINPTS){
		return false;
	}

	// undistort
	std::vector<cv::Point2f> vundiststart, vundistend;
	cv::undistortPoints(vstart, vundiststart, mData.A, mData.D);
	cv::undistortPoints(vend, vundistend, mData.A, mData.D);

	// compute initial pose from essential matrix for initialization
	if (mState == STATE::INIT){
		computePosefromE(vundiststart, vundistend, mPose.rvec, mPose.tvec);
	}

	sKeyframe &lkf = mData.map.GetLastKeyframe();

	// triangulation
	std::vector<cv::Point3f> vpt3d;
	triangulate(vundiststart, vundistend, lkf.pose, mPose, vpt3d);

	if (mState == STATE::INIT){				// two view BA
		sPose tmpPose = lkf.pose;
		if (!initialBA(vpt3d, vundiststart, vundistend, tmpPose, mPose)){
			return false;
		}
		else{
			lkf.pose = tmpPose;
		}
	}

	// check triangulation with reprojection error
	std::vector<cv::Point2f> vppt1, vppt2;
	cv::projectPoints(vpt3d, lkf.pose.rvec, lkf.pose.tvec, cv::Mat::eye(3, 3, CV_64F), cv::Mat(), vppt1);
	cv::projectPoints(vpt3d, mPose.rvec, mPose.tvec, cv::Mat::eye(3, 3, CV_64F), cv::Mat(), vppt2);

	int numinliers = 0;
	std::vector<bool> vinlier(vpt3d.size(), false);
	std::vector<sTrack*>::iterator it = vnewtrack.begin();

	std::vector<cv::Point3f> vinpt3d;
	std::vector<cv::KeyPoint> vinkpt;

	for (int i = 0, iend = int(vpt3d.size()); i < iend; ++i, ++it){

		double dist = cv::norm(vundiststart[i] - vppt1[i]) + cv::norm(vundistend[i] - vppt2[i]);
		dist /= 2.0;
		dist *= mData.focal;

		if (dist < PARAMS.PROJERR){
			vinlier[i] = true;
			vinpt3d.push_back(vpt3d[i]);		// 3d cooridnate
			vinkpt.push_back((*it)->kpt);		// keypoint
			++numinliers;
		}
	}

	// for initialization
	if (mState == STATE::INIT){

		if (float(numinliers) / float(vnewtrack.size()) < PARAMS.GOODINIT){
			LOGOUT("Not enough inliers for initialization\n");
			return false;
		}
		else{
			// compute middle of two frames
			cv::Point3f middle = (cv::Point3f(mPose.tvec) + cv::Point3f(lkf.pose.tvec)) / 2.0f;
			double distkeyframe = cv::norm(lkf.pose.tvec - mPose.tvec);

			struct dist3D{
				double dist;
				int ID;
				bool operator< (const dist3D &r) const { return dist < r.dist; }
			};

			// compute distance to median of vinpt3d
			std::vector<dist3D> vdist3D;
			for (int i = 0, iend = int(vinpt3d.size()); i < iend; ++i){
				dist3D tmp;
				tmp.ID = i;
				tmp.dist = cv::norm(middle - vinpt3d[i]);
				vdist3D.push_back(tmp);
			}
			std::sort(vdist3D.begin(), vdist3D.end());

			double distpoints = cv::norm(vinpt3d[vdist3D[int(vdist3D.size())/2].ID] - middle);

			if (PARAMS.BASETAN > distkeyframe / distpoints){
				LOGOUT("Not enough baseline\n");
				return false;
			}
		}
	}

	// add points to keyframe and map
	cv::Mat vindesc;
	if (vinkpt.size() != 0){
		mDetector.Describe(lkf.img, vinkpt, vindesc);
	}

	std::vector<int> vid;		// ID of points
	mData.map.UpdateLastKeyframe(vinpt3d, vinkpt, vindesc, vid);

	// set point ID to new tracks
	it = vnewtrack.begin();
	int counter = 0;
	for (int i = 0, iend = int(vinlier.size()); i < iend; ++i, ++it){
		if (vinlier[i]){
			(*it)->ptID = vid[counter];
			++counter;
		}
	}

	return true;
}

/*!
@brief		draw map
@param[in]	img		color image
*/
void CATAM::drawMap(cv::Mat &img) const
{
	const std::list<sTrack> &vtrack = mData.vtrack;

	std::vector<cv::Point3f> vpt3d;
	
	// select mapped tracks
	for (std::list<sTrack>::const_iterator it = vtrack.begin(), itend = vtrack.end();
		it != itend; ++it){
		if (it->ptID != NOID){
			vpt3d.push_back(mData.map.GetPoint(it->ptID));
		}
	}

	// project mapped tracks
	if (vpt3d.size() != 0){
		std::vector<cv::Point2f> vpt2d;
		cv::projectPoints(vpt3d, mPose.rvec, mPose.tvec, mData.A, mData.D, vpt2d);

		cv::Scalar col(0, 0, 255);		// map color
		const int size = 1;
		for (int i = 0, iend = int(vpt2d.size()); i < iend; ++i){
			cv::circle(img, cv::Point(vpt2d[i]), size, col, -1);
		}
	}
}

/*!
@brief		draw grid on calibration board
@param[in]	img		color image
*/
void CATAM::drawGrid(cv::Mat &img) const
{
	if (mData.havescale){	// scale already estimated

		sPose tmp;
		transformToWorld(mPose, tmp);

		// line drawing
		cv::Scalar col(0, 255, 0);
		const int linewidth = 2;

		const int size = mCalibrator.GetShortSide();
		std::vector<cv::Point3f> vpt3d(2 * size);		// for two sides
		const float interval = mCalibrator.GetSize();

		// y axis
		for (int i = 0; i < size; ++i){
			vpt3d[2 * i] = cv::Point3f(interval*i, 0, 0);
			vpt3d[2 * i + 1] = cv::Point3f(interval*i, interval*(size - 1), 0);
		}

		std::vector<cv::Point2f> vpt2d;
		cv::projectPoints(vpt3d, tmp.rvec, tmp.tvec, mData.A, mData.D, vpt2d);

		for (int i = 0, iend = int(vpt2d.size()) / 2; i < iend; ++i){
			cv::line(img, cv::Point(vpt2d[2 * i]), cv::Point(vpt2d[2 * i + 1]), col, linewidth);
		}

		// x axis
		for (int i = 0; i < size; ++i){
			vpt3d[2 * i] = cv::Point3f(0, interval*i, 0);
			vpt3d[2 * i + 1] = cv::Point3f(interval*(size - 1), interval*i, 0);
		}

		cv::projectPoints(vpt3d, tmp.rvec, tmp.tvec, mData.A, mData.D, vpt2d);

		for (int i = 0, iend = int(vpt2d.size()) / 2; i < iend; ++i){
			cv::line(img, cv::Point(vpt2d[2 * i]), cv::Point(vpt2d[2 * i + 1]), col, linewidth);
		}
	}
}


/*!
brief		check criteria for adding a new keyframe
@retval		do mapping or not
*/
bool CATAM::mappingCriteria(void) const
{
	// middle point between current frame and nearest keyframe
	const sKeyframe &nkf = mData.map.GetNearestKeyframe(mPose);
	double distkeyframe = cv::norm(mPose.tvec - nkf.pose.tvec);
	cv::Point3f middle = (cv::Point3f(mPose.tvec) + cv::Point3f(nkf.pose.tvec)) / 2.0f;

	// select median of mapped points
	struct dist3D{
		double dist;
		int ID;
		bool operator< (const dist3D &r) const { return dist < r.dist; }
	};

	std::vector<dist3D> vdist3D;
	for (std::list<sTrack>::const_iterator it = mData.vtrack.begin(), itend = mData.vtrack.end();
		it != itend; ++it){
		if (it->ptID != NOID){
			dist3D tmp;
			tmp.ID = it->ptID;
			tmp.dist = cv::norm(mData.map.GetPoint(it->ptID) - middle);
			vdist3D.push_back(tmp);
		}
	}

	std::sort(vdist3D.begin(), vdist3D.end());
	const cv::Point3f &median = mData.map.GetPoint(vdist3D[int(vdist3D.size()) / 2].ID);

	double distpoints = cv::norm(median - middle);

	// mapping criteria
	if (PARAMS.BASETAN < distkeyframe / distpoints){
		return true;
	}

	return false;
}

/*!
@brief		mapping
*/
void CATAM::mapping(void)
{
	makeMap();

	setKeyframe();

#ifndef MULTITHREAD
	localBA();
#endif				
}

/*!
@brief		tracking and mapping
*/
void CATAM::trackAndMap(void)
{
	bool relocal = false;

	if (mData.vprevpt.size() < PARAMS.MINPTS){		// not enough tracked points
		relocal = true;
	}
	else{

		int mappedpts = trackFrame();		// track points

		if (mappedpts < PARAMS.MINPTS){		// not enough mapped points
			relocal = true;					// start relocalization
		}
		else{
			if (!computePose()){
				relocal = true;
			}
			else{
				projectMap();			// match by map projection
				matchKeyframe();		// match with keyframe
				
				if (mappingCriteria()){	// check mapping criteria
					mapping();			// mapping
				}
			}
		}
	}

	if (relocal){
		mData.clearTrack();
		mData.map.GetGoodPoseforRelocalization(mPose);
		LOGOUT("Lost\n");
		mText = "Go back to this view";
		mState = STATE::RELOCAL;
	}
}

/*!
@brief		initialization
*/
void CATAM::initialize(void)
{
	if (mData.vprevpt.size() > PARAMS.MINPTS){	// if tracking points exist
		trackFrame();
	}
	else{
		LOGOUT("Initialization failed\n");
		reset();
	}
}

/*!
@brief		loca challenge points
@param[in]	file name
*/
void CATAM::loadChallenge(const std::string &name)
{
	std::ifstream in(name);

	if (in.is_open()){

		int num;
		in >> num;

		for (int i = 0; i < num; ++i){
			if (!in.eof()){
				int id;
				cv::Point3f pt;
				in >> id >> pt.x >> pt.y >> pt.z;
				mData.vtarget.insert(std::pair<int, cv::Point3f>(id, pt));
			}
		}
	}
	else{
		LOGOUT("%s not found\n", name.c_str());

		// just show origin of world coordinate system
		mData.vtarget.insert(std::pair<int, cv::Point3f>(0, cv::Point3f(0, 0, 0)));
	}
}

/*!
@brief				draw challenge coordinates
@param[in,out]		img		color image
*/
void CATAM::drawChallenge(cv::Mat &img)
{
	if (mData.havescale){

		cv::Scalar col(0, 0, 255);

		const int linewidth = 2;
		const int radius = 15;

		// world pose computation
		sPose tmp;
		transformToWorld(mPose, tmp);

		std::vector<cv::Point2f> vpt2d;
		std::vector<cv::Point3f> vpt3d;
		int ID = 0;

		// draw targets defined in world coordinate system
		int i = 0;
		for (std::map<int, cv::Point3f>::const_iterator it = mData.vtarget.begin(),
			itend = mData.vtarget.end(); it != itend; ++it, ++i){
			if (mChallengeNumber == i){
				ID = it->first;
				vpt3d.push_back(it->second);
				break;
			}
		}

		if (vpt3d.size() == 1){
			cv::projectPoints(vpt3d, tmp.rvec, tmp.tvec, mData.A, mData.D, vpt2d);

			cv::Point pt = vpt2d[0];
			cv::circle(img, pt, radius, col, linewidth);
			cv::putText(img, std::to_string(ID), pt + cv::Point(radius, 0), cv::FONT_HERSHEY_SIMPLEX, 1.5, col, 2);
		}
		else{
			mText = "No more challenge points";
		}
	}
}

/*!
@brief		transfrom local coordinate to world coordinate
@param[in]	local		local coordinate
@param[out]	world		world coordinate
*/
void CATAM::transformToWorld(const sPose &local, sPose &world) const
{
	// pose computation
	cv::Mat tmpM;
	local.getM(tmpM);

	tmpM *= mData.scale;

	cv::Mat M = tmpM * mData.transMat;

	cv::Rodrigues(M(cv::Rect(0, 0, 3, 3)), world.rvec);
	M(cv::Rect(3, 0, 1, 3)).copyTo(world.tvec);
}

/*!
@brief		acquire world coordinate system from a calibration pattern
@param[in]	pose	pose
*/
bool CATAM::getWorldCoordinate(sPose &pose) const
{
	bool found = mCalibrator.PoseEstimation(mGimg, mData.A, mData.D, pose.rvec, pose.tvec);

	if (!found){
		LOGOUT("Calibration board not found\n");
	}

	return found;
}

/*!
@brief		registration with world coordinate system
*/
void CATAM::registerWorld(void)
{
	// get pair of world and local pose
	sPose world;
	if (getWorldCoordinate(world)){
		std::pair<sPose, sPose> tmp;
		tmp.first = world;		// world pose
		tmp.second = mPose;		// local pose
		mData.vposePair.push_back(tmp);
		mText = "Translate camera and capture again";
	}

	if (mData.vposePair.size() > 1){

		std::vector<double> vscale;

		// compute scale from all pairs
		for (int i = 0, iend = int(mData.vposePair.size()) - 1; i < iend; ++i){

			// compute distance between two frames and compute ratio
			for (int j = i + 1, jend = int(mData.vposePair.size()); j < jend; ++j){

				cv::Mat Ri, Rj;
				mData.vposePair[i].second.getR(Ri);
				mData.vposePair[j].second.getR(Rj);

				cv::Mat R = Ri * Rj.inv();
				double numerator = cv::norm(mData.vposePair[i].first.tvec - R * mData.vposePair[j].first.tvec);
				double denominator = cv::norm(mData.vposePair[i].second.tvec - R * mData.vposePair[j].second.tvec);

				vscale.push_back(numerator / denominator);
			}
		}

		// average scale
		mData.scale = std::accumulate(vscale.begin(), vscale.end(), 0.0) / double(vscale.size());

		// compute transformation matrix from local to world after scaling from pair of median scale
		cv::Mat transMat = cv::Mat::zeros(cv::Size(4, 4), CV_64F);
		int size = int(mData.vposePair.size());
		for (int i = 0; i < size; ++i){
			sPose world = mData.vposePair[i].first;
			sPose local = mData.vposePair[i].second;
			local.tvec *= mData.scale;

			cv::Mat worldM, localM;
			world.getM(worldM);
			local.getM(localM);

			cv::add(transMat.clone(), localM.inv() * worldM, transMat);
		}

		// average rotation matrix
		cv::Mat X, D, Y;
		cv::SVD::compute(transMat(cv::Rect(0, 0, 3, 3)), D, X, Y);
		mData.transMat(cv::Rect(0, 0, 3, 3)) = (X * Y) / mData.scale;

		// average translation vector
		mData.transMat(cv::Rect(3, 0, 1, 3)) = transMat(cv::Rect(3, 0, 1, 3)) / double(size) / mData.scale;

		mData.havescale = true;

		mText = "Capture more for improvement";
	}
}

/*!
@brief		local bundle adjustment
*/
void CATAM::localBA(void)
{
#ifdef WITHBA
	while (1){

		STATE mainState = mState;

		if (mainState == STATE::TAM){

			// get data from map
			bool copied = mData.map.CopytoBA(mData.baData);
			std::vector<sKeyframe> &vkf = mData.baData.vkeyframe;
			int numkeyframes = int(vkf.size());

			if (copied && numkeyframes > 2){

				mBAMutex.lock();
				mDoingBA = true;
				mBAMutex.unlock();

				// check visibility of mapped points in each keyframe
				std::vector<cv::Point3f> &vpt3d = mData.baData.vpt3d;
				std::vector<int> checkvis(vpt3d.size(), 0);

				for (int i = 0, iend = numkeyframes; i < iend; ++i){
					sKeyframe &kf = vkf[i];
					for (int j = 0, jend = int(kf.vptID.size()); j < jend; ++j){
						++checkvis[kf.vptID[j]];
					}
				}

				// select visible mapped points
				std::vector<cv::Point3f> vusedpt3d;
				std::vector<int> vvisibleID;

				for (int i = 0, iend = int(checkvis.size()); i < iend; ++i){
					if (checkvis[i] > 1){	// should be visible in more than two views
						int num = int(vusedpt3d.size());
						checkvis[i] = num;
						vvisibleID.push_back(i);
						vusedpt3d.push_back(vpt3d[i]);
					}
					else{
						checkvis[i] = -1;
					}
				}

				// set data for cvsba
				std::vector< std::vector<cv::Point2f> > imagePoints(numkeyframes);
				std::vector< std::vector<int> > visibility(numkeyframes);
				std::vector<cv::Mat> cameraMatrix(numkeyframes);
				std::vector<cv::Mat> R(numkeyframes);
				std::vector<cv::Mat> T(numkeyframes);
				std::vector<cv::Mat> distCoeffs(numkeyframes);

				for (int i = 0; i < numkeyframes; ++i){

					std::vector<cv::Point2f> points(vusedpt3d.size());
					std::vector<int> vis(vusedpt3d.size(), 0);

					sKeyframe &kf = vkf[i];
					for (int j = 0, jend = int(kf.vptID.size()); j < jend; ++j){
						int id = checkvis[kf.vptID[j]];

						if (id != -1){
							points[id] = kf.vpt[j];
							vis[id] = 1;
						}
					}

					imagePoints[i] = points;
					visibility[i] = vis;
					cameraMatrix[i] = mData.A;
					distCoeffs[i] = mData.D;
					kf.pose.rvec.copyTo(R[i]);
					kf.pose.tvec.copyTo(T[i]);
				}

				LOGOUT("BA started with %d points %d frames\n", vpt3d.size(), numkeyframes);
				double val = mBA.run(vusedpt3d, imagePoints, visibility, cameraMatrix, R, T, distCoeffs);

				if (val < 0){
					LOGOUT("BA failed\n");
				}
				else{
					LOGOUT("Initial error = \t %f\n", mBA.getInitialReprjError());
					LOGOUT("Final error = \t %f\n", mBA.getFinalReprjError());

					for (int i = 0; i < numkeyframes; ++i){
						sKeyframe &kf = vkf[i];

						R[i].copyTo(kf.pose.rvec);
						T[i].copyTo(kf.pose.tvec);
					}

					for (int i = 0, iend = int(vvisibleID.size()); i < iend; ++i){
						int id = vvisibleID[i];
						vpt3d[id] = vusedpt3d[i];
					}

					mData.baData.vvisibleID = vvisibleID;
					mData.map.CopyfromBA(mData.baData);
				}

				mBAMutex.lock();
				mDoingBA = false;
				mBAMutex.unlock();
			}
		}
		else if (mainState == STATE::CLOSE){
			break;
		}

#ifndef MULTITHREAD
		break;
#endif
	}
#endif
}

/*!
@brief		change relocalization view
*/
void CATAM::changeRelocalImage(void)
{
	mData.map.GetPoseforRelocalization(mPose);
}

/*!
@brief		relocalization
*/
void CATAM::relocalize(void)
{
	if (mData.vKpt.size() == 0){
		return;
	}
	else if (matchKeyframe()){
		if (mData.havescale){
			mText = "Relocalized";
		}
		else{
			mText = "Capture calibration board and press space";
		}
		
		LOGOUT("Relocalized\n");
		mState = STATE::TAM;
	}
}

/*!
@brief			overlay view
@param[in,out]	img		color image
*/
void CATAM::drawView(cv::Mat &img)
{
	const sKeyframe& kf = mData.map.GetNearestKeyframe(mPose);

	cv::Mat mono = cv::Mat(img.size(), CV_8U);
	cv::Canny(kf.img, mono, 10, 100);

	std::vector<cv::Mat> color(3, mono);

	cv::Mat cimg;
	cv::merge(color, cimg);

	double alpha = 0.5;
	double beta = (1.0 - alpha);
	addWeighted(img.clone(), alpha, cimg, beta, 0.0, img);
}