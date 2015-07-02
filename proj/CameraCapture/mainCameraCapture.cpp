/*!
@file		mainCameraCapture.cpp
@brief		save video
*/

#include "opencv2/opencv.hpp"
#include "Cam.h"
#include "DirectoryConfig.h"

/*!
@brief		main function
*/
int main(int argc, char **argv)
{
	// start camera
	CCam cam;
	
	int width, height, channel;

	const bool load = false;						// load video or not
	std::string name = strData + "movie.avi";		// file name

	const bool save = true;		// save video or not

	if (load){
		if (!cam.OpenVideo(name, width, height, channel)){	// open camera
			return -1;
		}
	}
	else{
		if (!cam.Open(width, height, channel)){	// open camera
			return -1;
		}
		if (save){
			cam.SaveVideo(name);
		}
	}

	int type = channel == 1 ? CV_8UC1 : CV_8UC3;
	cv::Mat img(cv::Size(width, height), type);	// allocate image array

	// make window
	const char window[256] = "camera";
	cv::namedWindow(window);

	bool saveflag = false;

	printf("Press space to start and stop saving video\n");

	while (1){
		cam.Get(img);

		// show image
		cv::imshow(window, img);

		if (saveflag){
			cam.SaveImage();
		}

		int key = cv::waitKey(10);
		if (key == 0x1b){	// ESC
			break;
		}
		else if (key == ' '){
			saveflag = !saveflag;
			if (saveflag){
				printf("start\n");
			}
			else{
				printf("stop\n");
			}
		}
	}

	// close camera
	cam.Close();

	return 0;
}
