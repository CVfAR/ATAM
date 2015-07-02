# About

Abecedary tracking and mapping (ATAM): a toolkit for tracking competition  
Hideaki Uchiyama, Takafumi Taketomi, Sei Ikeda and Joao Paulo Silva do Monte Lima  

# Dependency
- OpenCV 3.0
- cvsba (if use bundle adjustment)
- LAPACK (if use bundle adjustment)

# Installation (on Windows)
## 1. Download and install following software

- CMake 2.8 (or other versions may be OK) from http://www.cmake.org/
- Visual Studio 2013 (or other IDEs may be OK)
		

## 2. Download and unzip following software somewhere (e.g. in C:/...../3rdparty)

- OpenCV 3.0 from http://opencv.org/
 - Set dll path (e.g. opencv/build/x64/vc12/bin if use VS 2013 and 64 bit compiler) to environment variable (PATH)
 - May need to uncheck all options for OpenCL (WITH\_OPENCL, WITH\_OPENCLAMDBLAS, WITH\_OPENCLAMDFFT, WITH\_OPENCL\_SVM) on CMake and compile OpenCV by yourself if ATAM crashes (especially on Mac)
- cvsba (cvsba-1.0.0.tgz) from http://www.uco.es/investiga/grupos/ava/node/39
 - UNIX users may need #include<limits> in utils/test_cvsba.cpp
 - UNIX users can install CLAPACK as instructed on the web
- CLAPACK (clapack-3.2.1-CMAKE.tgz) from http://www.netlib.org/clapack/

## 3. Compile CLAPACK with CMake

- Set clapack-3.2.1-CMAKE to "Where is the source code"
- Set clapack-3.2.1-CMAKE/build to "Where to build the binaries"
- Press Configure and Generate
- Open clapack-3.2.1-CMAKE/build/CLAPACK.sln and compile it in Release mode
		
## 4. Copy all files in cvsba-1.0.0/src/ to ATAM/src/cvsba/

## 5. Compile ATAM with CMake

- Set ATAM to "Where is the source code"	
- Set ATAM/build to "Where to build the binaries"
- Press Configure
- Set OpenCV_DIR to opencv/build/
- Press Configure
- Set LAPACK\_LIB to clapack-3.2.1-CMAKE/build/SRC/Release/lapack.lib (not necessary if not check USE\_BA)
- Set BLAS\_LIB to clapack-3.2.1-CMAKE/build/BLAS/SRC/Release/blas.lib (not necessary if not check USE\_BA)
- Set F2C\_LIB to clapack-3.2.1-CMAKE/build/F2CLIBS/libf2c/Release/libf2c.lib (not necessary if not check USE\_BA)
- Press Configure and Generate

### options for ATAM
- Check SHOW_LOG if output log on console window
- Check USE_BA if use bundle adjustment (BA)
- Check USE_MULTITHREAD if use multi threads for BA
- Check USE_CHESSBOARD if use a chessboard for camera calibration, otherwise a dot pattern
- Check USE_PGR if use a point grey camera, otherwise use a camera with OpenCV
 - Set PGR\_INCLUDE\_DIR to C:/Program Files/Point Grey Research/FlyCapture2/include (depends on where installed Flycapture)
 - Set PGR\_LIB to C:/Program Files/Point Grey Research/FlyCapture2/lib64/FlyCapture2.lib (depends on where installed Flycapture)


# How to use (on Windows)
### Set a target project (CameraCapture, CameraCalibration and ATAM) as StartUp Project when initially open each .sln

## 1. CameraCapture
- Open ATAM/build/proj/CameraCapture/CameraCapture.sln, compile and run it
 - May need to change mDeviceID to 1 or 2 at line 12 in Cam.cpp for a USB camera on laptop
- Press space to start saving video and press space again to stop saving, and press ESC to close
- Check ATAM/data/movie.avi

## 2. CameraCalibration
- Print opencv/sources/doc/pattern.png on a A4 paper (acircles_pattern.png if uncheck USE\_CHESSBOARD)
 - Need to change size in CCalibration::CCalibration() or call CCalibration::SetSize() if use different size
- Open ATAM/build/proj/CameraCalibration/CameraCalibration.sln, compile and run it
- Press space to capture images from different viewpoints several times, and press ESC to close
- Check ATAM/data/camera.xml

## 3. ATAM
- Open ATAM/build/proj/ATAM/ATAM.sln and modify
 - line 95 in cvsba.cpp as  
   int nconstframes = imagePoints.size() > 2 ? 2 : 0; //mcon
 - line 233 in cvsba.cpp as  
   //throw cv::Exception(8888, "Error occured during Sba optimization", "Sba::run", \_\_FILE\_\_, \_\_LINE\_\_);  
   return -1;
 - line 636 in sba\_levmar.c as  
   //return SBA_ERROR;
- Compile and run it
    - Ignore warning in cvsba
- Press space to start, translate a camera and press space (initialization)		
- Press space to capture a calibration board from different viewpoints at least twice (registration)

### Other operations of ATAM (on keyboard or window)
 - Press n if new points not appear (manually add a new keyframe)
 - Press n if relocalization not work (manually change view for relocalization)
 - Press c to visualize next challenge point
 - Press r to reset
 - Press ESC to close

# Tips for ATAM

- Notes
 - If initialization failed, translate more and press space again
 - If tracking failed, move the camera to fit edges
 - Once run and close ATAM, can modify parameters in ATAM/data/params.xml

- Dot colors
 - Blue: new points
 - White: mapped points
 - Red(small): projected mapped points
		
- Parameters
 - MAXPTS: number of keypoints by ORB
 - LEVEL: number of pyramid levels for ORB
 - DESCDIST: maximum distance between two ORB descriptors for judging as a correct match (between 0 and 256)
 - BASEANGLE: criterion for adding a new keyframe (degree)
 - BAKEYFRAMES: number of keyframes for local bundle adjustment (more than 2)
 - PROJERR: maximum reprojection error for inlier (pixel)
 - MINPTS: minimum number of points for judging as stable tracking (not good if less than 20)
 - PATCHSIZE: size of patch for KLT (pixel)
 - MATCHKEYFRAME: minimum inlier ratio (inliers/matches) for keypoint matching in matching with keyframes (between 0 and 1)
 - GOODINIT: minimum inlier ratio for initial triangulation (between 0 and 1)
 - RELOCALHIST: number of last keyframes used for relocalization (more than 1)
 - USEVIDEO: 1 if use video, otherwise 0 (use video saved with CameraCapture)
 - VIDEONAME: file name for video
 - CAMERANAME: file name for camera parameters

- ATAM/data/challenge.txt for challenging points is  
4  
1	0	1	10  
7	5	12	1  
11	53	34	5  
19	11	21	9  
as  
Number of challenge points  
ID1 X1 Y1 Z1  
ID2 X2 Y2 Z2  
...
