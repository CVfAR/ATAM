######################################
## Compiler setting
######################################

cmake_minimum_required( VERSION 2.8 )

set( CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory" )

set( PROJ_NAME PROJ_ATAM )
project( ${PROJ_NAME} )

# Compiler setting
include( CheckCXXCompilerFlag )

if( UNIX )
	SET( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x" )
endif( )

######################################
## External libraries
######################################

# OpenCV
set( BUILD_SHARED_LIBS "ON" )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

option( USE_BA "Use bundle adjustment" ON )

if( USE_BA )
	# for cvsba
	find_library( LAPACK_LIB NAMES lapack )
	find_library( BLAS_LIB NAMES blas )
	find_library( F2C_LIB NAMES f2c )

	set( CVSBA_LIBS ${LAPACK_LIB} ${F2C_LIB} ${BLAS_LIB} )
	message( STATUS "LAPACK_LIB = ${LAPACK_LIB}" )
	message( STATUS "BLAS_LIB = ${BLAS_LIB}" )
	message( STATUS "F2C_LIB = ${F2C_LIB}" )
endif( )

# point grey camera
option( USE_PGR "Use point grey camera" OFF )
if( USE_PGR )
	find_path( PGR_INCLUDE_DIR NAMES FlyCapture2.h )
	find_library( PGR_LIB NAMES flycapture )

	message( STATUS "FlyCapture2.h is in ${PGR_INCLUDE_DIR}" )
	include_directories( ${PGR_INCLUDE_DIR} )
	message( STATUS "PGR_LIB = ${PGR_LIB}" )
	set( CAM_LIB ${PGR_LIB} )
    add_definitions( -DUSEPGR )
endif( )

######################################
## Setting
######################################

# chessboard or dot pattern
option( USE_CHESSBOARD "Use chessboard" ON )

if( USE_BA )
	option( USE_MULTITHREAD "Use multi thread" ON )
endif( )

# show log or not
option( SHOW_LOG "Show logs" ON )
if( SHOW_LOG )
	add_definitions( -DSHOWLOG )
endif( )

######################################
## Sources
######################################

set( PROJ_SHARED_DIR ${CMAKE_SOURCE_DIR}/src )

# make DataDirectory.h from DataDirectory.h.in
set( PROJ_SHARED_DATADIR_DIR ${PROJ_SHARED_DIR}/datadir )
set( PROJ_SHARED_DATADIR_INCLUDE ${PROJ_SHARED_DATADIR_DIR}/DirectoryConfig.h )
configure_file ( "${CMAKE_SOURCE_DIR}/DataDirectory.h.in" ${PROJ_SHARED_DATADIR_INCLUDE} )

# camera
set( PROJ_SHARED_CAMERA_DIR ${PROJ_SHARED_DIR}/camera )
file( GLOB PROJ_SHARED_CAMERA_SOURCE ${PROJ_SHARED_CAMERA_DIR}/*.cpp )
file( GLOB PROJ_SHARED_CAMERA_INCLUDE ${PROJ_SHARED_CAMERA_DIR}/*.h )

# timer
set( PROJ_SHARED_TIMER_DIR ${PROJ_SHARED_DIR}/timer )
file( GLOB PROJ_SHARED_TIMER_SOURCE ${PROJ_SHARED_TIMER_DIR}/*.cpp )
file( GLOB PROJ_SHARED_TIMER_INCLUDE ${PROJ_SHARED_TIMER_DIR}/*.h )

# atam
set( PROJ_SHARED_ATAM_DIR ${PROJ_SHARED_DIR}/atam )
file( GLOB PROJ_SHARED_ATAM_SOURCE ${PROJ_SHARED_ATAM_DIR}/*.cpp )
file( GLOB PROJ_SHARED_ATAM_INCLUDE ${PROJ_SHARED_ATAM_DIR}/*.h )

# point detector
set( PROJ_SHARED_DETECTOR_DIR ${PROJ_SHARED_DIR}/detector )
file( GLOB PROJ_SHARED_DETECTOR_SOURCE ${PROJ_SHARED_DETECTOR_DIR}/*.cpp )
file( GLOB PROJ_SHARED_DETECTOR_INCLUDE ${PROJ_SHARED_DETECTOR_DIR}/*.h )

# bundle adjustment
if( USE_BA )
	set( PROJ_SHARED_CVSBA_DIR ${PROJ_SHARED_DIR}/cvsba )
	file( GLOB PROJ_SHARED_CVSBA_SOURCE ${PROJ_SHARED_CVSBA_DIR}/*.cpp ${PROJ_SHARED_CVSBA_DIR}/*.c)
	file( GLOB PROJ_SHARED_CVSBA_INCLUDE ${PROJ_SHARED_CVSBA_DIR}/*.h )
endif()

# calibration
set( PROJ_SHARED_CALIBRATION_DIR ${PROJ_SHARED_DIR}/calibration )
file( GLOB PROJ_SHARED_CALIBRATION_SOURCE ${PROJ_SHARED_CALIBRATION_DIR}/*.cpp)
file( GLOB PROJ_SHARED_CALIBRATION_INCLUDE ${PROJ_SHARED_CALIBRATION_DIR}/*.h )

######################################
## Project
######################################

set( PROJ_SUB_DIR ${CMAKE_SOURCE_DIR}/proj )
add_subdirectory( ${PROJ_SUB_DIR} )
