##############################
# CameraCalibration
##############################
set( PROJ_NAME CameraCalibration )
project( ${PROJ_NAME} )

# Add all files
file(GLOB PROJ_SOURCE *.cpp)
file(GLOB PROJ_INCLUDE *.h)

include_directories(${PROJ_SHARED_CAMERA_DIR})
include_directories(${PROJ_SHARED_DATADIR_DIR})
include_directories(${PROJ_SHARED_CALIBRATION_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})

if( USE_CHESSBOARD )
	add_definitions( -DCHESSBOARD )
endif()

# Target
add_executable(${PROJ_NAME}
    ${PROJ_SOURCE}
    ${PROJ_INCLUDE}
    ${PROJ_SHARED_CAMERA_SOURCE}
    ${PROJ_SHARED_CAMERA_INCLUDE}
    ${PROJ_SHARED_DATADIR_INCLUDE}
	${PROJ_SHARED_CALIBRATION_SOURCE}
    ${PROJ_SHARED_CALIBRATION_INCLUDE}
)
target_link_libraries(${PROJ_NAME}
    ${OpenCV_LIBS}
	${CAM_LIB}
)
