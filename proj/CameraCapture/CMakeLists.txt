##############################
# CameraCapture
##############################
set( PROJ_NAME CameraCapture )
project( ${PROJ_NAME} )

# internal
file(GLOB PROJ_SOURCE *.cpp)
file(GLOB PROJ_INCLUDE *.h)

# shared
include_directories(${PROJ_SHARED_CAMERA_DIR})

# Target
add_executable(
	${PROJ_NAME}
    ${PROJ_SOURCE}
    ${PROJ_INCLUDE}
    ${PROJ_SHARED_DATADIR_INCLUDE}
    ${PROJ_SHARED_CAMERA_SOURCE}
    ${PROJ_SHARED_CAMERA_INCLUDE}
)

target_link_libraries(
	${PROJ_NAME}
    ${OpenCV_LIBS}
	${CAM_LIBS}
)
