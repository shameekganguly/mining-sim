# create an executable
ADD_EXECUTABLE (09-wholeRobot
	${COMMON_SOURCE}
	main.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (09-wholeRobot ${COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/09-wholeRobot)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY drill_bit.obj DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY brake_cam2.obj DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY brake_cam.obj DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY roller_cone_drill_bit_02.obj DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY forebody.obj DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY rearbody.obj DESTINATION ${APP_RESOURCE_DIR})
FILE(COPY drill1.obj DESTINATION ${APP_RESOURCE_DIR})