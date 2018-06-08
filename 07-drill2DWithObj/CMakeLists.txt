# create an executable
ADD_EXECUTABLE (07-drill2DWithObj 
	${COMMON_SOURCE}
	main.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (07-drill2DWithObj ${COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/07-drill2DWithObj)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY drill_bit.obj DESTINATION ${APP_RESOURCE_DIR})




