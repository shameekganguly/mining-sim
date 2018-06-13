# create an executable
ADD_EXECUTABLE (16-ERotation 
	${COMMON_SOURCE}
	main.cpp)
	#main_test_Eigen.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (16-ERotation ${COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/16-ERotation)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})




