# create an executable
ADD_EXECUTABLE (05-voxelCollision 
	${COMMON_SOURCE}
	main.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (05-voxelCollision ${COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/05-voxelCollision)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})




