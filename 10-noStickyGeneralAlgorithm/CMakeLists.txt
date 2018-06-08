# create an executable
ADD_EXECUTABLE (10-noStickyGeneralAlgorithm
	${COMMON_SOURCE}
	main.cpp)

# and link the library against the executable
TARGET_LINK_LIBRARIES (10-noStickyGeneralAlgorithm ${COMMON_LIBRARIES})

# export resources such as model files.
# NOTE: this requires an install build
SET(RESOURCE_DIR ${BINARY_DIR}/resources)
FILE(MAKE_DIRECTORY ${RESOURCE_DIR})
SET(APP_RESOURCE_DIR ${RESOURCE_DIR}/10-noStickyGeneralAlgorithm)
FILE(MAKE_DIRECTORY ${APP_RESOURCE_DIR})
FILE(COPY drill_bit.obj DESTINATION ${APP_RESOURCE_DIR})