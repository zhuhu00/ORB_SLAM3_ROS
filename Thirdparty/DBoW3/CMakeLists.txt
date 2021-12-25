# ----------------------------------------------------------------------------
#   Basic Configuration
# ----------------------------------------------------------------------------
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

set(CMAKE_CXX_STANDARD 11)

PROJECT(DBoW3)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set(PROJECT_VERSION "0.0.1")
string(REGEX MATCHALL "[0-9]" PROJECT_VERSION_PARTS "${PROJECT_VERSION}")


#------------------------------------------------------
# Build type
#------------------------------------------------------

IF(NOT CMAKE_BUILD_TYPE )
   SET( CMAKE_BUILD_TYPE "Release" )
ENDIF()

#------------------------------------------------------
# Lib Names and Dirs
#------------------------------------------------------

set(HDRS_DBOW3
  DBoW3/BowVector.h
  DBoW3/FORB.h 
  DBoW3/FClass.h       
  DBoW3/FeatureVector.h
  DBoW3/ScoringObject.h   
  DBoW3/TemplatedVocabulary.h
  DBoW3/Database.h
  DBoW3/DBoW3.h
  DBoW3/DescManip.h
  DBoW3/export.h
  DBoW3/QueryResults.h
  DBoW3/quicklz.h
  DBoW3/Vocabulary.h
  )

set(SRCS_DBOW3
  DBoW3/BowVector.cpp
  DBoW3/FORB.cpp      
  DBoW3/FeatureVector.cpp
  DBoW3/ScoringObject.cpp
  DBoW3/Database.cpp
  DBoW3/DescManip.cpp
  DBoW3/QueryResults.cpp
  DBoW3/quicklz.c
  DBoW3/Vocabulary.cpp
  )

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)


# ----------------------------------------------------------------------------
#   Find Dependencies
# ----------------------------------------------------------------------------
find_package(OpenCV 3 REQUIRED)
IF(USE_CONTRIB)
add_definitions(-DUSE_CONTRIB)
ENDIF()
if(NOT OpenCV_VERSION VERSION_LESS "3.0")
    ADD_DEFINITIONS(-DOPENCV_VERSION_3)
    SET(OPENCV_VERSION_3 ON)
ELSE()
    SET(OPENCV_VERSION_3 OFF)
ENDIF()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})

add_library(DBoW3 SHARED ${SRCS_DBOW3} ${SRCS_DUTILS})
target_link_libraries(DBoW3 ${OpenCV_LIBS})
# SET(REQUIRED_LIBRARIES ${REQUIRED_LIBRARIES} ${OpenCV_LIBS})

