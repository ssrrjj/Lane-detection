cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(planar_segmentation)

find_package(PCL 1.8 REQUIRED)
find_package( OpenCV REQUIRED )
# cmake -DCMAKE_BUILD_TYPE=Debug ../

# cmake -DUSE_DEBUG=ON ..
option(USE_DEBUG "Enter debug mode" OFF)
if (USE_DEBUG)
  add_definitions(-DDEBUG)
endif()
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB CPP_SOURCES src/*.cpp)

add_library( lanedetection ${CPP_SOURCES})
#target_compile_options(lanedetection PRIVATE -O3)
target_include_directories(lanedetection PUBLIC include/ src/)
target_link_libraries( lanedetection ${PCL_LIBRARIES} ${OpenCV_LIBS} )

################# Compile the applications ###########################

# Application lanedet
add_executable (lanedet examples/lanedet.cpp)
target_include_directories(lanedet PUBLIC include/)
target_link_libraries (lanedet lanedetection ${PCL_LIBRARIES} ${OpenCV_LIBS} )

# pointcloudview
add_executable (ptcviewer examples/ptcviewer.cpp)
target_include_directories(ptcviewer PUBLIC include/)
target_link_libraries (ptcviewer lanedetection ${PCL_LIBRARIES} ${OpenCV_LIBS} )

################# Compile the tests ###################################

# test plane model fitting
#add_executable (test_pcfitplane test/test_pcfitplane.cpp)
#target_include_directories(test_pcfitplane PUBLIC include/)
#target_link_libraries (test_pcfitplane lanedetection ${PCL_LIBRARIES})

# test DBSCAN
#add_executable (test_dbscan test/test_dbscan.cpp)
#target_include_directories(test_dbscan PUBLIC include/)
#target_link_libraries (test_dbscan lanedetection ${PCL_LIBRARIES})

# test lane detection
#add_executable (test_lanedetection test/test_lanedetection.cpp)
#target_include_directories(test_lanedetection PUBLIC include/)
#target_link_libraries (test_lanedetection lanedetection ${PCL_LIBRARIES})

# test utils
#add_executable (test_utils test/test_utils.cpp)
#target_include_directories(test_utils PUBLIC include/)
#target_link_libraries (test_utils lanedetection ${PCL_LIBRARIES})

# test readpar
#add_executable (test_readpar test/test_readpar.cpp)
#target_include_directories(test_readpar PUBLIC include/)
#target_link_libraries (test_readpar lanedetection ${PCL_LIBRARIES})



