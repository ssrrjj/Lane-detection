cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(planar_segmentation)

find_package(PCL  REQUIRED)
find_package( OpenCV REQUIRED )
# cmake -DCMAKE_BUILD_TYPE=Debug ../

# The options need to be the same as Open3D's default
# If Open3D is configured and built with custom options, you'll also need to
# specify the same custom options.
option(STATIC_WINDOWS_RUNTIME "Use static (MT/MTd) Windows runtime" ON)
if(STATIC_WINDOWS_RUNTIME)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

# Find installed Open3D, which exports Open3D::Open3D
find_package(Open3D REQUIRED)

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
target_link_libraries( lanedetection ${PCL_LIBRARIES} ${OpenCV_LIBS} Open3D::Open3D)

################# Compile the applications ###########################

# Application lanedet
add_executable (lanedet examples/lanedet.cpp)
target_include_directories(lanedet PUBLIC include/)
target_link_libraries (lanedet lanedetection ${PCL_LIBRARIES} ${OpenCV_LIBS} Open3D::Open3D)

# pointcloudview
add_executable (ptcviewer examples/ptcviewer.cpp)
target_include_directories(ptcviewer PUBLIC include/)
target_link_libraries (ptcviewer lanedetection ${PCL_LIBRARIES} ${OpenCV_LIBS} )

# lane from plane
add_executable (LFP examples/lane_from_plane.cpp)
target_include_directories(LFP PUBLIC include/)
target_link_libraries (LFP lanedetection ${PCL_LIBRARIES} ${OpenCV_LIBS} Open3D::Open3D)

################# Compile the tests ###################################

# test plane model fitting
add_executable (test_pcfitplane test/test_pcfitplane.cpp)
target_include_directories(test_pcfitplane PUBLIC include/)
target_link_libraries (test_pcfitplane lanedetection ${PCL_LIBRARIES} Open3D::Open3D)

#if(WIN32)
get_target_property(open3d_type Open3D::Open3D TYPE)
if(open3d_type STREQUAL "SHARED_LIBRARY")
    message(STATUS "Copying Open3D.dll to ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>")
    add_custom_command(TARGET Draw POST_BUILD
                        COMMAND ${CMAKE_COMMAND} -E copy
                                ${CMAKE_INSTALL_PREFIX}/bin/Open3D.dll
                                ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>)
endif()
#endif()
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



