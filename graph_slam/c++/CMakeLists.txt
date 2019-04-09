
project(graph_interface)

cmake_minimum_required(VERSION 2.8.3)

# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -std=c++11 -w -g -O3 -fpermissive -mno-avx -mno-avx2")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -w -g -O3 -fpermissive")
SET(CMAKE_BUILD_TYPE "RelWithDebInfo")

if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

# rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen3 QUIET)
include_directories(${EIGEN3_INCLUDE_DIR} "/usr/include/eigen3")
# include_directories("/usr/include/eigen3")
include_directories("${PROJECT_SOURCE_DIR}/include")

################### G2O ######################
# set(G2O_ROOT /home/david/work/3rdlibs/g2o/install)
set(G2O_ROOT /home/davidz/work/3rdlib/g2o/install/usr/local)
include_directories(${G2O_ROOT}/include /usr/include/suitesparse) 
link_directories(${G2O_ROOT}/lib /usr/lib/i386-linux-gnu)
set(G2O_LIBS g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_core g2o_cli g2o_csparse_extension cxsparse)

################### GTSAM 4.0 ######################
# set(GTSAM_ROOT /home/david/work/3rdlibs/gtsam-4.0/build)
set(GTSAM_ROOT /home/davidz/work/3rdlib/gtsam/build)
 find_package(GTSAM REQUIRED 
   PATHS ${GTSAM_ROOT}) 
include_directories(${GTSAM_INCLUDE_DIR} ${GTSAM_ROOT}) # ${GTSAM_ROOT}/../gtsam/3rdparty/Eigen) # /usr/include/eigen3
link_directories(${GTSAM_ROOT}/gtsam )

#################### compile g2o interface ######################
add_library(g2o_graph SHARED g2o_graph.cpp g2o_parameter.cpp color.cpp matching_result.cpp camera_node.cpp)
target_link_libraries(g2o_graph ${G2O_LIBS})

################### compile gtsam interface ####################
add_library(gtsam_graph SHARED gtsam_graph.cpp gt_parameter.cpp color.cpp matching_result.cpp camera_node.cpp)
target_link_libraries(gtsam_graph gtsam)

################# example #######################
add_executable(test_gtsam test_gtsam.cpp)
target_link_libraries(test_gtsam gtsam_graph)

add_executable(test_g2o test_g2o.cpp)
target_link_libraries(test_g2o g2o_graph)



