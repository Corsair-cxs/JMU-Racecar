cmake_minimum_required(VERSION 2.8.3)
project(jmu_racecar_navigation)

find_package(catkin_simple REQUIRED)

#uncomment next line to use OpenCV library
#find_package(OpenCV REQUIRED)

#uncomment the next line to use the point-cloud library
#find_package(PCL 1.7 REQUIRED)

#uncomment the following 4 lines to use the Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(robotnik_msgs REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# use carefully;  can interfere with point-cloud library
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries: uncomment the following and edit arguments to create a new library
cs_add_library(odom_tf src/OdomTf.cpp)   
cs_add_library(pf_odom_tf src/pf_OdomTf.cpp)   

# Executables: uncomment the following and edit arguments to compile new nodes
# may add more of these lines for more nodes from the same package
cs_add_executable(jmu_odom_tf src/odom_tf_main.cpp)
cs_add_executable(jmu_pf_odom_tf src/pf_odom_tf_main.cpp)

#the following is required, if desire to link a node in this package with a library created in this same package
# edit the arguments to reference the named node and named library within this package
target_link_libraries(jmu_odom_tf odom_tf)
target_link_libraries(jmu_pf_odom_tf pf_odom_tf)

cs_install()
cs_export()
    