#Modify only if you know what you are doing!
cmake_minimum_required(VERSION 3.10)
project(lanelet2)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-backtrace-limit=0")
set (COTIRE_MINIMUM_NUMBER_OF_TARGET_SOURCES 1)
set (COTIRE_MAXIMUM_NUMBER_OF_UNITY_INCLUDES "-j")
SET(CMAKE_CXX_FLAGS -pthread)
include(toolchains.cmake)

###################
## find packages ##
###################
find_package(Eigen3 REQUIRED)
find_package(Boost 1.40 COMPONENTS serialization program_options filesystem regex REQUIRED)

find_package(PkgConfig)
find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
  PATH_SUFFIXES GeographicLib
  )
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})
find_library(GeographicLib_LIBRARIES
  NAMES Geographic
  )

find_library(PUGIXML_LIBRARIES
    NAMES pugixml
)
find_path(PUGIXML_INCLUDE_DIRS
    NAMES pugixml.hpp
    PATH_SUFFIXES pugixml
)

############################
## read source code files ##
############################

file(GLOB PROJECT_SOURCE_FILES_INC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "src/*.h" "src/*.hpp")
file(GLOB_RECURSE PROJECT_HEADER_FILES_INC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "include/*.h" "include/*.hpp")
file(GLOB PROJECT_SOURCE_FILES_SRC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "src/*.cpp")

SUBDIRLIST(subDirectories ${CMAKE_CURRENT_SOURCE_DIR})
foreach(SUB_DIRECTORY ${subDirectories})
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${SUB_DIRECTORY}/include)
    file(GLOB_RECURSE INCLUDE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${SUB_DIRECTORY}/include/*.h
				    ${CMAKE_CURRENT_SOURCE_DIR}/${SUB_DIRECTORY}/include/*.hpp
				    ${CMAKE_CURRENT_SOURCE_DIR}/${SUB_DIRECTORY}/src/*.h
				    ${CMAKE_CURRENT_SOURCE_DIR}/${SUB_DIRECTORY}/src/*.hpp)
    list(APPEND PROJECT_HEADER_FILES_INC ${INCLUDE_FILES})
    file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/${SUB_DIRECTORY}/src/*.cpp)
    list(APPEND PROJECT_SOURCE_FILES_SRC ${SOURCE_FILES})
endforeach()

###########
## Build ##
###########
# Add include and library directories
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${PUGIXML_INCLUDE_DIRS}
    ${}
    )
add_library(lanelet2_lib SHARED ${PROJECT_SOURCE_FILES_SRC} ${PROJECT_HEADER_FILES_INC})
file(GLOB SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/simu/*.cpp)
file(GLOB_RECURSE ALGLIB_FILES "alglib/*")
add_executable(simu ${SRC_FILES} ${ALGLIB_FILES}
        simu/car.cpp simu/car.h simu/main.cpp


        PlannerLib/Array.hh PlannerLib/Array.cc PlannerLib/AstarSolver.h PlannerLib/AstarSolver.cpp
        PlannerLib/MinHeap.h PlannerLib/MinHeap.cpp PlannerLib/QuadProg++.hh PlannerLib/QuadProg++.cc
        PlannerLib/QPSetup.h PlannerLib/QPSetup.cpp  PlannerLib/smoother.h  PlannerLib/smoother.cpp

        PlannerLib/OptimalTime.hpp PlannerLib/OptimalTime.cpp
        PlannerLib/polynomial.h PlannerLib/polynomial.cc
        PlannerLib/find_polynomial_roots_jenkins_traub.h PlannerLib/find_polynomial_roots_jenkins_traub.cc
        PlannerLib/IDM.hpp PlannerLib/IDM.cpp
        PlannerLib/MOBIL.hpp PlannerLib/MOBIL.cpp
        simu/RefCreater.cpp simu/RefCreater.h
        simu/txtRef.cpp simu/txtRef.h)
target_link_libraries(simu lanelet2_lib ${Boost_LIBRARIES} ${Eigen3_LIBRARIES} ${GeographicLib_LIBRARIES} ${PUGIXML_LIBRARIES})
