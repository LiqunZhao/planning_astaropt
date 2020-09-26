# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 2.8.12)
project(lanelet2_core)
set (COTIRE_MINIMUM_NUMBER_OF_TARGET_SOURCES 1)
set (COTIRE_MAXIMUM_NUMBER_OF_UNITY_INCLUDES "-j")

###################
## find packages ##
###################
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-backtrace-limit=0")

############################
## read source code files ##
############################
file(GLOB_RECURSE PROJECT_HEADER_FILES_INC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "include/*.h" "include/*.hpp")
file(GLOB PROJECT_SOURCE_FILES_INC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "src/*.h" "src/*.hpp")
file(GLOB PROJECT_SOURCE_FILES_SRC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "src/*.cpp")

if (PROJECT_SOURCE_FILES_SRC)
    set(LIBRARY_NAME ${PROJECT_NAME})
endif()

###########
## Build ##
###########
# Add include and library directories
include_directories(
    include/${PROJECT_NAME}
    ${Boost_INCLUDE_DIRS}
    )

target_link_libraries(${PROJECT_NAME} ${PROJECT_HEADER_FILES_INC} ${PROJECT_SOURCE_FILES_INC} ${PROJECT_SOURCE_FILES_SRC} ${Boost_LIBRARIES} ${Eigen3_LIBRARIES})

