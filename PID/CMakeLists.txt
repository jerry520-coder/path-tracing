cmake_minimum_required(VERSION 3.5)
project(mower VERSION 1.0)

# 添加编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall")

set(CMAKE_BUILD_TYPE Debug)

find_package(OpenCV REQUIRED COMPONENTS opencv_core opencv_highgui opencv_imgproc opencv_imgcodecs opencv_videoio opencv_video)

include_directories(${PROJECT_SOURCE_DIR}/include
                            "/usr/include/eigen3"
                            ${OpenCV_INCLUDE_DIRS})

set(SRC_LIST 
       ${PROJECT_SOURCE_DIR}/src/sweep/work_area.cpp 
       ${PROJECT_SOURCE_DIR}/src/sweep/garden.cpp 
       ${PROJECT_SOURCE_DIR}/src/sweep/work_area_api.cpp
       ${PROJECT_SOURCE_DIR}/src/common/public_type.cpp 
       ${PROJECT_SOURCE_DIR}/src/search/Astar.cpp 
       )

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib) #指定库的输出路径


add_library(plan SHARED ${SRC_LIST})
target_link_libraries(plan PUBLIC ${OpenCV_LIBS}) 
#这行代码指定将OpenCV_LIBS库链接到plan目标。PUBLIC关键字表示将链接的库作为plan目标的公共接口之一。这意味着，当其他目标链接到plan时，也将自动链接到OpenCV_LIBS库。

add_executable(mower_plan  ${PROJECT_SOURCE_DIR}/main/main.cpp)
target_link_libraries(mower_plan  plan)

