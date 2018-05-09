project(base VERSION 1.0.0)

include_directories(
    ./include 
    ../lib/include
)
add_definitions(-std=c++11 -O3 -Wall)

subdirs(
    ../modules/videoreader
)
add_executable(app
    src/ObjectUtils.cpp
    src/Dtracker.cpp
    src/connectCMS.cpp
    src/IOData.cpp
    src/ObjectDetector.cpp
    src/RedLightDetector.cpp
    src/KCFtrackmanager.cpp
    src/kcf.cpp
    src/gradientMex.cpp
    src/cn_data.cpp
    src/pugixml.cpp
    Main.cpp
)

add_subdirectory(../dlib dlib_build)

target_link_libraries(app
    ObjectDetector
    ${OpenCV_LIBS}
    curl
    json
    jsoncpp
    dlib::dlib
    vwscapture
    videoreader
)
