cmake_minimum_required(VERSION 2.8)
PROJECT(Network)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories("/usr/local/ffmpeg/include")
include_directories("/usr/include")
link_directories("/usr/local/ffmpeg/lib")
link_directories("/usr/lib/x86_64-linux-gnu")

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED FrameReceiver.cc PosePublisher.cc)
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    libavutil.so libavcodec.so libavformat.so libavdevice.so libavfilter.so libswscale.so libpostproc.so
    pthread
)