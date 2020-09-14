#include <stdio.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/epoll.h>
#include <string.h>
#include <netinet/in.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <iostream>
#include <vector>
#include <chrono>
#include <opencv2/opencv.hpp>

using namespace std;

enum MsgType {
    Camera = 1,
    Markers,
    Plane,
    Intrinsic
};

template<size_t n, typename T>
union ARRAY_BUFFER {
    T val_array[n];
    char buffer[sizeof(T) * n];
};

union INT_BUFFER {
    int val;
    char buffer[sizeof(int)];
};

#pragma pack(1)
template<size_t n, typename T>
struct DATA_BUFFER {
    INT_BUFFER type;
    ARRAY_BUFFER<n, T> array_buffer;
};

struct MarkerInfo {
    int id;
    cv::Mat Tmw;
};

class PosePublisher {
public:
    PosePublisher(string server_ip, int port);

    ~PosePublisher();

    int sendInt(int val);

    int sendMat4x4(const cv::Mat &mat);

    int sendTcw(int status, const cv::Mat &Tcw);

    int sendMarkers(const std::vector<MarkerInfo> &markers);

    int sendMarker(const MarkerInfo &marker);

    int sendIntrinsic(float fx, float fy, float cx, float cy);
private:
    int conn_fd;
    int listen_fd;
    INT_BUFFER int_buffer;
    ARRAY_BUFFER<16, float> mat4x4_buffer;
    ARRAY_BUFFER<4, float> intrinsic_buffer;
    DATA_BUFFER<16, float> marker_buffer;
};