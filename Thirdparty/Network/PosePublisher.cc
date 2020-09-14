#include "PosePublisher.h"

PosePublisher::PosePublisher(string ip, int port) {
    cout << "Initing Pose Publisher" << endl;
    int recv_len = -1;
    int ret = -1;

    struct sockaddr_in t_sockaddr;
    memset(&t_sockaddr, 0, sizeof(t_sockaddr));
    t_sockaddr.sin_family = AF_INET;
    t_sockaddr.sin_addr.s_addr = inet_addr(ip.c_str());
    t_sockaddr.sin_port = htons(port);

    int listen_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
        fprintf(stderr, "socket error %s errno: %d\n", strerror(errno), errno);
    }
    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    ret = bind(listen_fd,(struct sockaddr *) &t_sockaddr,sizeof(t_sockaddr));
    if (ret < 0) {
        fprintf(stderr, "bind socket error %s errno: %d\n", strerror(errno), errno);
    }

    ret = listen(listen_fd, 1024);
    if (ret < 0) {
        fprintf(stderr, "listen error %s errno: %d\n", strerror(errno), errno);
    }
    cout << "listening on " << ip << ":" << port << endl;
    struct sockaddr_in conn_addr;
    socklen_t conn_size = sizeof(conn_addr);
    conn_fd = accept(listen_fd,(struct sockaddr *) &conn_addr, &conn_size);
    cout << "accept connect from " << inet_ntoa(conn_addr.sin_addr) << endl;
    if(conn_fd < 0) {
        fprintf(stderr, "accpet socket error: %s errno :%d\n", strerror(errno), errno);
    }
    cout << "Init Pose Publisher Suc" << endl;
}

PosePublisher::~PosePublisher() {
    cout << "Release Pose Publisher" << endl;
    close(conn_fd);
    close(listen_fd);
}

int PosePublisher::sendTcw(int status, const cv::Mat &Tcw) {
    int len = 0;
    len += sendInt(MsgType::Camera);
    len += sendInt(status);
    if (status == 2) {
        len += sendMat4x4(Tcw);
    }
    return len;
}

int PosePublisher::sendMarkers(const vector<MarkerInfo> &markers) {
    int len = 0;
    len += sendInt(MsgType::Markers);
    len += sendInt(markers.size());
    for (const auto &marker : markers) {
        len += sendMarker(marker);
    }
    return len;
}

int PosePublisher::sendMarker(const MarkerInfo &marker) {
    int len = 0;
    len += sendInt(marker.id);
    len += sendMat4x4(marker.Tmw);
    return len;
}

int PosePublisher::sendIntrinsic(float fx, float fy, float cx, float cy) {
    int len = 0;
    intrinsic_buffer.val_array[0] = fx;
    intrinsic_buffer.val_array[1] = fy;
    intrinsic_buffer.val_array[2] = cx;
    intrinsic_buffer.val_array[3] = cy;
    len += sendInt(MsgType::Intrinsic);
    len += send(conn_fd, intrinsic_buffer.buffer, sizeof(intrinsic_buffer), MSG_WAITALL);
    return len;
}

int PosePublisher::sendInt(int val) {
    int_buffer.val = val;
    int len = send(conn_fd, int_buffer.buffer, sizeof(int), MSG_WAITALL);
    return len;
}
int PosePublisher::sendMat4x4(const cv::Mat &mat) {
    assert(mat.cols == 4 && mat.rows == 4 && mat.type() == CV_32F);
    cv::Mat Rcw(3, 3, CV_32F);
    cv::Mat tcw(3, 1, CV_32F);

    Rcw = mat.rowRange(0, 3).colRange(0, 3);
    tcw = mat.rowRange(0, 3).col(3);

    mat4x4_buffer.val_array[0] = Rcw.at<float>(0, 0);
    mat4x4_buffer.val_array[1] = Rcw.at<float>(1, 0);
    mat4x4_buffer.val_array[2] = Rcw.at<float>(2, 0);
    mat4x4_buffer.val_array[3] = 0.0;

    mat4x4_buffer.val_array[4] = Rcw.at<float>(0, 1);
    mat4x4_buffer.val_array[5] = Rcw.at<float>(1, 1);
    mat4x4_buffer.val_array[6] = Rcw.at<float>(2, 1);
    mat4x4_buffer.val_array[7] = 0.0;

    mat4x4_buffer.val_array[8] = Rcw.at<float>(0, 2);
    mat4x4_buffer.val_array[9] = Rcw.at<float>(1, 2);
    mat4x4_buffer.val_array[10] = Rcw.at<float>(2, 2);
    mat4x4_buffer.val_array[11] = 0.0;

    mat4x4_buffer.val_array[12] = tcw.at<float>(0);
    mat4x4_buffer.val_array[13] = tcw.at<float>(1);
    mat4x4_buffer.val_array[14] = tcw.at<float>(2);
    mat4x4_buffer.val_array[15] = 1.0;
    // for (int i = 0;i < 16;++i) {
    //     mat4x4_buffer.val_array[i] = Tcw.at<float>(i % 4, i / 4);
    // }
    int len = send(conn_fd, mat4x4_buffer.buffer, sizeof(mat4x4_buffer), MSG_WAITALL);
    return len;
}

