#include <iostream>
#include <mutex>
#include <thread>
#include <opencv2/opencv.hpp>

extern "C" {
    #include <stdio.h>
    #include <assert.h>
    #include <libavcodec/avcodec.h>
    #include <libavformat/avformat.h>
    #include <libswscale/swscale.h>
}
using namespace std;
using namespace cv;

class FrameReceiver {
public:
    FrameReceiver(string url);

    void startReceive();

    void stopReceive();

    Mat getCurFrame();

    bool isStop();

    ~FrameReceiver();
private:
    void receive();

    void copyDate(AVFrame *picture,int width,int height);

    AVFormatContext *pctx;
    AVCodecContext *pcodec_ctx_orig, *pcodec_ctx;
    AVCodec *pcodec;
    AVFrame *pframe;
    AVFrame *pframe_rgb;
    uint8_t *buffer;
    AVPacket pkt;
    bool stop;
    struct SwsContext *sws_ctx;
    int video_stream;

    Mat cur_frame;
    mutex m;
    thread *receive_thread;
};