#include "FrameReceiver.h"

FrameReceiver::FrameReceiver(string url) {
    cout << "Initing Frame Receiver" << endl;
    stop = false;
    av_register_all();

    pctx = NULL;
    AVDictionary *opt = NULL;
    // av_dict_set(&opt, "rtsp_transport", "tcp", 0);
    av_dict_set(&opt, "rw_timeout", "1000000", 0);
    if (avformat_open_input(&pctx, url.c_str(), NULL, &opt)!=0) {
        cout << "fail open input : " << url << endl;
        return;
    }
    av_dump_format(pctx, 0, url.c_str(), 0);
    assert(avformat_find_stream_info(pctx, NULL)>=0);
    video_stream = -1;
    cout << "nb stream : " << pctx->nb_streams << endl;
    for (int i=0; i<pctx->nb_streams; i++) {
        if (pctx->streams[i]->codec->codec_type==AVMEDIA_TYPE_VIDEO) {
            video_stream =i;
            break;
        }
    }
    if (-1==video_stream) {
        cout << "no video stream detected\n" << endl;
        return;
    }

    pcodec_ctx = pctx->streams[video_stream]->codec;
    
    AVCodec *pcodec = NULL;
    pcodec = avcodec_find_decoder(pcodec_ctx->codec_id);
    cout << "codec name : " << pcodec->name << endl;
    if (NULL == pcodec) {
        cout << "unsupported codec.\n" << endl;
        return;
    }

    pcodec_ctx_orig = avcodec_alloc_context3(pcodec);
    if (avcodec_copy_context(pcodec_ctx_orig, pcodec_ctx) != 0) {
        cout << "couldn't copy codec context\n" << endl;
        return;
    }

    if (avcodec_open2(pcodec_ctx, pcodec, NULL) < 0) {
        cout << "couldn't open codec\n" << endl;
        return;
    }


    pframe = av_frame_alloc();
    pframe_rgb = av_frame_alloc();
    assert(pframe && pframe_rgb);

    int num_bytes = avpicture_get_size(AV_PIX_FMT_BGRA, 
                        pcodec_ctx->width, pcodec_ctx->height);
    buffer = (uint8_t*)av_malloc(num_bytes * sizeof(uint8_t));

    avpicture_fill((AVPicture *)pframe_rgb, buffer, AV_PIX_FMT_BGRA,
                        pcodec_ctx->width, pcodec_ctx->height);

    cout << pcodec_ctx->pix_fmt << endl;
    sws_ctx = sws_getContext(
        pcodec_ctx->width,
        pcodec_ctx->height,
        pcodec_ctx->pix_fmt,
        pcodec_ctx->width,
        pcodec_ctx->height,
        AV_PIX_FMT_BGRA,
        SWS_BILINEAR,
        NULL,
        NULL,
        NULL
    );
    cout << "Init Frame Receiver Suc" << endl;
}

void FrameReceiver::startReceive() {
    receive_thread = new thread(&FrameReceiver::receive, this);
}

void FrameReceiver::copyDate(AVFrame *picture,int width,int height)
{

    int             nChannels;
    int             stepWidth;
    uchar*  pData;
    cv::Mat frameImage(cv::Size(width, height), CV_8UC4, cv::Scalar(0));
    stepWidth = frameImage.step;
    nChannels = frameImage.channels();
    pData     = frameImage.data;

    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            pData[i*stepWidth+j*nChannels+0] = picture->data[0][i*picture->linesize[0]+j*nChannels+0];
            pData[i*stepWidth+j*nChannels+1] = picture->data[0][i*picture->linesize[0]+j*nChannels+1];
            pData[i*stepWidth+j*nChannels+2] = picture->data[0][i*picture->linesize[0]+j*nChannels+2];
            pData[i*stepWidth+j*nChannels+3] = picture->data[0][i*picture->linesize[0]+j*nChannels+3];
        }
    }
    cvtColor(frameImage, frameImage, CV_RGBA2RGB);
    cv::flip(frameImage, frameImage, 1);
    lock_guard<mutex> lk(m);
    frameImage.copyTo(cur_frame);
    // cv::namedWindow("Video");
    // cv::imshow("Video", frameImage);
    // cv::waitKey(1);
    // cout << "frame size" << frameImage.size() << " " << cur_frame.size() << endl;
}

void FrameReceiver::stopReceive() {
    stop = true;
}

Mat FrameReceiver::getCurFrame() {
    lock_guard<mutex> lk(m);
    Mat frame;
    cur_frame.copyTo(frame);
    return frame;
}

bool FrameReceiver::isStop() {
    return stop;
}

void FrameReceiver::receive() {
    cout << "start receive" << endl;
    int frame_finished;
    while (av_read_frame(pctx, &pkt) >= 0) {
        if (stop) {
            break;
        }
        if (pkt.stream_index != video_stream) {
            continue;
        }
        avcodec_decode_video2(pcodec_ctx, pframe, &frame_finished, &pkt);
        if (!frame_finished)
            continue;
        sws_scale(sws_ctx, pframe->data, pframe->linesize,
            0, pcodec_ctx->height, pframe_rgb->data, pframe_rgb->linesize);
        copyDate(pframe_rgb, pcodec_ctx->width, pcodec_ctx->height);
    }
    if (!stop) {
        stop = true;
        cout << "receiver timeout" << endl;
    }
    cout << "stop receive" << endl;
}

FrameReceiver::~FrameReceiver() {
    cout << "Release Frame Receiver" << endl;
    stopReceive();
    receive_thread->join();
    delete receive_thread;
    
    av_free_packet(&pkt);

    // free the RGB image
    av_free(buffer);
    av_free(pframe_rgb);
    // free raw frame
    av_free(pframe);
    // close codecs
    avcodec_close(pcodec_ctx);
    avcodec_close(pcodec_ctx_orig);
    // close video file
    avformat_close_input(&pctx);
}
