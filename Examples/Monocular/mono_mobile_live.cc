/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include<unistd.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<csignal>
#include<opencv2/core/core.hpp>

#include<System.h>
#include "Thirdparty/Network/FrameReceiver.h"
#include "Thirdparty/Network/PosePublisher.h"
using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

bool stop;
void signal_handle(int signum) {
    if (signum == SIGINT) {
        cout << "sigint" << endl;
        stop = true;
        // exit(0);
    }
}


int main(int argc, char **argv)
{
    signal(SIGINT, signal_handle);

    string voc_path;
    string settings_path;
    if (argc == 1) {
        voc_path = "Vocabulary/ORBvoc.bin";
        settings_path = "Examples/Monocular/camera2.yaml";
    }
    else if (argc == 3) {
        voc_path = argv[1];
        settings_path = argv[2];
    }
    else {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    string url = "rtmp://192.168.43.92:1935/live/test";
    PosePublisher publisher("192.168.43.92", 12345);
    cv::FileStorage fSettings(settings_path, cv::FileStorage::READ);
    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];
    publisher.sendIntrinsic(fx, fy, cx, cy);

    FrameReceiver receiver(url);
    receiver.startReceive();
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(voc_path, settings_path, ORB_SLAM2::System::MONOCULAR, true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;

    std::cout << endl << "-------" << endl;
    std::cout << "Start processing live ..." << endl;

    // Main loop
    cv::Mat im;
    cv::Mat Tcw;
    int fps = 30;
    double tframe = 0;
    for(int ni=0; !stop && !receiver.isStop(); ni++)
    {
        // Read image from file
        im = receiver.getCurFrame();
        transpose(im, im);
        flip(im, im, 0);
        if(im.empty())
        {
            cout << endl << "No image yet" << endl;
            continue;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        Tcw = SLAM.TrackMonocular(im,tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack.push_back(ttrack);

        // publisher.sendInt(SLAM.GetTrackingState());
        publisher.sendTcw(SLAM.GetTrackingState(), Tcw);
        // if (SLAM.GetTrackingState() == 2 && Tcw.cols == 4 && Tcw.rows == 4) {
        // }
        vector<ORB_SLAM2::MapMarker*> markers = SLAM.GetAllMapMarkers();
        if (!markers.empty()) {
            vector<MarkerInfo> marker_infos;
            for (auto marker : markers) {
                marker_infos.push_back({marker->mMarkerId, marker->GetPose()});
            }
            publisher.sendMarkers(marker_infos);
        }
        // Wait to load the next frame
        double T = 1.0 / fps;
        

        if(ttrack<T) {
            usleep((T-ttrack)*1e6);
        }
        tframe += 1.0 / fps;
    }

    // Stop all threads
    SLAM.Shutdown();
    int nImages = vTimesTrack.size();
    // Tracking time statistics
    std::sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni < nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    std::cout << "-------" << endl << endl;
    std::cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    std::cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void sendMarkers(const vector<ORB_SLAM2::MapMarker*> &markers) {
    

}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}
