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
#include<sstream>
#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    string dazu_image_path = "/home/xzw/dataset/dazu/3/rgb/";
    string dazu_settings_path = "/home/xzw/WorkSpace/ORB_SLAM2/Examples/Monocular/dazu.yaml";
    string dazu_voc_path = "/home/xzw/WorkSpace/ORB_SLAM2/Vocabulary/ORBvoc.txt";
    int nImages = 10000;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(dazu_voc_path,dazu_settings_path,ORB_SLAM2::System::MONOCULAR,true);
    
    // Main loop
    cv::Mat im;
    double tframe = 0;
    ostringstream oss;
    for(int ni=0; ni<=nImages; ni++)
    {
        // Read image from file
        oss << setfill('0') << setw(4) << ni;
        string image_path = dazu_image_path + "/" + oss.str() + ".png";
        oss.str("");
        im = cv::imread(image_path, CV_LOAD_IMAGE_UNCHANGED);
        tframe += 1 / 30.0;

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << image_path << endl;
            break;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        usleep(1 / 30.0 * 1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}
