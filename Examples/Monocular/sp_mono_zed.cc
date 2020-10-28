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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<unistd.h>

#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

/** 
 * Run SuperPoint-SLAM with Kitti Dataset.
 * Distortion Coefficients are all zeros. k1 = k2 = p1 = p2 = 0.
 * 
 */
int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./sp_mono_kitti path_to_vocabulary path_to_settings path_to_sequence repeat_num" << endl;
        return 1;
    }

    string videoFile(argv[3]);
    cv::VideoCapture cap;
    cap.open(videoFile, cv::CAP_ANY);
    if (!cap.isOpened()) {
        std::cerr << "ERROR! Unable to open video\n";
        std::exit(1);
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::SP_MONOCULAR,true);
    cv::FileStorage f(cv::String(argv[2]).c_str(), cv::FileStorage::READ);
    double fps = f["System.Fps"];

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    int ni = -1;
    // Main loop
    cv::Mat img;
    vector<float> vTimesTrack;
    long long step = 0;
    while(cap.read(img))
    {
        if((step++) % 3);
        else continue;

        ++ni;
        double tframe = cap.get(cv::CAP_PROP_POS_MSEC);
        tframe /= 1000.0;
        cout << tframe << endl;


#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        cv::resize(img, img, cv::Size(960, 540));
        // Pass the image to the SLAM system
        if(SLAM.rtype == 0)
            SLAM.TrackSPMonocular(img, tframe);
        else
        {
            cout << " Frame [ " << ni << " ]\n";
            SLAM.TrackSPMonocular(img, tframe);
            cout << '\n' << endl;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack.push_back(ttrack);

        // Wait to load the next frame
        // double T=0;
        // if(ni<nImages-1)
        //     T = vTimestamps[ni+1]-tframe;
        // else if(ni>0)
        //     T = tframe-vTimestamps[ni-1];

        // if(ttrack<T)
        //     usleep((T-ttrack)*1e6);

        cout << "Tracking time: " << ttrack << endl;
        
        // if(fps == 0.0) fps = T;
        
        if(ttrack < fps)
        {
            usleep((fps - ttrack)*1e6);
        }
        
        // if(ttrack < 0.1)
        //     usleep((0.1-ttrack)*1e6);
    }

    // Stop all threads
    string argv4 = argv[4];
    string output = "ZED2_output_" + argv4 + ".txt";
    vector<float> record;
    SLAM.Shutdown(vTimesTrack, output, record);

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    int nImages = vTimesTrack.size();
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }

    cout << "==================================" << endl << endl;
    printf("%15s: %5.3f\n", "Median Tracking", vTimesTrack[nImages/2]);     
    printf("%15s: %5.3f\n", "Mean Tracking", totaltime/nImages);            
    printf("%15s: %5d\n", "RunType", int(f["System.RunType"]));                 
    printf("%15s: %5d\n", "scaleFactor", (int)f["SPDetector.scaleFactor"]);
    printf("%15s: %5d\n", "nLevels", (int)f["SPDetector.nLevels"]);
    printf("%15s: %5.3f\n", "IniThresSP", (float)f["SPDetector.IniThresSP"]);
    printf("%15s: %5.3f\n", "MinThresSP", (float)f["SPDetector.MinThresSP"]);
    printf("%15s: %5d\n", "levelup", (int)f["SPMatcher.levelup"]);
    printf("%15s: %5d\n", "windowSize", (int)f["SPMatcher.windowSize"]);
    printf("%15s: %5.3f\n", "FPS Limit", (float)fps);
    
    cout << endl << "==================================" << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;
    cout << "-------" << endl << endl;

    {   // File Output
        string outputPath("/home/leecw/Reps/SuperPointSLAM/");
        outputPath += output;
        ofstream output(outputPath);
        // ofstream more_output(outputPath + "_detail.txt");  && more_output.is_open()
        if(output.is_open())
        {
            for(int i=0; i<record.size(); i++)
            {
                output << record[i] << endl;
            }

            output.close();
            // more_output.close();
        }
    }


    SLAM.SaveKeyFrameTrajectoryTUM("ZED2_" + argv4 + ".txt");        // Save camera trajectory

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}
