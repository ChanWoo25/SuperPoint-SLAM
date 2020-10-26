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

#include<System.h>

using namespace std;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

/** 
 * Run SuperPoint-SLAM with TUM Dataset.
 * Distortion Coefficients are all zeros. k1 = k2 = p1 = p2 = 0.
 * 
 */
int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./sp_mono_tum path_to_vocabulary path_to_settings path_to_sequence repeat_num" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    string strFile = string(argv[3])+"/rgb.txt";
    LoadImages(strFile, vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::SP_MONOCULAR,true);

    cv::FileStorage f(cv::String(argv[2]).c_str(), cv::FileStorage::READ);
    double fps = f["System.Fps"];

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(string(argv[3])+"/"+vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        if(SLAM.rtype == 0)
            SLAM.TrackSPMonocular(im,tframe);
        else
        {
            cout << " Frame [ " << ni << " ]\n";
            SLAM.TrackSPMonocular(im, tframe);
            cout << '\n' << endl;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        cout << "Tracking time: " << ttrack << endl;
        
        if(fps == 0.0) fps = T;
        
        if(ttrack < fps)
        {
            usleep((fps - ttrack)*1e6);
        }

    }

    // // Stop all threads
    // SLAM.Shutdown(vTimesTrack);
    // Stop all threads
    string argv4 = argv[4];
    string output = "tum_output_" + argv4 + ".txt";
    vector<float> record;
    SLAM.Shutdown(vTimesTrack, output, record);


    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;
    cout << "-------" << endl << endl;
    record.push_back(vTimesTrack[nImages/2]);
    record.push_back(totaltime/nImages);


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

    SLAM.SaveKeyFrameTrajectoryTUM("tum_" + argv4 + ".txt");        // Save camera trajectory

    return 0;
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
