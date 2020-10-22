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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

#include "SPDetector.h"
#include "SPVocabulary.h"

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  

public:
    Tracking(System* pSys, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor); //, ORBVocabulary* pVoc

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);
    cv::Mat GrabImageSPMonocular(const cv::Mat &im, const double &timestamp);
    
    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);


public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Extra parameters
    int rType;
    int mLevelup;
    int windowSize;
    int LocalMapConstraint1;
    int LocalMapConstraint2;

    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;


    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();

    vector<float> mvTimesExt;
    vector<float> mvTimesIniPose;
    vector<float> mvTimesTrackLM;

    void PrintTable1(vector<float> &vTimesTrackTotal);
    void PrintTable1Value(vector<float> &times);

    vector<float> mvMatchRatio;
    void PrintMatchRatio();

protected:

    // Main tracking function. "It is independent of the input sensor."
    // After all Frame() constructor is done,
    // Initialization, Visualization update and Image matching 
    // are repeated depending on several "State"
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    
    // Map initialization for SuperPoint Monocular.
    // Since Matcher type is different(SPDetector != ORBextractor), some changes were required.
    // When using a monocular camera, Initialization is performed using two consecutive frames.
    void SPMonocularInitialization();
    
    // Finally, mState = OK;
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();

    // When there is no available motion model, tracking is performed with this function.
    // 1. Compute BoW vector and Match two Frames' Keypoints depending on mSensor.
    // 2. If there're more than 15 matches, Process 'SetPose' and 'Pose Optimization'.
    // 3. Remove Outliers, and return True if there are more than 10 matches left.
    bool TrackReferenceKeyFrame();

    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();

    // A function that updates "mvpLocalKeyFrames" 
    // by finding Local KeyFrames that matches the condition of ORB-SLAM2.
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //SuperPoint
    SuperPointSLAM::SPDetector *mpSPDetector;
    SuperPointSLAM::SPDetector *mpIniSPDetector;
    std::shared_ptr<SuperPointSLAM::SuperPoint> mpSPModel; 

    //BoW
    ORBVocabulary* mpORBVocabulary;
    SuperPointSLAM::SPVocabulary* mpSPVocabulary;

    KeyFrameDatabase* mpKeyFrameDB;
    
    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;

    // A vector with keyframes connected by 
    // "having a map point in common" with the current keyframe
    std::vector<KeyFrame*> mvpLocalKeyFrames;

    // A set of MapPoint Pointers contained by Local Keyframes.
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;

    // "ID of the frame" that succeeded in the last Relocalization.
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
