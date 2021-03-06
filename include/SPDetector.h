#ifndef SPDETECTOR_HPP
#define SPDETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <torch/torch.h>
#include <string>
#include <vector>
#include <iostream>
#include "SuperPoint.h"

namespace SuperPointSLAM
{



#define EPSILON 1e-19
#define HALF_PATCH_SIZE 15

class SPDetector
{
public:
    /**
     * @brief Construct a new SPDetector::SPDetector object. 
     * When Initialize SPDetector, (1) First we initialize 
     * SuperPoint Class with weight_dir and use_cuda arguments. 
     * (2) and Move to device(cpu or gpu) we'll use. 
     * (3) Make the model eveluation mode, too.
     * 
     * @param _weight_dir the PATH that contains pretrained weight.
     * @param _use_cuda whether the model operates in cpu or gpu.
     */
    SPDetector(int _nfeatures, float _scaleFactor, int _nlevels,
                float _IniThresSP, float _MinThresSP);
    ~SPDetector(){}

    int inline GetLevels(){
        return nlevels;}

    float inline GetScaleFactor(){
        return scaleFactor;}

    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }
    
    /**
     * @brief Detect input image's Keypoints and Compute Descriptor.
     * 
     * @param img Input image. We use img's deep copy object.
     * @return cv::Mat 
     */
    void detect(cv::InputArray _image, std::shared_ptr<SuperPointSLAM::SuperPoint> mpSPModel,
                    std::vector<cv::KeyPoint>& _keypoints, cv::Mat &_descriptors, int nlevels=1);
    
    int n_keypoints;

    std::vector<cv::Mat> mvImagePyramid;

private:
    c10::TensorOptions tensor_opts;     // Contains necessary info for creating proper at::Tensor
    c10::DeviceType mDeviceType;        // If our device can use the GPU, it has 'kCUDA', otherwise it has 'kCPU'.
    c10::Device mDevice; 

    torch::Tensor mProb; // Superpoint Output Probability Tensor              
    torch::Tensor mDesc; // Superpoint Output Descriptor Tensor               
    
    /**
     * kpts is [H, W] size Tensor. 
     * Its elements has 1 if there is a featrue, and 0 otherwise. 
     * After this function, kpts Tensor can guarantee that 
     * no overlapping feature points exist in each 3x3 patch.
     * 
     * This function is executed on the CPU because it requires direct access 
     * to the tensor elements. Therefore, in some cases, (Specifically, if 
     * detect() is done on the GPU,) performance can be greatly impaired  
     * because of the data travel time You can turn this option on and off 
     * using the 'nms' member variable.
     */
    void SemiNMS(at::Tensor& kpts);
    bool nms = true; // SemiNMS() on/off flag.

    int nfeatures;
    double scaleFactor;
    int nlevels;
    float IniThresSP;
    float MinThresSP;

    std::vector<int> mnFeaturesPerLevel;

    std::vector<int> umax;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
};

}


#endif