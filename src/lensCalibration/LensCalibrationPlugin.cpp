#include "LensCalibrationPlugin.hpp"
#include "LensCalibration.hpp"
#include "../common/Image.hpp"

#include <openMVG/calibration/patternDetect.hpp>
#include <openMVG/calibration/bestImages.hpp>
#include <openMVG/calibration/calibration.hpp>
#include <openMVG/calibration/exportData.hpp>
#include <openMVG/cameras/Camera_undistort_image.hpp>
#include <openMVG/image/pixel_types.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <map>
#include <vector>
#include <stdio.h>
#include <cassert>
#include <iostream>
#include <algorithm>
#include <stdexcept>

namespace openMVG_ofx {
namespace LensCalibration {


LensCalibrationPlugin::LensCalibrationPlugin(OfxImageEffectHandle handle)
  : OFX::ImageEffect(handle)
{
}

void LensCalibrationPlugin::syncPrivateData()
{
  std::cout << "LensCalibrationPlugin::syncPrivateData" << std::endl;
}

void LensCalibrationPlugin::beginSequenceRender(const OFX::BeginSequenceRenderArguments &args)
{

}

void LensCalibrationPlugin::endSequenceRender(const OFX::EndSequenceRenderArguments &args)
{

}

void LensCalibrationPlugin::render(const OFX::RenderArguments &args)
{
  std::cout << "render : time: " << args.time << std::endl;
  std::cout << "render : fieldToRender: " << args.fieldToRender << std::endl;
  std::cout << "render : renderQualityDraft: " << args.renderQualityDraft << std::endl;
  std::cout << "render : renderScale: " << args.renderScale.x << ", " << args.renderScale.y << std::endl;
  std::cout << "render : interactiveRenderStatus: " << args.interactiveRenderStatus << std::endl;
  std::cout << "render : args.renderWindow: (" << args.renderWindow.x1 << ", " << args.renderWindow.y1 << "), (" << args.renderWindow.x2 << ", "  << args.renderWindow.y2 << ")" << std::endl;

  if(abort())
  {
    std::cout << "render : abort" << std::endl;
    return;
  }
  OFX::Image *inputPtr = _srcClip->fetchImage(args.time);
  if(inputPtr == NULL)
  {
    std::cout << "Input image is NULL" << std::endl;
    return;
  }
  const Common::Image<float> inputImageOFX(inputPtr, Common::eOrientationTopDown);

  if(_outputIsCalibrated->getValue())
  {
    OfxPointD principalPoint = _outputCameraPrincipalPointOffset->getValue();
    
    // Lens already calibrated, directly undistort the input image
    openMVG::cameras::Pinhole_Intrinsic_Radial_K3 camera(inputImageOFX.getWidth(),
                                                         inputImageOFX.getHeight(),
                                                         _outputCameraFocalLenght->getValue(),
                                                         principalPoint.x,
                                                         principalPoint.y,
                                                         _outputLensDistortionRadialCoef1->getValue(),
                                                         _outputLensDistortionRadialCoef2->getValue(),
                                                         _outputLensDistortionRadialCoef3->getValue());

    openMVG::image::Image<openMVG::image::RGBfColor> inputImageMVG;
    inputImageMVG.resize(inputImageOFX.getWidth(), inputImageOFX.getHeight());
    
    // Convert OFX image to MVG image in order to call the openMVG undistort function
    convertRGBImage(inputImageOFX, inputImageMVG);
    openMVG::image::Image<openMVG::image::RGBfColor> outputImageMVG;
    openMVG::cameras::UndistortImage(inputImageMVG, &camera, outputImageMVG);
    
    OFX::Image *outputPtr = _dstClip->fetchImage(args.time);
    if(outputPtr == NULL)
    {
      std::cout << "Output image is NULL" << std::endl;
      return;
    }
    Common::Image<float> outputImageOFX(outputPtr, Common::eOrientationTopDown);
    convertRGBImage(outputImageMVG, outputImageOFX);
    std::cout << "Frame calibrate" << std::endl;
  }
  else
  {
    // Detect checkerboard for calibration
    if(_checkerPerFrame.count(args.time) == 0) // if not already extracted
    {
      std::cout << "Detect checkerboard for calibration at frame " << args.time << std::endl;
      std::cout << "checkerPerFrame.size(): " << _checkerPerFrame.size() << std::endl;
      OfxPointI imageSizeParamValue(_inputImageSize->getValue());
      OfxPointI imageSizeMVG{inputImageOFX.getWidth(), inputImageOFX.getHeight()};

      if(_checkerPerFrame.empty())
      {
        // If no checkerboard collected, initialize with the current image size
        _inputImageSize->setValue(inputImageOFX.getWidth(), inputImageOFX.getHeight());
      }
      else if(imageSizeParamValue.x != imageSizeMVG.x || imageSizeParamValue.y != imageSizeMVG.y)
      {
        std::cerr << "All images don't have the same size." << std::endl;
        return;
      }
      
      // Convert the input image into a gray 8 bits image
      cv::Mat cvInputGrayImage(inputImageOFX.getHeight(), inputImageOFX.getWidth(), cv::DataType<unsigned char>::type);
      if(_inputImageIsGray->getValue())
      {
        convertGGG32ToGRAY8(inputImageOFX, cvInputGrayImage);
      }
      else
      {
        convertRGB32ToGRAY8(inputImageOFX, cvInputGrayImage);
      }

      OfxPointI p(_inputPatternSize->getValue());
      cv::Size boardSize(p.x, p.y);
      EParamPatternType inputPatternType = EParamPatternType(_inputPatternType->getValue());
      openMVG::calibration::Pattern patternType = getPatternType(inputPatternType);

      // Store the checker points of the found pattern
      std::vector<cv::Point2f> checkerPoints;
      const int found = openMVG::calibration::findPattern(patternType, cvInputGrayImage, boardSize, checkerPoints);
      if(found)
      {
        _checkerPerFrame[args.time] = checkerPoints;
        std::cout << "Checker found at time " << args.time << "." << std::endl;
      }
      else
        std::cout << "Checker NOT found at time " << args.time << "." << std::endl;
        
      std::cout << "checkerPerFrame.size(): " << _checkerPerFrame.size() << std::endl;
      std::cout << "checkerPerFrame.at(time).size(): " << _checkerPerFrame.at(args.time).size() << std::endl;
    }

    OFX::Image *outputPtr = _dstClip->fetchImage(args.time);
    if(outputPtr == NULL)
    {
      std::cout << "Output image is NULL" << std::endl;
      return;
    }

    Common::Image<float> outputImage(outputPtr, Common::eOrientationTopDown);
    outputImage.copyFrom(inputImageOFX);

  }
}

void LensCalibrationPlugin::calibrateLens()
{
  if(_checkerPerFrame.empty())
    throw std::logic_error("No checkerboard detected.");

  OfxPointI p(_inputPatternSize->getValue());
  cv::Size boardSize(p.x, p.y);

  std::vector<std::size_t> remainingImagesIndexes(_checkerPerFrame.size());
  std::vector<float> calibImageScore;
  std::vector<std::size_t> calibInputFrames;
  std::vector<std::vector<cv::Point2f> > calibImagePoints;
  std::vector<long unsigned int> validFrames;
  std::vector<std::vector<cv::Point2f> > imagePoints;
  
  // Set the number of detected checkers
  _outputNbCheckersDetected->setValue(remainingImagesIndexes.size());

  OfxPointI imageSizeValue(_inputImageSize->getValue());
  cv::Size imageSize(imageSizeValue.x, imageSizeValue.y);
  
  // Fill validFrames vector with the id of the detected checkers and imagePoints vector with the checker points vectors
  for (std::map<OfxTime, std::vector<cv::Point2f> >::iterator it = _checkerPerFrame.begin(); it != _checkerPerFrame.end(); ++it)
  {
    validFrames.push_back(it->first);
    imagePoints.push_back(it->second);
  }
  
  // Select only the best images for the calibration among the valid frames
  openMVG::calibration::selectBestImages(imagePoints, imageSize, remainingImagesIndexes, _inputMaxCalibFrames->getValue(),
                                        validFrames, calibImageScore, calibInputFrames, calibImagePoints, _inputCalibGridSize->getValue());

  // Get openMVG pattern type enum from Plugin display choice enum
  EParamPatternType inputPatternType = EParamPatternType(_inputPatternType->getValue());
  openMVG::calibration::Pattern patternType = getPatternType(inputPatternType);

  // Create an object which stores all the checker points of the images
  std::vector<std::vector<cv::Point3f> > calibObjectPoints;
  openMVG::calibration::computeObjectPoints(boardSize, patternType, _inputSquareSize->getValue(), calibImagePoints, calibObjectPoints);

  int cvCalibFlags = 0;
  double totalAvgErr = 0;
  std::vector<cv::Mat> rvecs;
  float aspectRatio = 1.f;
  std::vector<cv::Mat> tvecs;
  std::vector<float> reprojErrs;
  std::vector<std::size_t> rejectInputFrames;

  cvCalibFlags |= CV_CALIB_ZERO_TANGENT_DIST;
  const std::array<int, 6> fixDistortionCoefs = {CV_CALIB_FIX_K1, CV_CALIB_FIX_K2, CV_CALIB_FIX_K3, CV_CALIB_FIX_K4, CV_CALIB_FIX_K5, CV_CALIB_FIX_K6};
  for (int i = _inputNbRadialCoef->getValue(); i < 6; ++i)
      cvCalibFlags |= fixDistortionCoefs[i];

  cv::Mat distCoeffs;
  cv::Mat cameraMatrix;
  // Store the result of the calibration's refinement loop
  bool isCalibrated = openMVG::calibration::calibrationIterativeOptimization(calibImagePoints,
                                                                             calibObjectPoints,
                                                                             imageSize,
                                                                             aspectRatio,
                                                                             cvCalibFlags,
                                                                             cameraMatrix,
                                                                             distCoeffs,
                                                                             rvecs,
                                                                             tvecs,
                                                                             reprojErrs,
                                                                             totalAvgErr, 
                                                                             _inputMaxTotalAvgErr->getValue(),
                                                                             _inputMinInputFrames->getValue(),
                                                                             calibInputFrames,
                                                                             calibImageScore,
                                                                             rejectInputFrames);

  setOutputParams(_outputIsCalibrated,
                  _outputAvgReprojErr,
                  _outputCameraFocalLenght,
                  _outputCameraPrincipalPointOffset,
                  _outputLensDistortionRadialCoef1,
                  _outputLensDistortionRadialCoef2,
                  _outputLensDistortionRadialCoef3,
                  _outputLensDistortionTangentialCoef1,
                  _outputLensDistortionTangentialCoef2,
                  isCalibrated, totalAvgErr,
                  cameraMatrix, distCoeffs);
}

bool LensCalibrationPlugin::isIdentity(const OFX::IsIdentityArguments &args, OFX::Clip * &identityClip, double &identityTime)
{
  return false;
}

void LensCalibrationPlugin::changedClip(const OFX::InstanceChangedArgs &args, const std::string &clipName)
{
  if(args.reason != OFX::InstanceChangeReason::eChangeTime)
  {
    clearAllData();
  }
}

void LensCalibrationPlugin::changedParam(const OFX::InstanceChangedArgs &args, const std::string &paramName)
{
  //Calibrate
  if(paramName == kParamCalibrate)
  {
    if(_outputIsCalibrated->getValue())
    {
      sendMessage(OFX::Message::eMessageError, "alreadycalibrated", "The lens is already calibrated. Change the isCalibrated status to add new image in order to recalibrate.");
      return;
    }
    calibrateLens();
    return;
  }

  //Clear Calibration
  if(paramName == kParamOutputClearCalibration)
  {
    clearCalibration();
    return;
  }

  //Clear All
  if(paramName == kParamOutputClearAll)
  {
    clearAllData();
    return;
  }
}

void LensCalibrationPlugin::clearAllData()
{
  _checkerPerFrame.clear();
  clearCalibration();
}

void LensCalibrationPlugin::clearCalibration()
{
  clearOutputParamValues();
  //invalid render
}

void LensCalibrationPlugin::clearOutputParamValues()
{
  _outputNbCheckersDetected->setValue(0);
  _outputIsCalibrated->setValue(0);
  _outputAvgReprojErr->setValue(0);
  _outputCameraFocalLenght->setValue(0);
  _outputCameraPrincipalPointOffset->setValue(0,0);
  _outputLensDistortionRadialCoef1->setValue(0);
  _outputLensDistortionRadialCoef2->setValue(0);
  _outputLensDistortionRadialCoef3->setValue(0);
  _outputLensDistortionTangentialCoef1->setValue(0);
  _outputLensDistortionTangentialCoef2->setValue(0);
}
} //namespace LensCalibration
} //namespace openMVG_ofx
