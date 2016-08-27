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
#include <array>
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
  _outputParams.push_back(_outputIsCalibrated);
  _outputParams.push_back(_outputAvgReprojErr);
  _outputParams.push_back(_outputCameraFocalLenght);
  _outputParams.push_back(_outputCameraPrincipalPointOffset);
  _outputParams.push_back(_outputLensDistortionRadialCoef1);
  _outputParams.push_back(_outputLensDistortionRadialCoef2);
  _outputParams.push_back(_outputLensDistortionRadialCoef3);
  _outputParams.push_back(_outputLensDistortionTangentialCoef1);
  _outputParams.push_back(_outputLensDistortionTangentialCoef2);
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
    convertRGBImage(inputImageMVG, outputImageOFX);
  }
  else
  {
    bool found = false;
    // Detect checkerboard for calibration
    if(checkerPerFrame.count(args.time) == 0) // if not already extracted
    {
      std::cout << "Detect checkerboard for calibration at frame " << args.time << std::endl;
      std::cout << "checkerPerFrame.size(): " << checkerPerFrame.size() << std::endl;
      OfxPointI imageSizeParamValue(_inputImageSize->getValue());
      OfxPointI imageSizeMVG{static_cast<int>(inputImageOFX.getWidth()), static_cast<int>(inputImageOFX.getHeight())};
      if(checkerPerFrame.empty())
        // If no checkerboard collected, initialize with the current image size
        _inputImageSize->setValue(inputImageOFX.getWidth(), inputImageOFX.getHeight());
      else if(imageSizeParamValue.x != imageSizeMVG.x || imageSizeParamValue.y != imageSizeMVG.y)
      {
        std::cerr << "All images don't have the same size." << std::endl;
//        throw std::logic_error("All images don't have the same size.");
        return;
      }

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
      std::cout << "inputPatternSize: " << boardSize.width << ", " << boardSize.height << std::endl;
      EParamPatternType inputPatternType = EParamPatternType(_inputPatternType->getValue());
      std::cout << "inputPatternType: " << int(inputPatternType) << std::endl;
      openMVG::calibration::Pattern patternType = getPatternType(inputPatternType);
      std::cout << "patternType openMVG: " << int(patternType) << std::endl;
      std::vector<cv::Point2f> checkerPoints;
      found = openMVG::calibration::findPattern(patternType, cvInputGrayImage, boardSize, checkerPoints);
      if(found)
        checkerPerFrame[args.time] = checkerPoints;
    }
    OFX::Image *outputPtr = _dstClip->fetchImage(args.time);
    if(outputPtr == NULL)
    {
      std::cout << "Output image is NULL" << std::endl;
      return;
    }
    Common::Image<float> outputImage(outputPtr, Common::eOrientationTopDown);
    outputImage.copyFrom(inputImageOFX);

    if(found)
      std::cout << "Checker found at time " << args.time << "." << std::endl;
    else
      std::cout << "Checker NOT found at time " << args.time << "." << std::endl;

    std::cout << "checkerPerFrame.size(): " << checkerPerFrame.size() << std::endl;
    std::cout << "checkerPerFrame.at(time).size(): " << checkerPerFrame.at(args.time).size() << std::endl;
    // TODO: export number of images for calibration to a user parameter
  }
}

void LensCalibrationPlugin::calibrateLens()
{
  if(checkerPerFrame.empty())
    throw std::logic_error("No checkerboard detected.");
  
  OfxPointI p(_inputPatternSize->getValue());
  cv::Size boardSize(p.x, p.y);
  EParamPatternType inputPatternType = EParamPatternType(_inputPatternType->getValue());
  openMVG::calibration::Pattern patternType = getPatternType(inputPatternType);
  
  std::vector<std::size_t> remainingImagesIndexes(checkerPerFrame.size());
  std::vector<float> calibImageScore;
  std::vector<std::size_t> calibInputFrames;
  std::vector<std::vector<cv::Point2f> > calibImagePoints;
  std::vector<long unsigned int> validFrames;
  std::vector<std::vector<cv::Point2f> > imagePoints;
  
  OfxPointI imageSizeValue(_inputImageSize->getValue());
  cv::Size imageSize(imageSizeValue.x, imageSizeValue.y);

  for (std::map<OfxTime, std::vector<cv::Point2f> >::iterator it = checkerPerFrame.begin(); it != checkerPerFrame.end(); ++it)
  {
    validFrames.push_back(it->first);
    imagePoints.push_back(it->second);
  }
  
  openMVG::calibration::selectBestImages(imagePoints, imageSize, remainingImagesIndexes, _inputMaxCalibFrames->getValue(),
                                        validFrames, calibImageScore, calibInputFrames, calibImagePoints, _inputCalibGridSize->getValue());
  
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
  
  setOutputParams(_outputCameraFocalLenght,
                  _outputCameraPrincipalPointOffset,
                  _outputLensDistortionRadialCoef1,
                  _outputLensDistortionRadialCoef2,
                  _outputLensDistortionRadialCoef3,
                  _outputLensDistortionTangentialCoef1,
                  _outputLensDistortionTangentialCoef2,
                  cameraMatrix, distCoeffs);
  
  _outputIsCalibrated->setValue(isCalibrated);
}


bool LensCalibrationPlugin::isIdentity(const OFX::IsIdentityArguments &args, OFX::Clip * &identityClip, double &identityTime)
{
  return false;
}

void LensCalibrationPlugin::changedClip(const OFX::InstanceChangedArgs &args, const std::string &clipName)
{
  
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
  
  //Clear All
  if(paramName == kParamOutputClear)
  {
    clearOutputParamValues();
    return;
  }
}


} //namespace LensCalibration
} //namespace openMVG_ofx