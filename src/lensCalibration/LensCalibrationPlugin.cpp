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
#include <bits/stl_vector.h>

#ifdef HAVE_CCTAG
#include <cctag/utils/LogTime.hpp>
#endif

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
  _inputImageSize->setValue(inputImageOFX.getWidth(), inputImageOFX.getHeight());
  
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
      std::cout << "Pattern Type : " << getPatternType(inputPatternType) << std::endl;

      // Store the checker points of the found pattern
      std::vector<cv::Point2f> checkerPoints;
      std::vector<int> detectedId;

      int found = false;
      if(patternType == openMVG::calibration::ASYMMETRIC_CCTAG_GRID)
      {
#ifdef HAVE_CCTAG
          std::clock_t startCh = std::clock();
          const std::size_t nRings = 3;
          const int pipeId = 0;
          const std::size_t frame = 1;
          cctag::Parameters cctagParams(nRings);
          boost::ptr_list<cctag::ICCTag> cctags;
          cctag::logtime::Mgmt durations( 25 );

          cctag::cctagDetection(cctags, pipeId, frame, cvInputGrayImage, cctagParams, &durations);

          boost::ptr_list<cctag::ICCTag>::iterator iterCCTags = cctags.begin();
          for(; iterCCTags != cctags.end(); iterCCTags++)
          {
            // Ignore CCTags without identification
            if(iterCCTags->id() < 0)
              continue;
            
            // Store detected Id
            detectedId.push_back(iterCCTags->id());

            // Store pixel coordinates of detected markers
            cv::Point2f detectedPoint((float) iterCCTags->x(), (float) iterCCTags->y());
            checkerPoints.push_back(detectedPoint);
          }
          assert(detectedId.size() == checkerPoints.size());
          found = true;

          double durationCh = (std::clock() - startCh) / (double) CLOCKS_PER_SEC;
          std::cout << "Find asymmetric CCTag grid duration: " << durationCh << std::endl;
          
          if(found)
          {
            _cctagsPerFrame[args.time] = cctags;
          }
#endif
      }
      else
      {
        found = openMVG::calibration::findPattern(patternType, cvInputGrayImage, boardSize, detectedId, checkerPoints);
      }

      if(found)
      {
        _checkerPerFrame[args.time]._detectedPoints = checkerPoints;
        _checkerPerFrame[args.time]._pointsId = detectedId;

        // Number of checkerboard detected
        if(OFX::getImageEffectHostDescription()->hostName == "uk.co.thefoundry.nuke")
        {
          // Only animated parameters are updated in Nuke UI
          _outputNbCheckersDetected->deleteAllKeys();
          _outputNbCheckersDetected->setValueAtTime(args.time, _checkerPerFrame.size());
        }
        else
        {
          _outputNbCheckersDetected->setValue(_checkerPerFrame.size());
        }

        std::cout << "Checker found at time " << args.time << "." << std::endl;
      }
      else
        std::cout << "Checker NOT found at time " << args.time << "." << std::endl;

      std::cout << "checkerPerFrame.size(): " << _checkerPerFrame.size() << std::endl;
      std::cout << "checkerPerFrame.at(time).size(): " << _checkerPerFrame.at(args.time)._detectedPoints.size() << std::endl;
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

  OfxPointI imageSizeValue(_inputImageSize->getValue());
  cv::Size imageSize(imageSizeValue.x, imageSizeValue.y);

  // Fill validFrames vector with the id of the detected checkers and imagePoints vector with the checker points vectors
  std::vector<std::size_t> validFrames;
  std::vector<std::vector<cv::Point2f> > imagePoints;
  for (std::map<OfxTime, CheckerPoints>::iterator it = _checkerPerFrame.begin(); it != _checkerPerFrame.end(); ++it)
  {
    validFrames.push_back(it->first);
    imagePoints.push_back(it->second._detectedPoints);
  }

  std::vector<float> calibImageScore;
  std::vector<std::size_t> calibInputFrames;
  std::vector<std::vector<cv::Point2f> > calibImagePoints;
  std::vector<std::size_t> remainingImagesIndexes(imagePoints.size());
  // Select only the best images for the calibration among the valid frames
  openMVG::calibration::selectBestImages(
      imagePoints, imageSize, _inputMaxCalibFrames->getValue(), validFrames,
      _inputCalibGridSize->getValue(), calibImageScore,
      calibInputFrames, calibImagePoints, remainingImagesIndexes);

  // Get openMVG pattern type enum from Plugin display choice enum
  EParamPatternType inputPatternType = EParamPatternType(_inputPatternType->getValue());
  openMVG::calibration::Pattern patternType = getPatternType(inputPatternType);

  // Create an object which stores all the checker points of the images
  std::vector<std::vector<cv::Point3f> > calibObjectPoints;
  {
    std::vector<cv::Point3f> templateObjectPoints;
    // Generate the object points coordinates
    openMVG::calibration::calcChessboardCorners(templateObjectPoints, boardSize, _inputSquareSize->getValue(), patternType);
    // Assign the corners to all items
    for(std::size_t frame: calibInputFrames)
    {
      // For some chessboard (ie. CCTag), we have an identification per point,
      // and only a sub-part of the corners may be detected.
      // So we only keep the visible corners from the templateObjectPoints
      std::vector<int>& pointsId = _checkerPerFrame[frame]._pointsId;
      std::vector<cv::Point3f> objectPoints;
      for(size_t i = 0; i < pointsId.size(); ++i)
        objectPoints[i] = templateObjectPoints[pointsId[i]];
      calibObjectPoints.push_back(objectPoints);
    }
    assert(calibInputFrames.size() == calibImagePoints.size());
  }

  int cvCalibFlags = 0;
  double totalAvgErr = 0;
  std::vector<cv::Mat> rvecs;
  float aspectRatio = 1.f;
  std::vector<cv::Mat> tvecs;
  std::vector<float> reprojErrs;
  std::vector<std::size_t> rejectInputFrames; //Only for debug panel.

  cvCalibFlags |= CV_CALIB_ZERO_TANGENT_DIST;
  const std::array<int, 6> fixDistortionCoefs = {CV_CALIB_FIX_K1, CV_CALIB_FIX_K2, CV_CALIB_FIX_K3, CV_CALIB_FIX_K4, CV_CALIB_FIX_K5, CV_CALIB_FIX_K6};
  for (int i = _inputNbRadialCoef->getValue(); i < 6; ++i)
      cvCalibFlags |= fixDistortionCoefs[i];

  cv::Mat distCoeffs;
  cv::Mat cameraMatrix;
  // Store the result of the calibration's refinement loop.
  bool isCalibrated = openMVG::calibration::calibrationIterativeOptimization(imageSize,
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
                                                                             calibImagePoints,
                                                                             calibObjectPoints,
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
  
  sendMessage(OFX::Message::eMessageMessage, "calibrationFinished", "Calibration successful");
}

bool LensCalibrationPlugin::isIdentity(const OFX::IsIdentityArguments &args, OFX::Clip * &identityClip, double &identityTime)
{
  return false;
}

void LensCalibrationPlugin::changedClip(const OFX::InstanceChangedArgs &args, const std::string &clipName)
{}

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
  _outputNbCheckersDetected->setValue(0);
  _inputImageSize->setValue(0,0);
  clearCalibration();
}

void LensCalibrationPlugin::clearCalibration()
{
  clearOutputParamValues();
}

void LensCalibrationPlugin::clearOutputParamValues()
{
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
