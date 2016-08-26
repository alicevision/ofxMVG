#pragma once
#include "ofxsImageEffect.h"
#include "LensCalibrationPluginFactory.hpp"
#include "LensCalibrationPluginDefinition.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace openMVG_ofx {
namespace LensCalibration {

/**
 * @brief LensCalibrationPlugin Class
 */
class LensCalibrationPlugin : public OFX::ImageEffect 
{
private:
  //(!) Don't delete these, OFX::ImageEffect is managing them
  
  //Clips
  OFX::Clip *_srcClip = fetchClip(kOfxImageEffectSimpleSourceClipName); //Source clip
  OFX::Clip *_dstClip = fetchClip(kOfxImageEffectOutputClipName); //Destination clip
  
  //Calibration parameters
  OFX::IntParam *_outputNbCheckersDetected = fetchIntParam(kParamNbCheckersDetected);
  OFX::BooleanParam *_outputIsCalibrated = fetchBooleanParam(kParamIsCalibrated);
  OFX::BooleanParam *_inputImageIsGray = fetchBooleanParam(kParamInputImageIsGray);
  OFX::Int2DParam *_inputImageSize = fetchInt2DParam(kParamImageSize);
  OFX::ChoiceParam *_inputPatternType = fetchChoiceParam(kParamPatternType);
  OFX::Int2DParam *_inputPatternSize = fetchInt2DParam(kParamPatternSize);
  OFX::DoubleParam *_inputSquareSize = fetchDoubleParam(kParamSquareSize);
  OFX::IntParam *_inputNbRadialCoef = fetchIntParam(kParamNbRadialCoef);
  OFX::IntParam *_inputMaxCalibFrames = fetchIntParam(kParamMaxCalibFrames);
  OFX::IntParam *_inputCalibGridSize = fetchIntParam(kParamCalibGridSize);
  OFX::IntParam *_inputMinInputFrames = fetchIntParam(kParamMinInputFrames);
  OFX::DoubleParam *_inputMaxTotalAvgErr = fetchDoubleParam(kParamMaxTotalAvgErr);

  //Output parameters
  OFX::DoubleParam *_outputAvgReprojErr = fetchDoubleParam(kParamOutputAvgReprojErr);
  OFX::DoubleParam *_outputCameraFocalLenght = fetchDoubleParam(kParamOutputFocalLenght);
  OFX::Double2DParam *_outputCameraPrincipalPointOffset = fetchDouble2DParam(kParamOutputPrincipalPointOffset);

  OFX::DoubleParam *_outputLensDistortionRadialCoef1 = fetchDoubleParam(kParamOutputRadialCoef1);
  OFX::DoubleParam *_outputLensDistortionRadialCoef2 = fetchDoubleParam(kParamOutputRadialCoef2);
  OFX::DoubleParam *_outputLensDistortionRadialCoef3 = fetchDoubleParam(kParamOutputRadialCoef3);
  OFX::DoubleParam *_outputLensDistortionTangentialCoef1 = fetchDoubleParam(kParamOutputTangentialCoef1);
  OFX::DoubleParam *_outputLensDistortionTangentialCoef2 = fetchDoubleParam(kParamOutputTangentialCoef2);

  //Debug parameters
  OFX::BooleanParam *_debugEnable = fetchBooleanParam(kParamDebugEnable);
  OFX::StringParam *_debugRejectedImgFolder = fetchStringParam(kParamDebugRejectedImgFolder);
  OFX::StringParam *_debugSelectedImgFolder = fetchStringParam(kParamDebugSelectedImgFolder);
  
  // Cache
  std::map<OfxTime, std::vector<cv::Point2f> > _checkerPerFrame;

public:
  
  /**
   * @brief Plugin Constructor
   * @param handle
   */
  LensCalibrationPlugin(OfxImageEffectHandle handle);

  /** @brief The sync private data action, called when the effect needs to sync any private data to persistant parameters */
  void syncPrivateData();

  /**
   * @brief client begin sequence render function
   * @param[in] args
   */
  void beginSequenceRender(const OFX::BeginSequenceRenderArguments &args);
  
  /**
   * @brief client end sequence render function
   * @param[in] args
   */
  void endSequenceRender(const OFX::EndSequenceRenderArguments &args);

  /**
   * @brief Override render method
   * @param[in] args
   */
  virtual void render(const OFX::RenderArguments &args);

  /**
   * @brief Override isIdentity method
   * @param[in] args
   * @param[in,out] identityClip
   * @param[in,out] identityTime
   * @return 
   */
  virtual bool isIdentity(const OFX::IsIdentityArguments &args, OFX::Clip * &identityClip, double &identityTime);
  
  /**
   * @brief Override changedClip method
   * @param[in] args
   * @param[in] clipName
   */
  virtual void changedClip(const OFX::InstanceChangedArgs &args, const std::string &clipName);

  /**
   * @brief Override changedParam method
   * @param[in] args
   * @param[in] paramName
   */
  virtual void changedParam(const OFX::InstanceChangedArgs &args, const std::string &paramName);
  
private:
  /**
   * @brief Calibrate lens distortion
   */
  void calibrateLens();
  
  /**
   * @brief Clear the calibration parameters and the group of images used for the calibration
   */
  void clearAllData();
  
  /**
   * @brief Clear only the calibration parameters
   */
  void clearCalibration();
  
  /**
   * @brief Clear all the output calibration parameters
   */
  void clearOutputParamValues();

};


} //namespace LensCalibration
} //namespace openMVG_ofx
