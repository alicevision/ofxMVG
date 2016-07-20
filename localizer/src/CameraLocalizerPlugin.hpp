#pragma once
#include "ofxsImageEffect.h"
#include "CameraLocalizer.hpp"
#include "CameraLocalizerPluginFactory.hpp"
#include"CameraLocalizerPluginDefinition.hpp"

//Maximum number of input clip 
#define K_MAX_INPUTS 5


namespace openMVG_ofx {

namespace Localizer {

/**
 * @brief CameraLocalizerPlugin Class
 */
class CameraLocalizerPlugin : public OFX::ImageEffect 
{   
private:
  //(!) Don't delete these, OFX::ImageEffect is managing them
  
  //Clips
  OFX::Clip *_srcClip[K_MAX_INPUTS]; //Source clip
  OFX::Clip *_dstClip = fetchClip(kOfxImageEffectOutputClipName); //Destination clip
  
  //Input Parameters
  OFX::StringParam *_inputCalibrationFile[K_MAX_INPUTS];
  OFX::ChoiceParam *_inputLensDistortion[K_MAX_INPUTS];
  OFX::ChoiceParam *_inputLensDistortionMode[K_MAX_INPUTS];
  OFX::DoubleParam *_inputLensDistortionCoef1[K_MAX_INPUTS];
  OFX::DoubleParam *_inputLensDistortionCoef2[K_MAX_INPUTS];
  OFX::DoubleParam *_inputLensDistortionCoef3[K_MAX_INPUTS];
  OFX::DoubleParam *_inputLensDistortionCoef4[K_MAX_INPUTS];
  OFX::Double2DParam *_inputOpticalCenter[K_MAX_INPUTS];
  OFX::ChoiceParam *_inputFocalLengthMode[K_MAX_INPUTS];
  OFX::DoubleParam *_inputFocalLength[K_MAX_INPUTS];
  OFX::BooleanParam *_inputFocalLengthVarying[K_MAX_INPUTS];
  OFX::Double3DParam *_inputRelativePoseTranslate[K_MAX_INPUTS];
  OFX::Double3DParam *_inputRelativePoseRotate[K_MAX_INPUTS];
  OFX::Double3DParam *_inputRelativePoseScale[K_MAX_INPUTS];
  
  //Global Parameters
  OFX::ChoiceParam *_featureType = fetchChoiceParam(kParamFeaturesType);
  OFX::ChoiceParam *_featurePreset = fetchChoiceParam(kParamFeaturesPreset);
  OFX::StringParam *_voctreeFile = fetchStringParam(kParamVoctreeFile);
  OFX::ChoiceParam *_rigMode = fetchChoiceParam(kParamRigMode);
  
  //Advanced Parameters
  OFX::ChoiceParam *_algorithm = fetchChoiceParam(kParamAdvancedAlgorithm);
  OFX::DoubleParam *_reprojectionError = fetchDoubleParam(kParamAdvancedReprojectionError);
  OFX::IntParam *_nbImageMatch = fetchIntParam(kParamAdvancedNbImageMatch);
  OFX::IntParam *_maxResults = fetchIntParam(kParamAdvancedMaxResults);
  OFX::StringParam *_voctreeWeights = fetchStringParam(kParamAdvancedVoctreeWeights);
  OFX::IntParam *_matchingError = fetchIntParam(kParamAdvancedMatchingError);
  OFX::IntParam *_cctagNbNearestKeyFrames = fetchIntParam(kParamAdvancedCctagNbNearestKeyFrames);
  OFX::IntParam *_baMinPointVisibility = fetchIntParam(kParamAdvancedBaMinPointVisibility);
  OFX::StringParam *_debugFolder = fetchStringParam(kParamAdvancedDebugFolder);    
 
  //Camera Output Parameters
  OFX::IntParam *_cameraOutputIndex = fetchIntParam(kParamOutputIndex);
  OFX::Double3DParam *_cameraOutputTranslate = fetchDouble3DParam(kParamOutputTranslate);
  OFX::Double3DParam *_cameraOutputRotate = fetchDouble3DParam(kParamOutputRotate);
  OFX::Double3DParam *_cameraOutputScale = fetchDouble3DParam(kParamOutputScale);
  OFX::Double2DParam *_cameraOutputOpticalCenter = fetchDouble2DParam(kParamOutputOpticalCenter);
  OFX::DoubleParam *_cameraOutputFocalLength = fetchDoubleParam(kParamOutputFocalLength);
  OFX::DoubleParam *_cameraOutputNear = fetchDoubleParam(kParamOutputNear);
  OFX::DoubleParam *_cameraOutputFar = fetchDoubleParam(kParamOutputFar);
  
  //Process Data
  //LocalizerProcessData _processData;
  
public:
  
  /**
   * @brief Plugin Constructor
   * @param handle
   */
  CameraLocalizerPlugin(OfxImageEffectHandle handle);
  
  /**
   * @brief Update if needed the localizer param data structure
   */
  void parametersSetup();

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
};


} //namespace Localizer

} //namespace openMVG_ofx
