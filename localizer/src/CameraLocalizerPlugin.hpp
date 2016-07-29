#pragma once
#include "ofxsImageEffect.h"
#include "CameraLocalizer.hpp"
#include "CameraLocalizerPluginFactory.hpp"
#include "CameraLocalizerPluginDefinition.hpp"

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
  OFX::BooleanParam *_inputIsGrayscale[K_MAX_INPUTS];
  OFX::StringParam *_inputLensCalibrationFile[K_MAX_INPUTS];
  OFX::DoubleParam *_inputSensorWidth[K_MAX_INPUTS];
  OFX::Double2DParam *_inputOpticalCenter[K_MAX_INPUTS];
  OFX::ChoiceParam *_inputFocalLengthMode[K_MAX_INPUTS];
  OFX::DoubleParam *_inputFocalLength[K_MAX_INPUTS];
  OFX::BooleanParam *_inputFocalLengthVarying[K_MAX_INPUTS];
  OFX::ChoiceParam *_inputLensDistortion[K_MAX_INPUTS];
  OFX::ChoiceParam *_inputLensDistortionMode[K_MAX_INPUTS];
  OFX::DoubleParam *_inputLensDistortionCoef1[K_MAX_INPUTS];
  OFX::DoubleParam *_inputLensDistortionCoef2[K_MAX_INPUTS];
  OFX::DoubleParam *_inputLensDistortionCoef3[K_MAX_INPUTS];
  OFX::DoubleParam *_inputLensDistortionCoef4[K_MAX_INPUTS];
  OFX::Double3DParam *_inputRelativePoseRotateM1[K_MAX_INPUTS];
  OFX::Double3DParam *_inputRelativePoseRotateM2[K_MAX_INPUTS];
  OFX::Double3DParam *_inputRelativePoseRotateM3[K_MAX_INPUTS];
  OFX::Double3DParam *_inputRelativePoseCenter[K_MAX_INPUTS];
  OFX::GroupParam *_inputGroupRelativePose[K_MAX_INPUTS];
  
  //Global Parameters
  OFX::ChoiceParam *_featureType = fetchChoiceParam(kParamFeaturesType);
  OFX::ChoiceParam *_featurePreset = fetchChoiceParam(kParamFeaturesPreset);
  OFX::StringParam *_reconstructionFile = fetchStringParam(kParamReconstructionFile);
  OFX::StringParam *_descriptorsFolder = fetchStringParam(kParamDescriptorsFolder);
  OFX::StringParam *_voctreeFile = fetchStringParam(kParamVoctreeFile);
  OFX::ChoiceParam *_rigMode = fetchChoiceParam(kParamRigMode);
  OFX::StringParam *_rigCalibrationFile = fetchStringParam(kParamRigCalibrationFile);
  
  //Advanced Parameters
  OFX::ChoiceParam *_algorithm = fetchChoiceParam(kParamAdvancedAlgorithm);
  OFX::DoubleParam *_reprojectionError = fetchDoubleParam(kParamAdvancedReprojectionError);
  OFX::IntParam *_nbImageMatch = fetchIntParam(kParamAdvancedNbImageMatch);
  OFX::IntParam *_maxResults = fetchIntParam(kParamAdvancedMaxResults);
  OFX::StringParam *_voctreeWeightsFile = fetchStringParam(kParamAdvancedVoctreeWeights);
  OFX::IntParam *_matchingError = fetchIntParam(kParamAdvancedMatchingError);
  OFX::IntParam *_cctagNbNearestKeyFrames = fetchIntParam(kParamAdvancedCctagNbNearestKeyFrames);
  OFX::IntParam *_baMinPointVisibility = fetchIntParam(kParamAdvancedBaMinPointVisibility);
  OFX::DoubleParam *_distanceRatio = fetchDoubleParam(kParamAdvancedDistanceRatio);
  OFX::BooleanParam *_useGuidedMatching = fetchBooleanParam(kParamAdvancedUseGuidedMatching);        
  OFX::StringParam *_debugFolder = fetchStringParam(kParamAdvancedDebugFolder);
  OFX::BooleanParam *_alwaysComputeFrame = fetchBooleanParam(kParamAdvancedDebugAlwaysComputeFrame);  
  
  //Tracking Parameters
  OFX::ChoiceParam *_trackingRangeMode = fetchChoiceParam(kParamTrackingRangeMode);
  OFX::IntParam *_trackingRangeMin = fetchIntParam(kParamTrackingRangeMin);
  OFX::IntParam *_trackingRangeMax = fetchIntParam(kParamTrackingRangeMax);
  OFX::PushButtonParam *_trackingButton = fetchPushButtonParam(kParamTrackingTrack);
 
  //Output Parameters
  OFX::IntParam *_cameraOutputIndex = fetchIntParam(kParamOutputIndex);
  OFX::Double3DParam *_cameraOutputTranslate = fetchDouble3DParam(kParamOutputTranslate);
  OFX::Double3DParam *_cameraOutputRotate = fetchDouble3DParam(kParamOutputRotate);
  OFX::Double3DParam *_cameraOutputScale = fetchDouble3DParam(kParamOutputScale);
  OFX::Double2DParam *_cameraOutputOpticalCenter = fetchDouble2DParam(kParamOutputOpticalCenter);
  OFX::DoubleParam *_cameraOutputFocalLength = fetchDoubleParam(kParamOutputFocalLength);
  OFX::DoubleParam *_cameraOutputNear = fetchDoubleParam(kParamOutputNear);
  OFX::DoubleParam *_cameraOutputFar = fetchDoubleParam(kParamOutputFar);
  OFX::DoubleParam *_cameraOutputLensDistortionCoef1 = fetchDoubleParam(kParamOutputDistortionCoef1);
  OFX::DoubleParam *_cameraOutputLensDistortionCoef2 = fetchDoubleParam(kParamOutputDistortionCoef2);
  OFX::DoubleParam *_cameraOutputLensDistortionCoef3 = fetchDoubleParam(kParamOutputDistortionCoef3);
  OFX::DoubleParam *_cameraOutputLensDistortionCoef4 = fetchDoubleParam(kParamOutputDistortionCoef4);
  
  OFX::DoubleParam *_outputErrorMean = fetchDoubleParam(kParamOutputErrorMean);
  OFX::DoubleParam *_outputErrorMin = fetchDoubleParam(kParamOutputErrorMin);
  OFX::DoubleParam *_outputErrorMax = fetchDoubleParam(kParamOutputErrorMax);
  OFX::PushButtonParam *_outputClear = fetchPushButtonParam(kParamOutputClear);
  
  // Invalidation Parameters
  OFX::IntParam *_forceInvalidation = fetchIntParam(kParamForceInvalidation);
  OFX::IntParam *_forceInvalidationAtTime = fetchIntParam(kParamForceInvalidationAtTime);
  
  //Process Data
  LocalizerProcessData _processData;
  bool _uptodateParam = false;
  bool _uptodateDescriptor = false;

  //Connected clip index vector
  std::vector<unsigned int> _connectedClipIdx;

  // cache
  std::map<OfxTime, openMVG::localization::LocalizationResult> _localizationResultsAtTime;

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
  
  /**
   * Reset all plugin parameters
   */
  void reset();
  
  /**
   * @brief update the member connected clip index collection
   */
  void updateConnectedClipIndexCollection();
  
  /**
   * @brief update camera output index parameter to fit with clip number
   */
  void updateCameraOutputIndexRange();
  
  /**
   * 
   */
  void updateRigOptions();
  
  /**
   * 
   * @param input
   */
  void updateLensDistortion(unsigned int input);
  
  /**
   * 
   * @param input
   */
  void updateLensDistortionMode(unsigned int input);
  
  /**
   * 
   * @param input
   */
  void updateFocalLength(unsigned int input);
  
  /**
   * 
   */
  void updateTrackingRangeMode();
  
  /**
   * 
   * @param time
   * @param inputClipIdx
   * @param queryIntrinsics
   */
  bool getInputIntrinsics(double time, unsigned int inputClipIdx, openMVG::cameras::Pinhole_Intrinsic &queryIntrinsics);
  
  /**
   * 
   * @param time
   * @param inputClipIdx
   * @param outputImage
   * @return 
   */
  bool getInputInGrayScale(double time, unsigned int inputClipIdx, openMVG::image::Image<unsigned char> &outputImage);
  
  std::size_t getNbConnectedInput()
  {
    return _connectedClipIdx.size();
  }
  
  bool hasInput()
  {
    return (_connectedClipIdx.size() > 0);
  }
    
  bool isRigInInput()
  {
    return (_connectedClipIdx.size() > 1);
  }

  void invalidRender()
  {
    _forceInvalidation->setValue(1 + _forceInvalidation->getValue());
  }

  void invalidRenderAtTime(OfxTime time)
  {
    _forceInvalidationAtTime->setValue(1 + _forceInvalidationAtTime->getValue());
  }

  bool hasCachedLocalizationResults(OfxTime time) const
  {
    return _localizationResultsAtTime.find(time) != _localizationResultsAtTime.end();
  }

  const openMVG::localization::LocalizationResult& getCachedLocalizationResults(OfxTime time) const
  {
    return _localizationResultsAtTime.at(time);
  }

};


} //namespace Localizer
} //namespace openMVG_ofx
