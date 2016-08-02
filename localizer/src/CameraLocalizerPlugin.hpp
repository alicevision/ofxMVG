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
  OFX::BooleanParam *_overlay = fetchBooleanParam(kParamOverlay);
  OFX::ChoiceParam *_algorithm = fetchChoiceParam(kParamAdvancedAlgorithm);
  OFX::ChoiceParam *_estimatorMatching = fetchChoiceParam(kParamAdvancedEstimatorMatching);
  OFX::ChoiceParam *_estimatorResection = fetchChoiceParam(kParamAdvancedEstimatorResection);
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
  OFX::Double3DParam *_cameraOutputTranslate[K_MAX_INPUTS];
  OFX::Double3DParam *_cameraOutputRotate[K_MAX_INPUTS];
  OFX::Double3DParam *_cameraOutputScale[K_MAX_INPUTS];
  OFX::Double2DParam *_cameraOutputOpticalCenter[K_MAX_INPUTS];
  OFX::DoubleParam *_cameraOutputFocalLength[K_MAX_INPUTS];
  OFX::DoubleParam *_cameraOutputNear[K_MAX_INPUTS];
  OFX::DoubleParam *_cameraOutputFar[K_MAX_INPUTS];
  OFX::DoubleParam *_cameraOutputLensDistortionCoef1[K_MAX_INPUTS];
  OFX::DoubleParam *_cameraOutputLensDistortionCoef2[K_MAX_INPUTS];
  OFX::DoubleParam *_cameraOutputLensDistortionCoef3[K_MAX_INPUTS];
  OFX::DoubleParam *_cameraOutputLensDistortionCoef4[K_MAX_INPUTS];
  OFX::DoubleParam *_outputStatErrorMean[K_MAX_INPUTS];
  OFX::DoubleParam *_outputStatErrorMin[K_MAX_INPUTS];
  OFX::DoubleParam *_outputStatErrorMax[K_MAX_INPUTS];
  OFX::DoubleParam *_outputStatNbMatchedImages[K_MAX_INPUTS];
  OFX::DoubleParam *_outputStatNbDetectedFeatures[K_MAX_INPUTS];
  OFX::DoubleParam *_outputStatNbMatchedFeatures[K_MAX_INPUTS];
  OFX::DoubleParam *_outputStatNbInlierFeatures[K_MAX_INPUTS];
  
  //Output Cache Parameters
  OFX::PushButtonParam *_outputClear = fetchPushButtonParam(kParamOutputClear);
  
  //Invalidation Parameters
  OFX::IntParam *_forceInvalidation = fetchIntParam(kParamForceInvalidation);
  OFX::IntParam *_forceInvalidationAtTime = fetchIntParam(kParamForceInvalidationAtTime);
  
  //Process Data
  LocalizerProcessData _processData;
  bool _uptodateParam = false;
  bool _uptodateDescriptor = false;

  //Connected clip index vector
  std::vector<unsigned int> _connectedClipIdx;

  //Output Parameters List
  std::vector<OFX::ValueParam*> _outputParams;

  //Cache
  std::map<OfxTime, openMVG::localization::LocalizationResult> _localizationResultsAtTime;
  std::map<OfxTime, std::vector<openMVG::features::SIOPointFeature> > _extractedFeaturesAtTime;

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
  
  bool displayOverlay() const
  {
    return _overlay->getValue();
  }
  
  std::size_t getNbConnectedInput() const
  {
    return _connectedClipIdx.size();
  }
  
  bool hasInput() const
  {
    return (_connectedClipIdx.size() > 0);
  }
    
  bool isRigInInput() const
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
  
  void clearOutputParamValuesAtTime(OfxTime time)
  {
    _localizationResultsAtTime.erase(time);
    _extractedFeaturesAtTime.erase(time);
    
    for(OFX::ValueParam* outputParam: _outputParams)
      outputParam->deleteKeyAtTime(time);
  }
  
  void clearOutputParamValues()
  {
    _localizationResultsAtTime.clear();
    _extractedFeaturesAtTime.clear();

    for(OFX::ValueParam* outputParam: _outputParams)
      outputParam->deleteAllKeys();
  }

  bool hasCachedLocalizationResults(OfxTime time) const
  {
    return _localizationResultsAtTime.find(time) != _localizationResultsAtTime.end();
  }

  const openMVG::localization::LocalizationResult& getCachedLocalizationResults(OfxTime time) const
  {
    return _localizationResultsAtTime.at(time);
  }
  
  bool hasAllOutputParamKey(OfxTime time) const
  {
    for(auto input : _connectedClipIdx)
    {
     if(_cameraOutputTranslate[input]->getKeyIndex(time, OFX::eKeySearchNear) == -1)
       return false;
    }
    return true;
  }

  bool hasCachedFeatures(OfxTime time) const
  {
    return _extractedFeaturesAtTime.find(time) != _extractedFeaturesAtTime.end();
  }

  const std::vector<openMVG::features::SIOPointFeature>& getCachedFeatures(OfxTime time) const
  {
    return _extractedFeaturesAtTime.at(time);
  }

};


} //namespace Localizer
} //namespace openMVG_ofx
