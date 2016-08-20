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
  OFX::PushButtonParam *_rigCalibration = fetchPushButtonParam(kParamRigCalibration);
  OFX::StringParam *_rigCalibrationFile = fetchStringParam(kParamRigCalibrationFile);
  OFX::PushButtonParam *_rigCalibrationLoad = fetchPushButtonParam(kParamRigCalibrationLoad);
  OFX::PushButtonParam *_rigCalibrationSave = fetchPushButtonParam(kParamRigCalibrationSave);
  
  //Advanced Parameters
  OFX::BooleanParam *_overlayDetectedFeatures = fetchBooleanParam(kParamAdvancedOverlayDetectedFeatures);
  OFX::BooleanParam *_overlayMatchedFeatures = fetchBooleanParam(kParamAdvancedOverlayMatchedFeatures);
  OFX::BooleanParam *_overlayResectionFeatures = fetchBooleanParam(kParamAdvancedOverlayResectionFeatures);
  OFX::BooleanParam *_overlayReprojectionError = fetchBooleanParam(kParamAdvancedOverlayReprojectionError);
  OFX::BooleanParam *_overlayReconstructionVisibility = fetchBooleanParam(kParamAdvancedOverlayReconstructionVisibility);
  OFX::BooleanParam *_overlayFeaturesId = fetchBooleanParam(kParamAdvancedOverlayFeaturesId);
  OFX::BooleanParam *_overlayFeaturesScaleOrientation = fetchBooleanParam(kParamAdvancedOverlayFeaturesScaleOrientation);
  OFX::DoubleParam *_overlayFeaturesScaleOrientationRadius = fetchDoubleParam(kParamAdvancedOverlayFeaturesScaleOrientationRadius);
  OFX::BooleanParam *_overlayTracks = fetchBooleanParam(kParamAdvancedOverlayTracks);
  OFX::IntParam *_overlayTracksWindowSize = fetchIntParam(kParamAdvancedOverlayTracksWindowSize);
  
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
  
  OFX::StringParam *_sfMDataNbViews = fetchStringParam(kParamAdvancedSfMDataNbViews);
  OFX::StringParam *_sfMDataNbPoses = fetchStringParam(kParamAdvancedSfMDataNbPoses);
  OFX::StringParam *_sfMDataNbIntrinsics = fetchStringParam(kParamAdvancedSfMDataNbIntrinsics);
  OFX::StringParam *_sfMDataNbStructures = fetchStringParam(kParamAdvancedSfMDataNbStructures);
  OFX::StringParam *_sfMDataNbControlPoints = fetchStringParam(kParamAdvancedSfMDataNbControlPoints);
  
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
  OFX::StringParam *_serializedResults = fetchStringParam(kParamCacheSerializedResults);
  
  //Invalidation Parameters
  OFX::IntParam *_forceInvalidation = fetchIntParam(kParamForceInvalidation);
  OFX::IntParam *_forceInvalidationAtTime = fetchIntParam(kParamForceInvalidationAtTime);
  
  //Process Data
  LocalizerProcessData _processData;
  bool _uptodateParam = false;
  bool _uptodateDescriptor = false;

  //Connected clip index vector
  std::vector<std::size_t> _connectedClipIdx;

  //Output Parameters List
  std::vector<OFX::ValueParam*> _outputParams;

  //Cache
  std::map<OfxTime, std::map<std::size_t, FrameData> > _framesData;

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
   * @brief Set regin of definition for the right input
   * @param[in] args
   * @param[out] rod
   * @return 
   */
  bool getRegionOfDefinition(const OFX::RegionOfDefinitionArguments &args, OfxRectD &rod);

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
   * @brief Try to calibrate the Rig in input from cache data
   */
  void calibrateRig();
  
  /**
   * @brief Load Rig Calibration From a file
   * @param[in] filePath
   */
  void loadRigCalibration(const std::string &filePath);
  
  /**
   * @brief Save Rig Calibration From a file
   * @param[in] filePath
   */
  void saveRigCalibration(const std::string &filePath);
  
  /**
   * @brief Reset all plugin parameters
   */
  void reset();
  
  /**
   * @brief Clear relative poses for each input
   */
  void clearAllRelativePoses();
  
  /**
   * @brief serialize cache data
   */
  void serializeCacheData();
  
  /**
   * @brief Update the member connected clip index collection
   */
  void updateConnectedClipIndexCollection();
  
  /**
   * @brief Update camera output index parameter to fit with clip number
   */
  void updateCameraOutputIndexRange();
  
  /**
   * @brief Update rig UI plugin options
   */
  void updateRigOptions();
  
  /**
   * @brief Update lens distortion UI coefficients, for the chosen input
   * @param[in] input
   */
  void updateLensDistortion(std::size_t input);
  
  /**
   * @brief Update lens distortion UI plugin mode, for the chosen input
   * @param[in] input
   */
  void updateLensDistortionMode(std::size_t input);
  
  /**
   * @brief Update focal length UI plugin mode, for the chosen input
   * @param[in] input
   */
  void updateFocalLength(std::size_t input);
  
  /**
   * @brief Update UI tracking range
   */
  void updateTrackingRangeMode();
  
  /**
   * @brief Update UI Output parameters with localization results
   * @param[in] time
   * @param[in] clipIndex
   * @param[in] locResults
   * @param[in] extractedFeatures
   * @return 
   */
  void updateOutputParamAtTime(double time, 
                                std::size_t clipIndex, 
                                const openMVG::localization::LocalizationResult& locResults, 
                                const std::vector<openMVG::features::SIOPointFeature>& extractedFeatures);
  
  /**
   * @brief Set a pose from the given input
   * @param[in] clipIndex
   * @param[out] subPose
   */
  void getInputSubPose(std::size_t clipIndex, openMVG::geometry::Pose3& subPose);
  
  /**
   * @brief Set intrinsics from the given input and time
   * @param[in] time
   * @param[in] inputClipIdx
   * @param[out] queryIntrinsics
   */
  bool getInputIntrinsics(double time, std::size_t clipIndex, openMVG::cameras::Pinhole_Intrinsic &queryIntrinsics);
  
  /**
   * @brief Set a map of grayscale image from input
   * @param[in] time
   * @param[out] mapInputImage
   * @return 
   */
  bool getInputsInGrayScale(double time, std::map< std::size_t, openMVG::image::Image<unsigned char> > &mapInputImage);

  
  std::size_t getNbConnectedInput() const
  {
    return _connectedClipIdx.size();
  }
  
  std::size_t getDataIndexFromClipIndex(std::size_t inputIndex) const
  {
    auto it = find(_connectedClipIdx.begin(), _connectedClipIdx.end(), inputIndex);
    return std::distance(_connectedClipIdx.begin(), it);
  }
  
  int getOverlayTracksWindowSize() const
  {
    return _overlayTracksWindowSize->getValue();
  }
  
  double getOverlayScaleOrientationRadius() const
  {
    return _overlayFeaturesScaleOrientationRadius->getValue();
  }
  
  const std::map<std::size_t, FrameData> &getFrameDataCache(OfxTime time) const
  {
    return _framesData.at(time);
  }
  
  const FrameData& getOutputFrameDataCache(OfxTime time) const
  {
    return _framesData.at(time).at(_cameraOutputIndex->getValue() - 1);
  }
  
  const openMVG::sfm::SfM_Data& getLocalizerSfMData() const
  {
    return _processData.localizer->getSfMData();
  }
    
  bool hasOverlayDetectedFeatures() const
  {
    return _overlayDetectedFeatures->getValue();
  }
  
  bool hasOverlayMatchedFeatures() const
  {
    return _overlayMatchedFeatures->getValue();
  }
  
  bool hasOverlayResectionFeatures() const
  {
    return _overlayResectionFeatures->getValue();
  }
  
  bool hasOverlayReprojectionError() const
  {
    return _overlayReprojectionError->getValue();
  }
  
  bool hasOverlayReconstructionVisibility() const
  {
    return _overlayReconstructionVisibility->getValue();
  }
  
  bool hasOverlayFeaturesId() const
  {
    return _overlayFeaturesId->getValue();
  }
    
  bool hasOverlayFeaturesScaleOrientation() const
  {
    return _overlayFeaturesScaleOrientation->getValue();
  }
    
  bool hasOverlayTracks() const
  {
    return _overlayTracks->getValue();
  }
  
  bool hasFrameDataCache(OfxTime time) const
  {
    return _framesData.find(time) != _framesData.end();
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
  
  bool hasInput() const
  {
    return (_connectedClipIdx.size() > 0);
  }
    
  bool isRigInInput() const
  {
    return (_connectedClipIdx.size() > 1);
  }
  
  bool isRigModeUnknown()
  {
    return (static_cast<EParamRigMode>(_rigMode->getValue()) == eParamRigModeUnKnown);
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
    _framesData.erase(time);
    for(OFX::ValueParam* outputParam: _outputParams)
      outputParam->deleteKeyAtTime(time);
    serializeCacheData();
  }
  
  void clearOutputParamValues()
  {
    _framesData.clear();
    for(OFX::ValueParam* outputParam: _outputParams)
      outputParam->deleteAllKeys();
    _serializedResults->setValue("");
  }
};


} //namespace Localizer
} //namespace openMVG_ofx
