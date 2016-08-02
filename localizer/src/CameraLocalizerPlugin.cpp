#include "CameraLocalizerPlugin.hpp"
#include "Image.hpp"

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

#include <openMVG/numeric/numeric.h>

#include <stdio.h>
#include <cassert>
#include <iostream>
#include <algorithm>

namespace openMVG_ofx {
namespace Localizer {

namespace bfs = boost::filesystem;

CameraLocalizerPlugin::CameraLocalizerPlugin(OfxImageEffectHandle handle)
  : OFX::ImageEffect(handle)
{
  for(unsigned int input = 0; input < K_MAX_INPUTS; ++input)
  {
    //Source clips
    _srcClip[input] = fetchClip(kClip(input)); 
    
    //Input Parameters
    _inputIsGrayscale[input] = fetchBooleanParam(kParamInputIsGrayscale(input));
    _inputLensCalibrationFile[input] = fetchStringParam(kParamInputLensCalibrationFile(input));
    _inputLensDistortion[input] = fetchChoiceParam(kParamInputDistortion(input));
    _inputLensDistortionMode[input] = fetchChoiceParam(kParamInputDistortionMode(input));
    _inputLensDistortionCoef1[input] = fetchDoubleParam(kParamInputDistortionCoef1(input));
    _inputLensDistortionCoef2[input] = fetchDoubleParam(kParamInputDistortionCoef2(input));
    _inputLensDistortionCoef3[input] = fetchDoubleParam(kParamInputDistortionCoef3(input));
    _inputLensDistortionCoef4[input] = fetchDoubleParam(kParamInputDistortionCoef4(input));
    _inputOpticalCenter[input] = fetchDouble2DParam(kParamInputOpticalCenter(input));
    _inputFocalLengthMode[input] = fetchChoiceParam(kParamInputFocalLengthMode(input));
    _inputFocalLength[input] = fetchDoubleParam(kParamInputFocalLength(input));
    _inputFocalLengthVarying[input] = fetchBooleanParam(kParamInputFocalLengthVarying(input));
    _inputSensorWidth[input] = fetchDoubleParam(kParamInputSensorWidth(input));
    _inputRelativePoseRotateM1[input] = fetchDouble3DParam(kParamInputRelativePoseRotateM1(input));
    _inputRelativePoseRotateM2[input] = fetchDouble3DParam(kParamInputRelativePoseRotateM2(input));
    _inputRelativePoseRotateM3[input] = fetchDouble3DParam(kParamInputRelativePoseRotateM3(input));
    _inputRelativePoseCenter[input] = fetchDouble3DParam(kParamInputRelativePoseCenter(input));
    _inputGroupRelativePose[input] = fetchGroupParam(kParamInputGroupRelativePose(input));
    
    //Output Parameters
    _cameraOutputTranslate[input] = fetchDouble3DParam(kParamOutputTranslate(input));
    _cameraOutputRotate[input] = fetchDouble3DParam(kParamOutputRotate(input));
    _cameraOutputScale[input] = fetchDouble3DParam(kParamOutputScale(input));
    _cameraOutputOpticalCenter[input] = fetchDouble2DParam(kParamOutputOpticalCenter(input));
    _cameraOutputFocalLength[input] = fetchDoubleParam(kParamOutputFocalLength(input));
    _cameraOutputNear[input] = fetchDoubleParam(kParamOutputNear(input));
    _cameraOutputFar[input] = fetchDoubleParam(kParamOutputFar(input));
    _cameraOutputLensDistortionCoef1[input] = fetchDoubleParam(kParamOutputDistortionCoef1(input));
    _cameraOutputLensDistortionCoef2[input] = fetchDoubleParam(kParamOutputDistortionCoef2(input));
    _cameraOutputLensDistortionCoef3[input] = fetchDoubleParam(kParamOutputDistortionCoef3(input));
    _cameraOutputLensDistortionCoef4[input] = fetchDoubleParam(kParamOutputDistortionCoef4(input));
    _outputStatErrorMean[input] = fetchDoubleParam(kParamOutputStatErrorMean(input));
    _outputStatErrorMin[input] = fetchDoubleParam(kParamOutputStatErrorMin(input));
    _outputStatErrorMax[input] = fetchDoubleParam(kParamOutputStatErrorMax(input));
    _outputStatNbMatchedImages[input] = fetchDoubleParam(kParamOutputStatNbMatchedImages(input));
    _outputStatNbDetectedFeatures[input] = fetchDoubleParam(kParamOutputStatNbDetectedFeatures(input));
    _outputStatNbMatchedFeatures[input] = fetchDoubleParam(kParamOutputStatNbMatchedFeatures(input));
    _outputStatNbInlierFeatures[input] = fetchDoubleParam(kParamOutputStatNbInlierFeatures(input));
    
    _outputParams.push_back(_cameraOutputTranslate[input]);
    _outputParams.push_back(_cameraOutputRotate[input]);
    _outputParams.push_back(_cameraOutputScale[input]);
    _outputParams.push_back(_cameraOutputFocalLength[input]);
    _outputParams.push_back(_cameraOutputOpticalCenter[input]);
    _outputParams.push_back(_outputStatErrorMean[input]);
    _outputParams.push_back(_outputStatErrorMin[input]);
    _outputParams.push_back(_outputStatErrorMax[input]);
    _outputParams.push_back(_outputStatNbMatchedImages[input]);
    _outputParams.push_back(_outputStatNbDetectedFeatures[input]);
    _outputParams.push_back(_outputStatNbMatchedFeatures[input]);
    _outputParams.push_back(_outputStatNbInlierFeatures[input]);
  }
  //reset all plugins options;
  reset();
}

void CameraLocalizerPlugin::parametersSetup()
{
  //Get Features type enum
  EParamFeaturesType describer = static_cast<EParamFeaturesType>(_featureType->getValue());

  switch(describer)
  {
    //Set SIFT an SIFTAndCCTag parameters
    case eParamFeaturesTypeSIFT :
    case eParamFeaturesTypeSIFTAndCCTag :
    {
      if(!_uptodateDescriptor) 
      {
        openMVG::localization::VoctreeLocalizer *tmpLoc;
        tmpLoc = new openMVG::localization::VoctreeLocalizer(_reconstructionFile->getValue(),
                                                             _descriptorsFolder->getValue(),
                                                             _voctreeFile->getValue(),
                                                             _voctreeWeightsFile->getValue()
#if HAVE_CCTAG
                                                             ,(eParamFeaturesTypeSIFTAndCCTag == describer)
#endif
                                                             );
        _processData.localizer.reset(tmpLoc);
      }

      openMVG::localization::VoctreeLocalizer::Parameters *tmpParam;
      tmpParam = new openMVG::localization::VoctreeLocalizer::Parameters();
      _processData.param.reset(tmpParam);
      tmpParam->_algorithm = LocalizerProcessData::getAlgorithm(static_cast<EParamAlgorithm>(_algorithm->getValue()));;
      tmpParam->_numResults = _nbImageMatch->getValue();
      tmpParam->_maxResults = _maxResults->getValue();
      tmpParam->_numCommonViews = 3; // TODO: unused
      tmpParam->_ccTagUseCuda = false;
      tmpParam->_matchingError = _matchingError->getValue();
      tmpParam->_useGuidedMatching = _useGuidedMatching->getValue();

    } break;
    
#if HAVE_CCTAG
    //Set CCTag only parameters
    case eParamFeaturesTypeCCTag :
    {
      if(!_uptodateDescriptor)
      {
        openMVG::localization::CCTagLocalizer *tmpLoc;
        tmpLoc = new openMVG::localization::CCTagLocalizer(_reconstructionFile->getValue(),
                                                           _descriptorsFolder->getValue());
        _processData.localizer.reset(tmpLoc);
      }

      openMVG::localization::CCTagLocalizer::Parameters *tmpParam;
      tmpParam = new openMVG::localization::CCTagLocalizer::Parameters();
      _processData.param.reset(tmpParam);
      tmpParam->_nNearestKeyFrames = _cctagNbNearestKeyFrames->getValue();
      
    } break;
#endif
    
    //Invalid describer type
    default : throw std::invalid_argument("Unrecognized Features Type : " + std::to_string(describer));
  }
  //
  assert(_processData.localizer);
  assert(_processData.param);
  
  //Set other common parameters
  _processData.param->_matchingEstimator = LocalizerProcessData::getMatchingEstimator(static_cast<EParamEstimatorMatching>(_estimatorMatching->getValue()));
  _processData.param->_resectionEstimator = LocalizerProcessData::getResectionEstimator(static_cast<EParamEstimatorResection>(_estimatorResection->getValue()));
  _processData.param->_featurePreset = LocalizerProcessData::getDescriberPreset(static_cast<EParamFeaturesPreset>(_featurePreset->getValue()));
  _processData.param->_refineIntrinsics = false; //TODO: globalBundle
  _processData.param->_visualDebug = _debugFolder->getValue();
  _processData.param->_errorMax = _reprojectionError->getValue();
  _processData.param->_fDistRatio = _distanceRatio->getValue();
}

void CameraLocalizerPlugin::beginSequenceRender(const OFX::BeginSequenceRenderArguments &args)
{
  std::cout << "sequence render : begin : " << timeLineGetTime() << std::endl;
  if(!_uptodateParam || !_uptodateDescriptor)
  {
   parametersSetup();
   _uptodateParam = true;
   _uptodateDescriptor = true;
  }
}

void CameraLocalizerPlugin::endSequenceRender(const OFX::EndSequenceRenderArguments &args)
{
  // TODO:
  // Bundle if needed and multiple images collected.
}

void CameraLocalizerPlugin::render(const OFX::RenderArguments &args)
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

  //Get output index
  int outputIndex = _cameraOutputIndex->getValue() - 1;

  if(!_srcClip[outputIndex]->isConnected())
  {
    std::cerr << "invalid output index" << std::endl;
    return;
  }
  
  //Localization Process
  bool localized = false;
  openMVG::localization::LocalizationResult localizationResult;
  openMVG::cameras::Pinhole_Intrinsic_Radial_K3 intrinsics;
  
  std::vector<openMVG::image::Image<unsigned char> > vecImageGray(getNbConnectedInput());
  for(unsigned int i = 0; i < getNbConnectedInput(); ++i)
  {
    unsigned int input = _connectedClipIdx[i];
    if(!getInputInGrayScale(args.time, input, vecImageGray[i]))
    {
      return;
    }
  }

  try
  {
  
  // Don't launch the tracker if we already have a keyFrame at current time.
  // We only need to provide the output image to nuke.
  if(!_alwaysComputeFrame->getValue() &&
      hasAllOutputParamKey(args.time)  && 
      hasCachedLocalizationResults(args.time) // TODO: remove and read intrinsics from output parameters
      )
  {
    std::cout << "render : stopped : frame already computed, time : " << args.time << std::endl;
    localized = true;
    if(hasCachedLocalizationResults(args.time))
      localizationResult = getCachedLocalizationResults(args.time);
    // TODO: read intrinsics from output params
    intrinsics = localizationResult.getIntrinsics();
  }
  else
  {
    //Ensure Localizer is correctly initialized
    bool isInit = _processData.localizer->isInit();
    if(!isInit)
    {
      std::cerr << "Cannot initialize the camera localizer at frame " << args.time << "." << std::endl;
      return;
    }

    //Collect Data
    std::vector<openMVG::geometry::Pose3 > vecSubPoses(getNbConnectedInput() - 1); //Don't save main camera
    std::vector<openMVG::cameras::Pinhole_Intrinsic_Radial_K3 > vecQueryIntrinsics; //TODO : Change for different camera type
    std::vector<bool> vecHasIntrinsics(getNbConnectedInput());

    for(unsigned int i = 0; i < getNbConnectedInput(); ++i)
    {
      unsigned int input = _connectedClipIdx[i];

      if(i > 0) //Don't save main camera
      {
        auto &rotate = vecSubPoses[i - 1].rotation();
        auto &center = vecSubPoses[i - 1].center();

        _inputRelativePoseRotateM1[input]->getValue(rotate(0,0), rotate(0,1), rotate(0,2));
        _inputRelativePoseRotateM2[input]->getValue(rotate(1,0), rotate(1,1), rotate(1,2));
        _inputRelativePoseRotateM3[input]->getValue(rotate(2,0), rotate(2,1), rotate(2,2));

        _inputRelativePoseCenter[input]->getValue(center(0), center(1), center(2));
      }

      //TODO : Change for different camera type
      openMVG::cameras::Pinhole_Intrinsic_Radial_K3 queryIntrinsics(vecImageGray[i].Width(), vecImageGray[i].Height());

      vecHasIntrinsics[input] = getInputIntrinsics(args.time, input, queryIntrinsics);

      vecQueryIntrinsics.push_back(queryIntrinsics);
    }

    if(isRigInInput())
    {
      openMVG::geometry::Pose3 mainCameraPose;

      localized = _processData.localizeRig(vecImageGray,
                              vecQueryIntrinsics,
                              vecSubPoses,
                              mainCameraPose);
      if(localized)
      {
        //TODO : add transformation for other camera of the rig

        setPoseToParamsAtTime(
                 mainCameraPose,
                 args.time,
                 _cameraOutputTranslate[outputIndex],
                 _cameraOutputRotate[outputIndex],
                 _cameraOutputScale[outputIndex]);
      }
      intrinsics = vecQueryIntrinsics[outputIndex];
    }
    else
    {
      const openMVG::image::Image<unsigned char>& imageGray = vecImageGray.front();
      const std::pair<std::size_t, std::size_t> queryImageSize(std::make_pair(imageGray.Width(), imageGray.Height()));
      std::unique_ptr<openMVG::features::Regions> queryRegions;

      if(abort())
        return;

      _processData.extractFeatures(imageGray, queryRegions);
      _extractedFeaturesAtTime[args.time] = dynamic_cast<const openMVG::features::SIFT_Regions*>(queryRegions.get())->Features();
      // We can start to display the 2D points
      this->redrawOverlays();

      if(abort())
        return;

      localized = _processData.localize(queryRegions,
                                        queryImageSize,
                                        vecHasIntrinsics.front(),
                                        vecQueryIntrinsics.front(),
                                        localizationResult);

      if(localized)
      {
        setPoseToParamsAtTime(
                localizationResult.getPose(),
                args.time,
                _cameraOutputTranslate[outputIndex],
                _cameraOutputRotate[outputIndex],
                _cameraOutputScale[outputIndex]);

        setIntrinsicsToParamsAtTime(
                localizationResult.getIntrinsics(),
                args.time,
                _inputSensorWidth[outputIndex]->getValue(),
                _cameraOutputFocalLength[outputIndex],
                _cameraOutputOpticalCenter[outputIndex]);

        setStatToParamsAtTime(
                localizationResult,
                _extractedFeaturesAtTime.at(args.time),
                args.time,
                _outputStatErrorMean[outputIndex],
                _outputStatErrorMin[outputIndex],
                _outputStatErrorMax[outputIndex],
                _outputStatNbMatchedImages[outputIndex],
                _outputStatNbDetectedFeatures[outputIndex],
                _outputStatNbMatchedFeatures[outputIndex],
                _outputStatNbInlierFeatures[outputIndex]);

        _localizationResultsAtTime[args.time] = localizationResult;
        intrinsics = localizationResult.getIntrinsics();
      }
    }

    if(!localized)
    {
      clearOutputParamValuesAtTime(args.time);
    }
  }
  
  } catch(std::exception& e)
  {
    this->sendMessage(OFX::Message::eMessageError, "cameralocalization.render", e.what());
  }

  std::cout << "render : fetch output "  << std::endl;

  OFX::Image *outputPtr = _dstClip->fetchImage(args.time);
  if(outputPtr == NULL)
  {
    std::cout << "Output image is NULL" << std::endl;
    return;
  }

  Image<float> outputImage(outputPtr, eOrientationTopDown);
  if(localized)
  {
    openMVG::image::Image<unsigned char> undistortedImage;
    openMVG::cameras::UndistortImage(vecImageGray[outputIndex], &intrinsics, undistortedImage);
    convertGRAY8ToRGB32(undistortedImage, outputImage);
  }
  else
  {
    convertGRAY8ToRGB32(vecImageGray[outputIndex], outputImage);
  }

  if(_alwaysComputeFrame->getValue())
  {
    invalidRender();
  }
  this->redrawOverlays();
}


bool CameraLocalizerPlugin::isIdentity(const OFX::IsIdentityArguments &args, OFX::Clip * &identityClip, double &identityTime)
{
  return false;
}

void CameraLocalizerPlugin::changedClip(const OFX::InstanceChangedArgs &args, const std::string &clipName)
{
  //a clip have been changed by the user or the plugin
  if(args.reason != OFX::InstanceChangeReason::eChangeTime)
  {
    _uptodateParam = false;
    updateConnectedClipIndexCollection();
    updateCameraOutputIndexRange();

    updateRigOptions();
    _trackingButton->setEnabled(hasInput());
  }
}

void CameraLocalizerPlugin::changedParam(const OFX::InstanceChangedArgs &args, const std::string &paramName)
{
  //A parameter change
  _uptodateParam = false;
  
  //Change output index
  if((paramName == kParamOutputIndex) && (args.reason == OFX::InstanceChangeReason::eChangeUserEdit))
  {
    if(!_srcClip[_cameraOutputIndex->getValue() - 1]->isConnected())
    {
      int min;
      int max;
      _cameraOutputIndex->getRange(min, max);
      _cameraOutputIndex->setValue(min);
    }
    return;
  }
  
  //Change reconstruction file
  if(paramName == kParamReconstructionFile)
  {
    bfs::path matchPath = bfs::path(_reconstructionFile->getValue()).parent_path() / "_mvg_build" / "matches";
    if(bfs::is_directory(matchPath))
      _descriptorsFolder->setValue(matchPath.string());
    _uptodateDescriptor = false;
    return;
  }
  
  //Change reconstruction data
  if((paramName == kParamDescriptorsFolder) 
          || (paramName == kParamVoctreeFile) 
          || (paramName == kParamAdvancedVoctreeWeights))
  {
    _uptodateDescriptor = false;
    return;
  }
  
  //Rig Mode change
  if(paramName == kParamRigMode)
  {
    updateRigOptions();
    return;
  }
  
  if(paramName == kParamRigCalibrationFile)
  {
    std::vector<openMVG::geometry::Pose3> subposes;
    openMVG::rig::loadRigCalibration(_rigCalibrationFile->getValue(), subposes);
    
    if(subposes.size() >= K_MAX_INPUTS)
    {
      sendMessage(OFX::Message::eMessageWarning, "rig.subpose.file",
              "The number of cameras in the RIG file contains more cameras than the plugin supports.");
    }
    else if(subposes.size() != (getNbConnectedInput() - 1))
    {
      sendMessage(OFX::Message::eMessageWarning, "rig.subpose.file",
              "The number of cameras in the RIG file does not match the number of connected input clips.");
      return;
    }
    
    const std::size_t nbSubPoses = std::min(subposes.size(), std::size_t(K_MAX_INPUTS - 1));
    
    for(unsigned int i = 0; i < nbSubPoses; ++i)
    {
      unsigned int input = _connectedClipIdx[i + 1];
      
      if(input == 0)
      {
        continue;
      }
      
      const auto rotate = subposes[i].rotation();
      const auto center = subposes[i].center();

      _inputRelativePoseRotateM1[input]->setValue(rotate(0,0), rotate(0,1), rotate(0,2));
      _inputRelativePoseRotateM2[input]->setValue(rotate(1,0), rotate(1,1), rotate(1,2));
      _inputRelativePoseRotateM3[input]->setValue(rotate(2,0), rotate(2,1), rotate(2,2));
      _inputRelativePoseCenter[input]->setValue(center(0), center(1), center(2));
    }
    return;
  }

  //Tracking Range Mode change
  if(paramName == kParamTrackingRangeMode)
  {
    updateTrackingRangeMode();
    return;
  }
  
  //Tracking button
  if(paramName == kParamTrackingTrack)
  {
    //TODO
    return;
  }
  
  //Clear Current Frame
  if(paramName == kParamOutputClearCurrentFrame)
  {
    clearOutputParamValuesAtTime(args.time);
    
    invalidRenderAtTime(args.time);
    return;
  }
  
  //Clear All
  if(paramName == kParamOutputClear)
  {
    clearOutputParamValues();
    
    invalidRender();
    return;
  }
  
  //Input Parameter
  std::size_t input = getParamInputId(paramName);
  
  if((input >= 0) && (input < K_MAX_INPUTS))
  {
      if(paramName == kParamInputDistortion(input))
      {
        updateLensDistortion(input);
        return;
      }
      
      if(paramName == kParamInputDistortionMode(input))
      {
        updateLensDistortionMode(input);
        return;
      }
      
      if(paramName == kParamInputFocalLengthMode(input))
      {
        updateFocalLength(input);
        return;
      }
      
      if(paramName == kParamInputLensCalibrationFile(input))
      {
        openMVG::cameras::Pinhole_Intrinsic_Radial_K3 intrinsic; //TODO : multiple camera type
        openMVG::dataio::readCalibrationFromFile(_inputLensCalibrationFile[input]->getValue(), intrinsic);
        
        _inputFocalLength[input]->setValue(intrinsic.focal());
        _inputOpticalCenter[input]->setValue(intrinsic.principal_point()(0), intrinsic.principal_point()(1));
        _inputLensDistortionMode[input]->setValue(LocalizerProcessData::getLensDistortionModelFromEnum(intrinsic.getType()));
        
        const std::vector<double>& parameters = intrinsic.getDistortionParams();
    
        if(parameters.size() > 0)
          _inputLensDistortionCoef1[input]->setValue(parameters[0]);
        if(parameters.size() > 1)
          _inputLensDistortionCoef2[input]->setValue(parameters[1]);
        if(parameters.size() > 2)
          _inputLensDistortionCoef3[input]->setValue(parameters[2]);
        if(parameters.size() > 3)
          _inputLensDistortionCoef4[input]->setValue(parameters[3]);
        if(parameters.size() > 4)
          std::cerr << "[CameraLocalizer] Warning: There is some ignored distortion parameters." << std::endl;

        return;
      }
  }
  
}

void CameraLocalizerPlugin::reset()
{
  _uptodateParam = false;
  _uptodateDescriptor = false;
  
  updateConnectedClipIndexCollection();
  
  for(unsigned int input = 0; input < K_MAX_INPUTS; ++input)
  {
    updateLensDistortion(input);
    updateLensDistortionMode(input);
    updateFocalLength(input);
  }
  
  updateRigOptions();
  updateTrackingRangeMode();
  updateCameraOutputIndexRange();
  
  _trackingButton->setEnabled(hasInput());
}

void CameraLocalizerPlugin::updateConnectedClipIndexCollection()
{
  _connectedClipIdx.clear();
  for(unsigned int input = 0; input < K_MAX_INPUTS; ++input)
  {
      if(_srcClip[input]->isConnected())
      {
        _connectedClipIdx.push_back(input);
      }
  }
}

void CameraLocalizerPlugin::updateCameraOutputIndexRange()
{
  unsigned int min = 0;
  unsigned int max = 0;
  
  if(_connectedClipIdx.empty())
  {
    //no connected clip
    _cameraOutputIndex->setEnabled(false);
    _cameraOutputIndex->setValue(0);
  }
  else
  {
    _cameraOutputIndex->setEnabled(true);
    min = _connectedClipIdx.front() + 1;
    max = _connectedClipIdx.back() + 1;
    
    unsigned int value = _cameraOutputIndex->getValue();
    if((value < min) || (value > max) || (!_srcClip[value - 1]->isConnected()))
    {
      _cameraOutputIndex->setValue(min);
    }
  }
  _cameraOutputIndex->setRange(min, max);
  _cameraOutputIndex->setDisplayRange(min, max);
}

void CameraLocalizerPlugin::updateRigOptions()
{
  bool unknown = true;
  
  if(isRigInInput())
  {
    unknown = (static_cast<EParamRigMode>(_rigMode->getValue()) == eParamRigModeUnKnown);
    _rigMode->setIsSecret(false);
  }
  else
  {
    _rigMode->setIsSecret(true);
  }
  
  _rigCalibrationFile->setIsSecret(unknown);
  for (unsigned int input = 0; input < K_MAX_INPUTS; ++input)
  {
    _inputRelativePoseRotateM1[input]->setIsSecret(unknown);
    _inputRelativePoseRotateM2[input]->setIsSecret(unknown);
    _inputRelativePoseRotateM3[input]->setIsSecret(unknown);
    _inputRelativePoseCenter[input]->setIsSecret(unknown);
    _inputGroupRelativePose[input]->setIsSecret(unknown);
  }
}

void CameraLocalizerPlugin::updateLensDistortion(unsigned int input)
{
  bool unknown = (static_cast<EParamLensDistortion>(_inputLensDistortion[input]->getValue()) == eParamLensDistortionUnKnown);
  _inputLensDistortionMode[input]->setIsSecret(unknown);
  _inputLensDistortionCoef1[input]->setIsSecret(unknown);
  _inputLensDistortionCoef2[input]->setIsSecret(unknown);
  _inputLensDistortionCoef3[input]->setIsSecret(unknown);
  _inputLensDistortionCoef4[input]->setIsSecret(unknown);
}

void CameraLocalizerPlugin::updateLensDistortionMode(unsigned int input)
{
  EParamLensDistortionMode distortionMode = static_cast<EParamLensDistortionMode>(_inputLensDistortionMode[input]->getValue());

}

void CameraLocalizerPlugin::updateTrackingRangeMode()
{
  bool enabled = (static_cast<EParamTrackingRangeMode>(_trackingRangeMode->getValue()) == eParamRangeCustom);
  _trackingRangeMin->setEnabled(enabled);
  _trackingRangeMax->setEnabled(enabled);
}

void CameraLocalizerPlugin::updateFocalLength(unsigned int input)
{
  bool unknown = (static_cast<EParamFocalLengthMode>(_inputFocalLengthMode[input]->getValue()) == eParamFocalLengthModeUnKnown);
  _inputFocalLength[input]->setIsSecret(unknown);
  _inputFocalLengthVarying[input]->setIsSecret(unknown);
}

bool CameraLocalizerPlugin::getInputIntrinsics(double time, unsigned int inputClipIdx, openMVG::cameras::Pinhole_Intrinsic &queryIntrinsics)
{
  EParamLensDistortion lensDistortionType = static_cast<EParamLensDistortion>(_inputLensDistortion[inputClipIdx]->getValue());
  
  const bool hasIntrinsics = lensDistortionType == eParamLensDistortionKnown || lensDistortionType == eParamLensDistortionApproximate;
  
  if(!hasIntrinsics)
  {
    return false;
  }
  
  double ppx;
  double ppy;

  _inputOpticalCenter[inputClipIdx]->getValue(ppx, ppy);

  //TODO : Change for different camera type
  queryIntrinsics.updateFromParams({
    _inputFocalLength[inputClipIdx]->getValueAtTime(time),
    ppx,
    ppy,
    _inputLensDistortionCoef1[inputClipIdx]->getValue(),
    _inputLensDistortionCoef2[inputClipIdx]->getValue(),
    _inputLensDistortionCoef3[inputClipIdx]->getValue()
  });

  return true;
}

bool CameraLocalizerPlugin::getInputInGrayScale(double time, unsigned int inputClipIdx, openMVG::image::Image<unsigned char> &outputImage)
{
  OFX::Image *inputPtr = _srcClip[inputClipIdx]->fetchImage(time);

  if(inputPtr == NULL)
  {
    return false;
  }
  
  Image<float> inputImage(inputPtr, eOrientationTopDown);
  
  outputImage.resize(inputImage.getWidth(), inputImage.getHeight());
  
  if(_inputIsGrayscale[inputClipIdx]->getValue())
  {
    convertGGG32ToGRAY8(inputImage, outputImage);
  }
  else
  {
    convertRGB32ToGRAY8(inputImage, outputImage);
  }
  
  return true;
}

} //namespace Localizer
} //namespace openMVG_ofx