#include "CameraLocalizerPlugin.hpp"
#include "../common/Image.hpp"

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/utility.hpp> 

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
  for(std::size_t input = 0; input < K_MAX_INPUTS; ++input)
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
  //reset all plugins options
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
  
  const openMVG::sfm::SfM_Data &sfMData = _processData.localizer->getSfMData();
  
  _sfMDataNbViews->setValue( std::to_string(sfMData.views.size()) );
  _sfMDataNbPoses->setValue( std::to_string(sfMData.poses.size()) ); 
  _sfMDataNbIntrinsics->setValue( std::to_string(sfMData.intrinsics.size()) ); 
  _sfMDataNbStructures->setValue( std::to_string(sfMData.structure.size()) );
  _sfMDataNbControlPoints->setValue( std::to_string(sfMData.control_points.size()) );
}

bool CameraLocalizerPlugin::getRegionOfDefinition(const OFX::RegionOfDefinitionArguments &args, OfxRectD &rod)
{
  if((getNbConnectedInput() <= 0) || 
          (!_srcClip[_cameraOutputIndex->getValue()-1]->isConnected()))
  {
    // rod = OfxRectD{0,0,-1,-1};
    throw OFX::Exception::Suite(kOfxStatFailed);
  }
  rod = _srcClip[_cameraOutputIndex->getValue()-1]->getRegionOfDefinition(args.time);
  
  return true;
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
  std::cout << "render : [info] time: " << args.time << std::endl;
  std::cout << "render : [info] fieldToRender: " << args.fieldToRender << std::endl;
  std::cout << "render : [info] renderQualityDraft: " << args.renderQualityDraft << std::endl;
  std::cout << "render : [info] renderScale: " << args.renderScale.x << ", " << args.renderScale.y << std::endl;
  std::cout << "render : [info] interactiveRenderStatus: " << args.interactiveRenderStatus << std::endl;
  std::cout << "render : [info] args.renderWindow: (" << args.renderWindow.x1 << ", " << args.renderWindow.y1 << "), (" << args.renderWindow.x2 << ", "  << args.renderWindow.y2 << ")" << std::endl;
  std::cout << "render : [info] output clip index : " << _cameraOutputIndex->getValue() - 1 << std::endl;
  
  if(abort())
  {
    return;
  }
  
  //Check if no connected input
  if(getNbConnectedInput() <= 0)
  {
    std::cout << "render : [quit] no input" << std::endl;
    return;
  }
  
  //Get output index
  int outputClipIndex = _cameraOutputIndex->getValue() - 1;
  if(!_srcClip[outputClipIndex]->isConnected())
  {
    std::cerr << "render : [error] invalid output index" << std::endl;
    return;
  }
 
  //Process Data initialization
  std::map<std::size_t, openMVG::localization::LocalizationResult> mapLocResults;
  std::map<std::size_t, openMVG::cameras::Pinhole_Intrinsic_Radial_K3> mapIntrinsics; //TODO : Change for different camera type
  std::map<std::size_t, openMVG::image::Image<unsigned char> > mapImageGray;
  
  //Collect Images in input
  if(!getInputsInGrayScale(args.time, mapImageGray))
  {
    std::cerr << "render : [error] can't collect images in input" << std::endl;
    return;
  }
  
  try
  {  
    //Check if the frame has already been computed
    if(!_alwaysComputeFrame->getValue() &&
        hasFrameDataCache(args.time)) 
    {
      //Don't launch the tracker if we already have a keyFrame at current time.
      //We only need to provide the output image to the host.
      std::cout << "render : [stopped] frame already computed at frame : " << args.time << std::endl;
      
      for(auto &inputFrameData : getFrameDataCache(args.time))
      {
        mapLocResults[inputFrameData.first] = inputFrameData.second.localizationResult;
        mapIntrinsics[inputFrameData.first] = inputFrameData.second.localizationResult.getIntrinsics(); //TODO: remove and read intrinsics from output parameters
      }
      std::cout << "render : [stopped] cache loaded at time : " << args.time << std::endl;
    }
    else
    {
      //Ensure Localizer is correctly initialized
      if(!_processData.localizer->isInit())
      {
        std::cerr << "render : [error] Cannot initialize the camera localizer at frame " << args.time << "." << std::endl;
        return;
      }
      
      //Collect Query Data
      std::vector<bool> vecQueryHasIntrinsics(getNbConnectedInput());
      std::vector< openMVG::cameras::Pinhole_Intrinsic_Radial_K3 > vecQueryIntrinsics(getNbConnectedInput()); //TODO : Change for different camera type
      std::vector< std::pair<std::size_t, std::size_t> > vecQueryImageSize(getNbConnectedInput()); 
      std::vector< std::unique_ptr<openMVG::features::Regions> > vecQueryRegions(getNbConnectedInput());
      std::vector< openMVG::geometry::Pose3 > vecQuerySubPoses(getNbConnectedInput() - 1); //Don't save main camera

      for(std::size_t input = 0; input < getNbConnectedInput(); ++input)
      {
        std::size_t clipIndex = _connectedClipIdx[input];

        if(input > 0) //We don't save the main camera relative pose (for the moment)
        {
          getInputSubPose(clipIndex,  vecQuerySubPoses[input - 1]);
        }
        vecQueryImageSize[input] = std::make_pair<std::size_t, std::size_t>(mapImageGray[clipIndex].Width(), mapImageGray[clipIndex].Height());  
        vecQueryIntrinsics[input] = openMVG::cameras::Pinhole_Intrinsic_Radial_K3 (mapImageGray[clipIndex].Width(), mapImageGray[clipIndex].Height());  //TODO : Change for different camera type
        vecQueryHasIntrinsics[input] = getInputIntrinsics(args.time, clipIndex, vecQueryIntrinsics[input]);
      }
      
      if(abort())
      {
        return;
      }
      
      //Extract features
      _processData.extractFeatures(mapImageGray, vecQueryRegions);
      
      if(abort())
      {
        return;
      }
      
      //Localization Process
      if(isRigInInput() && !isRigModeUnknown())
      {
        std::cout << "render : [localization] Known RIG" << std::endl;
        openMVG::geometry::Pose3 mainCameraPose;
        std::vector<openMVG::localization::LocalizationResult> vecLocResults;

        _processData.localizeRig(vecQueryRegions,
                                vecQueryImageSize,
                                vecQueryIntrinsics,
                                vecQuerySubPoses,
                                mainCameraPose,
                                vecLocResults);
        
        for(std::size_t input = 0; input < vecLocResults.size(); ++input)
        {
          std::size_t clipIndex = _connectedClipIdx[input];
          mapLocResults[clipIndex] = vecLocResults[input];
        }
      }
      else
      {
        if(isRigInInput())
          std::cout << "render : [localization] Simple mode : unknown RIG" << std::endl;
        else
          std::cout << "render : [localization] Simple mode : one camera" << std::endl;
                
        for(std::size_t input = 0; input < getNbConnectedInput(); ++input)
        {
          std::size_t clipIndex = _connectedClipIdx[input];
          _processData.localize(vecQueryRegions[input],
                                vecQueryImageSize[input],
                                vecQueryHasIntrinsics[input],
                                vecQueryIntrinsics[input],
                                mapLocResults[clipIndex]);
        }
      }
      
      if(abort())
      {
        return;
      }
      
      //Create frame temp cache structure
      std::map<std::size_t, FrameData> frameDataCache; 
      
      for(std::size_t output = 0; output < getNbConnectedInput(); ++output)
      {
        std::cout << "render : [write] output  : " << output << std::endl;
        std::size_t clipIndex = _connectedClipIdx[output];
        
        //Update frame temp cache
        frameDataCache[clipIndex].extractedFeatures = dynamic_cast<const openMVG::features::SIFT_Regions*>(vecQueryRegions[output].get())->Features();
        frameDataCache[clipIndex].localizationResult = mapLocResults[clipIndex];
        frameDataCache[clipIndex].undistortedPt2D = mapLocResults[clipIndex].retrieveUndistortedPt2D();
         
        if(mapLocResults[clipIndex].isValid())
        {
          std::cout << "render : [write] update output UI parameters " << std::endl;
          updateOutputParamAtTime(args.time, 
                                  clipIndex, 
                                  mapLocResults[clipIndex], 
                                  frameDataCache[clipIndex].extractedFeatures);
          
          std::cout << "render : [write] update intrinsics " << std::endl;
          mapIntrinsics[clipIndex] = mapLocResults[clipIndex].getIntrinsics();
        }
      }
      
      std::cout << "render : [cache] update with frame temp cache " << std::endl;
      //Update cache with frame temp cache
      {
        if(hasFrameDataCache(args.time))
        {
          //Lock before updating the existing frame data
          for(auto &outputDataCache : frameDataCache)
          {
            std::lock_guard<std::mutex> guard(_framesData[args.time][outputDataCache.first].mutex);
             _framesData[args.time][outputDataCache.first] = outputDataCache.second;
          }
        }
        else
        {
          //Create a new cache at this time
          _framesData[args.time] = frameDataCache;
        }
      }
    }
  }
  catch(std::exception &e)
  {
    this->sendMessage(OFX::Message::eMessageError, "cameralocalization.render", e.what());
  }
  
  //Update Overlay
  std::cout << "render : [overlay] redraw"  << std::endl;
  this->redrawOverlays();

  //Fetch Output image
  std::cout << "render : [output clip] fetch"  << std::endl;
  OFX::Image *outputPtr = _dstClip->fetchImage(args.time);
  if(outputPtr == NULL)
  {
    std::cout << "render : [output clip] is NULL" << std::endl;
    return;
  }
  Common::Image<float> outputImage(outputPtr, Common::eOrientationTopDown);
  
  // TODO: always undistort (fill vecIntrinsics from params)
  if(mapLocResults[outputClipIndex].isValid())
  {
    openMVG::image::Image<unsigned char> undistortedImage;
    std::cout << "render : [output clip] compute undistorted "  << std::endl;
    openMVG::cameras::UndistortImage(mapImageGray[outputClipIndex], &mapIntrinsics[outputClipIndex], undistortedImage);
    std::cout << "render : [output clip] convert and copy "  << std::endl;
    convertGRAY8ToRGB32(undistortedImage, outputImage);
  }
  else
  {
    std::cout << "render : [output clip] no calibration "  << std::endl;
    convertGRAY8ToRGB32(mapImageGray[outputClipIndex], outputImage);
  }

  if(_alwaysComputeFrame->getValue())
  {
    invalidRender();
  }
}


bool CameraLocalizerPlugin::isIdentity(const OFX::IsIdentityArguments &args, OFX::Clip * &identityClip, double &identityTime)
{
  return false;
}

void CameraLocalizerPlugin::changedClip(const OFX::InstanceChangedArgs &args, const std::string &clipName)
{
  std::cout << "----------" << std::endl;
  std::cout << "changedClip: " << clipName << std::endl;
  std::cout << "  args.reason: " << int(args.reason) << std::endl;
  std::cout << "  args.time: " << args.time << std::endl;
  std::cout << "----------" << std::endl;
  
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
    OFX::ImageEffectHostDescription* desc = OFX::getImageEffectHostDescription();
   
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
  
  //Want save Rig Calibration
  if(paramName == kParamRigCalibrationWantSave)
  {
    _rigCalibrationSavePath->setIsSecret(!_rigCalibrationWantSave->getValue());
  }
  
  //Rig calibration
  if(paramName == kParamRigCalibration)
  {
    calibrateRig();
    return;
  }
  
  //Load Rig calibration
  if(paramName == kParamRigCalibrationFile)
  {
    loadRigCalibration(_rigCalibrationFile->getValue());
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
  if(paramName == kParamCacheClearCurrentFrame)
  {
    clearOutputParamValuesAtTime(args.time);
    
    invalidRenderAtTime(args.time);
    return;
  }
  
  //Clear All
  if(paramName == kParamCacheClear)
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


void CameraLocalizerPlugin::calibrateRig()
{
  openMVG::rig::Rig rigCalibration;
  std::vector< std::vector<openMVG::localization::LocalizationResult> > dataPerCamera(getNbConnectedInput());

  //Collect cache data per camera
  for(auto &framesDataAtTime : _framesData)
  {
    for(std::size_t cameraIndex = 0; cameraIndex < framesDataAtTime.second.size(); ++cameraIndex)
    {
      auto &cameraFrameDataAtTime = framesDataAtTime.second[cameraIndex];
      if(cameraFrameDataAtTime.isLocalized())
      {
        dataPerCamera[cameraIndex].push_back(framesDataAtTime.second[cameraIndex].localizationResult);
      }
    }
  }

  //Set tracking result per camera
  for(std::size_t cameraIndex = 0; cameraIndex < dataPerCamera.size(); ++cameraIndex)
  {
    rigCalibration.setTrackingResult(dataPerCamera[cameraIndex], cameraIndex);
  }

  if(!rigCalibration.initializeCalibration())
  {
    sendMessage(OFX::Message::eMessageWarning, "rig.calibration.process",
        "Unable to find a proper initialization for the relative poses for Rig calibration  ! Aborting...");
    return;
  }

  if(!rigCalibration.optimizeCalibration())
  {
    sendMessage(OFX::Message::eMessageWarning, "rig.calibration.process",
        "Unable to optimize the relative poses for Rig calibration ! Aborting...");
    return;
  }
  else
  {
    std::cout << "rig calibration : [write] clear cache" << std::endl;

    //clear all keys
    clearOutputParamValues();
    invalidRender();

    //update subposes
    const std::vector<openMVG::geometry::Pose3>& subposes = rigCalibration.getRelativePoses();

    for(std::size_t pose = 0; pose < subposes.size(); ++pose)
    {
      std::size_t input = pose + 1; //don't have main camera

      std::cout << "rig calibration : [write] update relative pose : " << pose << " for input : " << input << std::endl; 

      const auto rotate = subposes[pose].rotation();
      const auto center = subposes[pose].center();

      _inputRelativePoseRotateM1[input]->setValue(rotate(0,0), rotate(0,1), rotate(0,2));
      _inputRelativePoseRotateM2[input]->setValue(rotate(1,0), rotate(1,1), rotate(1,2));
      _inputRelativePoseRotateM3[input]->setValue(rotate(2,0), rotate(2,1), rotate(2,2));
      _inputRelativePoseCenter[input]->setValue(center(0), center(1), center(2));
    }

    // save the rig calibration (subposes)
    if(_rigCalibrationWantSave->getValue())
    {
      std::cout << "rig calibration : [write] save calibration file" << std::endl;
      std::string filePath = _rigCalibrationSavePath->getValue();
      if(rigCalibration.saveCalibration(filePath))
      {
        sendMessage(OFX::Message::eMessageMessage, "rig.calibration.process",
            "Rig calibration succeed ! Relative Poses have been update, calibration file save : " + filePath);
      }
      else
      {
        sendMessage(OFX::Message::eMessageWarning, "rig.calibration.process",
            "Rig calibration succeed but can't write calibration file : " + filePath);
      }
    }
    else
    {
      sendMessage(OFX::Message::eMessageMessage, "rig.calibration.process",
          "Rig calibration succeed ! Relative Poses have been update.");
    }

    _rigMode->setValue((int)EParamRigMode::eParamRigModeKnown);
  }
}

void CameraLocalizerPlugin::loadRigCalibration(const std::string &filePath)
{
  std::vector<openMVG::geometry::Pose3> subposes;
  openMVG::rig::loadRigCalibration(filePath, subposes);

  if(subposes.size() >= K_MAX_INPUTS)
  {
    sendMessage(OFX::Message::eMessageWarning, "rig.subpose.file",
            "The number of cameras in the RIG file contains more cameras than the plugin supports.");
    return;
  }
  else if(subposes.size() != (getNbConnectedInput() - 1))
  {
    sendMessage(OFX::Message::eMessageWarning, "rig.subpose.file",
            "The number of cameras in the RIG file does not match the number of connected input clips.");
    return;
  }

  const std::size_t nbSubPoses = std::min(subposes.size(), std::size_t(K_MAX_INPUTS - 1));

  for(std::size_t i = 0; i < nbSubPoses; ++i)
  {
    std::size_t input = _connectedClipIdx[i + 1];

    if(input == 0) //Don't set the main camera
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
}
  
void CameraLocalizerPlugin::reset()
{
  _uptodateParam = false;
  _uptodateDescriptor = false;
  
  updateConnectedClipIndexCollection();
  
  for(std::size_t input = 0; input < K_MAX_INPUTS; ++input)
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
  for(std::size_t input = 0; input < K_MAX_INPUTS; ++input)
  {
      if(_srcClip[input]->isConnected())
      {
        _connectedClipIdx.push_back(input);
      }
  }
}

void CameraLocalizerPlugin::updateCameraOutputIndexRange()
{
  std::size_t min = 0;
  std::size_t max = 0;
  
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
    
    std::size_t value = _cameraOutputIndex->getValue();
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
    unknown = isRigModeUnknown();
    _rigMode->setIsSecret(false);
    _rigCalibrationWantSave->setIsSecret(!unknown);
    _rigCalibrationSavePath->setIsSecret(!unknown);
    if(unknown)
    {
      _rigCalibrationSavePath->setIsSecret(!_rigCalibrationWantSave->getValue());
    }
  }
  else
  {
    _rigMode->setIsSecret(true);
    _rigCalibrationWantSave->setIsSecret(true);
    _rigCalibrationSavePath->setIsSecret(true);
  }
  
  _rigCalibrationFile->setIsSecret(unknown);
  _rigCalibration->setIsSecret(!unknown);
  
  for (std::size_t input = 0; input < K_MAX_INPUTS; ++input)
  {
    _inputRelativePoseRotateM1[input]->setIsSecret(unknown);
    _inputRelativePoseRotateM2[input]->setIsSecret(unknown);
    _inputRelativePoseRotateM3[input]->setIsSecret(unknown);
    _inputRelativePoseCenter[input]->setIsSecret(unknown);
    _inputGroupRelativePose[input]->setIsSecret(unknown);
  }
}

void CameraLocalizerPlugin::updateLensDistortion(std::size_t input)
{
  bool unknown = (static_cast<EParamLensDistortion>(_inputLensDistortion[input]->getValue()) == eParamLensDistortionUnKnown);
  _inputLensDistortionMode[input]->setIsSecret(unknown);
  _inputLensDistortionCoef1[input]->setIsSecret(unknown);
  _inputLensDistortionCoef2[input]->setIsSecret(unknown);
  _inputLensDistortionCoef3[input]->setIsSecret(unknown);
  _inputLensDistortionCoef4[input]->setIsSecret(unknown);
}

void CameraLocalizerPlugin::updateLensDistortionMode(std::size_t input)
{
  EParamLensDistortionMode distortionMode = static_cast<EParamLensDistortionMode>(_inputLensDistortionMode[input]->getValue());
  //TODO : display the good number of coefficient per distortion mode
}

void CameraLocalizerPlugin::updateTrackingRangeMode()
{
  bool enabled = (static_cast<EParamTrackingRangeMode>(_trackingRangeMode->getValue()) == eParamRangeCustom);
  _trackingRangeMin->setEnabled(enabled);
  _trackingRangeMax->setEnabled(enabled);
}

void CameraLocalizerPlugin::updateFocalLength(std::size_t input)
{
  bool unknown = (static_cast<EParamFocalLengthMode>(_inputFocalLengthMode[input]->getValue()) == eParamFocalLengthModeUnKnown);
  _inputFocalLength[input]->setIsSecret(unknown);
  _inputFocalLengthVarying[input]->setIsSecret(unknown);
}

void CameraLocalizerPlugin::updateOutputParamAtTime(double time, 
                                                    std::size_t clipIndex, 
                                                    const openMVG::localization::LocalizationResult& locResults, 
                                                    const std::vector<openMVG::features::SIOPointFeature>& extractedFeatures) 
{
  setPoseToParamsAtTime(
          locResults.getPose(),
          time,
          _cameraOutputTranslate[clipIndex],
          _cameraOutputRotate[clipIndex],
          _cameraOutputScale[clipIndex]);


  setIntrinsicsToParamsAtTime(
          locResults.getIntrinsics(),
          time,
          _inputSensorWidth[clipIndex]->getValue(),
          _cameraOutputFocalLength[clipIndex],
          _cameraOutputOpticalCenter[clipIndex]);


  setStatToParamsAtTime(
          locResults,
          extractedFeatures, //read only
          time,
          _outputStatErrorMean[clipIndex],
          _outputStatErrorMin[clipIndex],
          _outputStatErrorMax[clipIndex],
          _outputStatNbMatchedImages[clipIndex],
          _outputStatNbDetectedFeatures[clipIndex],
          _outputStatNbMatchedFeatures[clipIndex],
          _outputStatNbInlierFeatures[clipIndex]);
}

void CameraLocalizerPlugin::getInputSubPose(std::size_t clipIndex, openMVG::geometry::Pose3& subPose)
{
  auto &rotate = subPose.rotation();
  auto &center = subPose.center();

  _inputRelativePoseRotateM1[clipIndex]->getValue(rotate(0,0), rotate(0,1), rotate(0,2));
  _inputRelativePoseRotateM2[clipIndex]->getValue(rotate(1,0), rotate(1,1), rotate(1,2));
  _inputRelativePoseRotateM3[clipIndex]->getValue(rotate(2,0), rotate(2,1), rotate(2,2));
  _inputRelativePoseCenter[clipIndex]->getValue(center(0), center(1), center(2));
}

bool CameraLocalizerPlugin::getInputIntrinsics(double time, std::size_t clipIndex, openMVG::cameras::Pinhole_Intrinsic &queryIntrinsics)
{
  EParamLensDistortion lensDistortionType = static_cast<EParamLensDistortion>(_inputLensDistortion[clipIndex]->getValue());
  
  const bool hasIntrinsics = lensDistortionType == eParamLensDistortionKnown || lensDistortionType == eParamLensDistortionApproximate;
  
  if(!hasIntrinsics)
  {
    return false;
  }
  
  double ppx;
  double ppy;

  _inputOpticalCenter[clipIndex]->getValue(ppx, ppy);

  //TODO : Change for different camera type
  queryIntrinsics.updateFromParams({
    _inputFocalLength[clipIndex]->getValueAtTime(time),
    ppx,
    ppy,
    _inputLensDistortionCoef1[clipIndex]->getValue(),
    _inputLensDistortionCoef2[clipIndex]->getValue(),
    _inputLensDistortionCoef3[clipIndex]->getValue()
  });

  return true;
}

bool CameraLocalizerPlugin::getInputsInGrayScale(double time, std::map< std::size_t, openMVG::image::Image<unsigned char> > &mapInputImage)
{
  for(std::size_t input = 0; input < getNbConnectedInput(); ++input)
  {
    std::size_t clipIndex = _connectedClipIdx[input];
    OFX::Image *inputPtr = _srcClip[clipIndex]->fetchImage(time);

    if(inputPtr == NULL)
    {
      return false;
    }

    Common::Image<float> inputImage(inputPtr, Common::eOrientationTopDown);
    mapInputImage[clipIndex] = openMVG::image::Image<unsigned char>(inputImage.getWidth(), inputImage.getHeight());
    
    if(_inputIsGrayscale[clipIndex]->getValue())
    {
      convertGGG32ToGRAY8(inputImage, mapInputImage[clipIndex]);
    }
    else
    {
      convertRGB32ToGRAY8(inputImage, mapInputImage[clipIndex]);
    }
  }
  return true;
}

} //namespace Localizer
} //namespace openMVG_ofx