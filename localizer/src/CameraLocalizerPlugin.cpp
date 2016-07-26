#include "CameraLocalizerPlugin.hpp"
#include "Image.hpp"

#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

#include <openMVG/numeric/numeric.h>

#include <stdio.h>
#include <cassert>
#include <iostream>

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
    _inputFocalLengthVarying[input]= fetchBooleanParam(kParamInputFocalLengthVarying(input));
    _inputSensorWidth[input] = fetchDoubleParam(kParamInputSensorWidth(input));
    _inputRelativePoseTranslate[input] = fetchDouble3DParam(kParamInputRelativePoseTranslate(input));
    _inputRelativePoseRotate[input] = fetchDouble3DParam(kParamInputRelativePoseRotate(input));
    _inputRelativePoseScale[input] = fetchDouble3DParam(kParamInputRelativePoseScale(input));
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
      //TODO:
      // tmpParam->_useGuidedMatching = xxx
      // tmpParam->_fDistRatio = xxx

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
  _processData.param->_featurePreset = LocalizerProcessData::getDescriberPreset(static_cast<EParamFeaturesPreset>(_featurePreset->getValue()));
  _processData.param->_refineIntrinsics = false; //TODO: globalBundle
  _processData.param->_visualDebug = _debugFolder->getValue();
  _processData.param->_errorMax = _reprojectionError->getValue();
}

void CameraLocalizerPlugin::beginSequenceRender(const OFX::BeginSequenceRenderArguments &args)
{
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
  
  //Ensure Localizer is correctly initialized
  bool isInit = _processData.localizer->isInit();
  if(!isInit)
  {
    //setPersistentMessage(OFX::Message::eMessageError, "Cannot initialize the camera localizer.");
    std::cerr << "Cannot initialize the camera localizer at frame " << args.time << "." << std::endl;
    return;
  }

  int outputIndex = _cameraOutputIndex->getValue() - 1;
  
  //fetch Image
  if(!_srcClip[outputIndex]->isConnected())
  {
    std::cout << "unavailable clip"  << std::endl;
    return;
  }
  std::cout << "render : fetch input - min range : "  <<  _srcClip[outputIndex]->getFrameRange().min << " - time : " << args.time << std::endl;

  OFX::Image *inputPtr = _srcClip[outputIndex]->fetchImage(args.time);

  if(inputPtr == NULL)
  {
    std::cout << "Input image is NULL" << std::endl;
    return;
  }
  
  std::size_t width;
  std::size_t height;
  openMVG::image::Image<unsigned char> imageGray;
  {
    // Retrieve the input image and convert it to gray 8 bits
    Image<float> inputImage(inputPtr, eOrientationTopDown);
    width = inputPtr->getRegionOfDefinition().x2 - inputPtr->getRegionOfDefinition().x1;
    height = inputPtr->getRegionOfDefinition().y2 - inputPtr->getRegionOfDefinition().y1;
    std::cout << "render : gray scale input" << std::endl;
    imageGray.resize(inputImage.getWidth(), inputImage.getHeight());
//    convertRGB32ToGRAY8(inputImage, imageGray);
    convertGGG32ToGRAY8(inputImage, imageGray);
//    openMVG::image::WriteJpg("/tmp/test.jpg", imageGray);
  }

  openMVG::localization::LocalizationResult localizationResult;
  if(isRigInInput())
  {
    std::cout << "RIG NOT YET IMPLEMENTED" << std::endl;
    //openMVG::geometry::Pose3 rigPose;

    //TODO
  }
  else
  {
    // TODO: support multiple camera model support
    openMVG::cameras::Pinhole_Intrinsic_Radial_K3 queryIntrinsics(width, height);
    EParamLensDistortion lensDistortionType = static_cast<EParamLensDistortion>(_inputLensDistortion[outputIndex]->getValue());
    const bool hasIntrinsics = lensDistortionType == eParamLensDistortionKnown || lensDistortionType == eParamLensDistortionApproximate;
    if(hasIntrinsics)
    {
      std::cout << "Lens distortion to intrinsics" << std::endl;

      double ppx;
      double ppy;
      _inputOpticalCenter[outputIndex]->getValue(ppx, ppy);

      queryIntrinsics.updateFromParams({
        _inputFocalLength[outputIndex]->getValue(),
        ppx,
        ppy,
        _inputLensDistortionCoef1[outputIndex]->getValue(),
        _inputLensDistortionCoef2[outputIndex]->getValue(),
        _inputLensDistortionCoef3[outputIndex]->getValue()
        });
    }

    _processData.localize(imageGray,
                          hasIntrinsics,
                          queryIntrinsics,
                          localizationResult);
  }

  //TODO : push_back intermediate data if bundle needed

  //TODO : Set results at time into output camera parameters
  setPoseToParamsAtTime(
          localizationResult.getPose(),
          args.time,
          _cameraOutputTranslate,
          _cameraOutputRotate,
          _cameraOutputScale);
  
  setIntrinsicsToParamsAtTime(
          localizationResult.getIntrinsics(),
          args.time,
          _inputSensorWidth[outputIndex]->getValue(),
          _cameraOutputFocalLength,
          _cameraOutputOpticalCenter);

  std::cout << "render : fetch output "  << std::endl;

  OFX::Image *outputPtr = _dstClip->fetchImage(args.time);

  if(outputPtr == NULL)
  {
    std::cout << "Input image is NULL" << std::endl;
    return;
  }
  
  Image<float> outputImage(outputPtr, eOrientationTopDown);
    
  std::cout << "render : gray input convert to output F32 "  << std::endl;
  convertGRAY8ToRGB32(imageGray, outputImage);
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
}

void CameraLocalizerPlugin::reset()
{
  _uptodateParam = false;
  _uptodateDescriptor = false;
  
  for(unsigned int input = 0; input < K_MAX_INPUTS; ++input)
  {
    updateLensDistortion(input);
    updateLensDistortionMode(input);
    updateFocalLength(input);
  }
  
  updateRigOptions();
  updateConnectedClipIndexCollection();
  updateCameraOutputIndexRange();
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
  
  bool unknown = (static_cast<EParamRigMode>(_rigMode->getValue()) == eParamRigModeUnKnown);
  _rigCalibrationFile->setIsSecret(unknown);
  for (unsigned int input = 0; input < K_MAX_INPUTS; ++input)
  {
    _inputRelativePoseTranslate[input]->setIsSecret(unknown);
    _inputRelativePoseRotate[input]->setIsSecret(unknown);
    _inputRelativePoseScale[input]->setIsSecret(unknown);
  }
}

void CameraLocalizerPlugin::updateLensDistortion(unsigned int input)
{
  bool unknown = (static_cast<EParamLensDistortion>(_rigMode->getValue()) == eParamLensDistortionUnKnown);
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

void CameraLocalizerPlugin::updateFocalLength(unsigned int input)
{
  bool unknown = (static_cast<EParamFocalLengthMode>(_rigMode->getValue()) == eParamFocalLengthModeUnKnown);
  _inputFocalLength[input]->setIsSecret(unknown);
  _inputFocalLengthVarying[input]->setIsSecret(unknown);
}

} //namespace Localizer
} //namespace openMVG_ofx