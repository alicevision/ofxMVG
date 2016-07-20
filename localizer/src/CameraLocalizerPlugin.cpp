#include "CameraLocalizerPlugin.hpp"
#include <stdio.h>
#include <iostream>

namespace openMVG_ofx
{

namespace Localizer
{

CameraLocalizerPlugin::CameraLocalizerPlugin(OfxImageEffectHandle handle) : 
    OFX::ImageEffect(handle)
{
  for(unsigned int input = 0; input < K_MAX_INPUTS; ++input)
  {
    //Source clips
    _srcClip[input] = fetchClip(kClip(input)); 
    
    //Input Parameters
    _inputCalibrationFile[input] = fetchStringParam(kParamInputCalibrationFile(input));
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
    _inputRelativePoseTranslate[input] = fetchDouble3DParam(kParamInputRelativePoseTranslate(input));
    _inputRelativePoseRotate[input] = fetchDouble3DParam(kParamInputRelativePoseRotate(input));
    _inputRelativePoseScale[input] = fetchDouble3DParam(kParamInputRelativePoseScale(input));
  }
}

void CameraLocalizerPlugin::parametersSetup()
{
  
}

void CameraLocalizerPlugin::beginSequenceRender(const OFX::BeginSequenceRenderArguments &args)
{
  // TODO:
  // setup params if not up-to-date
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
  
  // TODO:
  // fetch Images
  // Convert to openMVG Images
  // Call: localize / localizeRig
  // push_back intermediate data if bundle needed
  // setAtTime results into "Camera Output" params
}

bool CameraLocalizerPlugin::isIdentity(const OFX::IsIdentityArguments &args, OFX::Clip * &identityClip, double &identityTime)
{
  return false;
}

void CameraLocalizerPlugin::changedClip(const OFX::InstanceChangedArgs &args, const std::string &clipName)
{
 
}

void CameraLocalizerPlugin::changedParam(const OFX::InstanceChangedArgs &args, const std::string &paramName)
{
  
}


} //namespace Localizer

} //namespace openMVG_ofx