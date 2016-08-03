#include "LensCalibrationPlugin.hpp"
#include <stdio.h>
#include <cassert>
#include <iostream>
#include <algorithm>

namespace openMVG_ofx {
namespace LensCalibration {


LensCalibrationPlugin::LensCalibrationPlugin(OfxImageEffectHandle handle)
  : OFX::ImageEffect(handle)
{}


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
}


bool LensCalibrationPlugin::isIdentity(const OFX::IsIdentityArguments &args, OFX::Clip * &identityClip, double &identityTime)
{
  return false;
}

void LensCalibrationPlugin::changedClip(const OFX::InstanceChangedArgs &args, const std::string &clipName)
{
  
}

void LensCalibrationPlugin::changedParam(const OFX::InstanceChangedArgs &args, const std::string &paramName)
{

}


} //namespace LensCalibration
} //namespace openMVG_ofx