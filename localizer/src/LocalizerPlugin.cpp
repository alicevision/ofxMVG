#include "LocalizerPlugin.hpp"
#include <stdio.h>
#include <iostream>

LocalizerPlugin::LocalizerPlugin(OfxImageEffectHandle handle) : 
    OFX::ImageEffect(handle)
{
}

void LocalizerPlugin::render(const OFX::RenderArguments &args)
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

bool LocalizerPlugin::isIdentity(const OFX::IsIdentityArguments &args, OFX::Clip * &identityClip, double &identityTime)
{
  return false;
}

void LocalizerPlugin::changedClip(const OFX::InstanceChangedArgs &args, const std::string &clipName)
{
 
}

void LocalizerPlugin::changedParam(const OFX::InstanceChangedArgs &args, const std::string &paramName)
{
  
}
