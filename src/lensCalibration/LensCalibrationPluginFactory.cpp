#pragma once
#include "LensCalibrationPluginFactory.hpp"
#include "LensCalibrationPluginDefinition.hpp"
#include "LensCalibrationInteract.hpp"

namespace openMVG_ofx {
namespace LensCalibration {

void LensCalibrationPluginFactory::describe(OFX::ImageEffectDescriptor& desc)
{
  //Plugin Labels
  desc.setLabels(
    "LensCalibration",
    "LensCalibration",
    "openMVG LensCalibration");

  //Plugin grouping
  desc.setPluginGrouping("openMVG");

  //Plugin description
  desc.setPluginDescription( //TODO description
    "LensCalibration estimates the best distortion parameters "
    "according to the couple camera/optics of a dataset."
    "\n"
    "The plugin supports video file & folder containing images or "
    "image sequence."
  );

  //Supported contexts
  desc.addSupportedContext(OFX::eContextFilter);
  desc.addSupportedContext(OFX::eContextGeneral);
  desc.addSupportedContext(OFX::eContextPaint);

  //Supported pixel depths
  desc.addSupportedBitDepth(OFX::eBitDepthUByte);
  desc.addSupportedBitDepth(OFX::eBitDepthUShort);
  desc.addSupportedBitDepth(OFX::eBitDepthFloat);

  //Flags
  desc.setSingleInstance(false);
  desc.setHostFrameThreading(false);
  desc.setSupportsMultiResolution(true);
  desc.setSupportsTiles(false);
  desc.setTemporalClipAccess(false);
  desc.setRenderTwiceAlways(false);
  desc.setSupportsMultipleClipPARs(false);

  desc.setOverlayInteractDescriptor( new LensCalibrationOverlayDescriptor);
}

void LensCalibrationPluginFactory::describeInContext(OFX::ImageEffectDescriptor& desc, OFX::ContextEnum context)
{
  //Input Clip
  OFX::ClipDescriptor *srcClip = desc.defineClip(KClipInput);
  srcClip->addSupportedComponent(OFX::ePixelComponentRGBA);
  srcClip->setTemporalClipAccess(false);
  srcClip->setSupportsTiles(false);
  srcClip->setIsMask(false);
  srcClip->setOptional(false);
  

  //Output clip
  OFX::ClipDescriptor *dstClip = desc.defineClip(kOfxImageEffectOutputClipName);
  dstClip->addSupportedComponent(OFX::ePixelComponentRGBA);
  dstClip->setSupportsTiles(false);
}

OFX::ImageEffect* LensCalibrationPluginFactory::createInstance(OfxImageEffectHandle handle, OFX::ContextEnum context)
{
  return new LensCalibrationPlugin(handle);
}

} //namespace LensCalibration
} //namespace openMVG_ofx