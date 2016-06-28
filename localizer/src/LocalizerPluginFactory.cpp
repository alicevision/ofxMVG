#include "LocalizerPluginFactory.hpp"
#include "LocalizerPlugin.hpp"

/**
 * 
 * @param ids
 */
void OFX::Plugin::getPluginIDs(OFX::PluginFactoryArray &ids)
{
  static LocalizerPluginFactory p("openMVG.Localizer", 1, 0);
  ids.push_back(&p);
}  

void LocalizerPluginFactory::describe(OFX::ImageEffectDescriptor& desc)
{
  //Plugin Labels
  desc.setLabels(
    "Localizer",
    "Localizer",
    "openMVG Localizer");
  
  //Plugin grouping
  desc.setPluginGrouping("openMVG");

  //Plugin description
  desc.setPluginDescription("Localizer description");

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
  desc.setTemporalClipAccess(true);
  desc.setRenderTwiceAlways(false);
  desc.setSupportsMultipleClipPARs(false);
}

void LocalizerPluginFactory::describeInContext(OFX::ImageEffectDescriptor& desc, OFX::ContextEnum context)
{
  //Input Clip
  OFX::ClipDescriptor *srcClip = desc.defineClip("main");
  srcClip->addSupportedComponent(OFX::ePixelComponentRGBA);
  srcClip->setTemporalClipAccess(true);
  srcClip->setSupportsTiles(false);
  srcClip->setIsMask(false);
  srcClip->setOptional(false);
  
  
  //Output clip
  OFX::ClipDescriptor *dstClip = desc.defineClip(kOfxImageEffectOutputClipName);
  dstClip->addSupportedComponent(OFX::ePixelComponentRGBA);
  dstClip->setSupportsTiles(false);
}

OFX::ImageEffect* LocalizerPluginFactory::createInstance(OfxImageEffectHandle handle, OFX::ContextEnum context)
{
  return new LocalizerPlugin(handle);
}